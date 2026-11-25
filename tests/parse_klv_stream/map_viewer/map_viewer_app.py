import argparse, base64, io, json, logging, math, os, socket, sys, threading, time, urllib.error, urllib.request, webbrowser
from collections import deque
from flask import Flask, Response, abort, jsonify, request

BASE = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(os.path.expanduser("~"), "GPSMapCache")
KLV_CORNER_ORDER = (2, 3, 4, 5)   # packet ids of KLV Point 1..4 = image corners TL, TR, BR, BL
MAX_CORNER_OFFSET = 0.0745        # KLV corner offsets saturate at 0.075 deg when a point is out of range
LAT_LIMIT, LON_LIMIT = 89.999, 179.999   # KLV 'cannot compute' values sit at the range limit (90 / 180): reject them
RANGE_MAX = [10000.0]             # m: frame center farther than this from the camera is not drawn (0 = off), from --max-range
MIN_CORNERS = 2                   # corners are drawn when at least this many are valid
EST_RANGE = [1000.0]              # m: estimated visible area is extended at most this far from the camera (--est-range)
BIG_AREA = [2000.0]               # m: a received 4-corner footprint with a side / depth above this is not trusted: the estimate is shown (0 = off, --big-area)
EST_HOLD = [10.0]                 # s: the estimated corners are kept this long unless the gimbal / camera moved (--est-hold)
EST_ANGLE = [3.0]                 # deg: a change of any gimbal / platform angle above this refreshes the estimate at once (--est-angle)
EST_MOVE = [10.0]                 # m: camera moved farther than this since the last estimate -> refresh at once (--est-move)
EST_STATE = {}                    # last estimate: time, valid-corner mask, camera, angles, estimated corners
CORNER_PAIR = {0: 3, 3: 0, 1: 2, 2: 1}   # TL<->BL, TR<->BR: corners on the same vertical edge
CAM_DEFAULT = []                  # [lat, lon, alt] from --cam, used when the stream has no camera position
GROUND_ALT = [0.0]                # ground elevation (MSL) under the camera, from --ground-alt
START = []                        # [lat, lon, zoom] from --start, initial map view until GPS data arrives
PREFETCH_HALF_M = 1500            # half side of the download area: 1500 m -> 3x3 km
PREFETCH_ZOOMS = range(14, 20)    # zoom levels 14..19
NET_DOWN = [0.0]                  # time until which the network is considered down (avoids waiting for a timeout per tile)
TILES = {
    "sat": ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", "image/jpeg"),
    "osm": ("https://tile.openstreetmap.org/{z}/{x}/{y}.png", "image/png"),
}

app = Flask(__name__, static_folder=os.path.join(BASE, "static"), static_url_path="/static")
logging.getLogger("werkzeug").setLevel(logging.ERROR)
cond = threading.Condition()
state = {"seq": 0, "msg": "{}"}
# Video preview: the C++ app connects to --video-port (TCP) and sends [4-byte length][JPEG] frames; /video re-serves them as MJPEG
vcond = threading.Condition()
vbuf = deque()                    # (arrival time, jpeg): the latest frames, served by /video
PLACEHOLDER_JPG = base64.b64decode("/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDABALDA4MChAODQ4SERATGCgaGBYWGDEjJR0oOjM9PDkzODdASFxOQERXRTc4UG1RV19iZ2hnPk1xeXBkeFxlZ2P/2wBDARESEhgVGC8aGi9jQjhCY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2P/wAARCAACAAIDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDlqKKKYj//2Q==")   # 2x2 grey frame, keeps the HTTP connection alive when idle


def good(p):
    return (all(math.isfinite(v) for v in p) and abs(p[0]) < LAT_LIMIT and abs(p[1]) < LON_LIMIT
            and (abs(p[0]) > 1e-9 or abs(p[1]) > 1e-9))


def dist_m(a, b):
    """Great-circle distance in meters between two (lat, lon) points."""
    r = math.pi / 180
    h = (math.sin((b[0] - a[0]) * r / 2) ** 2 +
         math.cos(a[0] * r) * math.cos(b[0] * r) * math.sin((b[1] - a[1]) * r / 2) ** 2)
    return 12742000.0 * math.asin(min(1.0, math.sqrt(h)))


# ---------------------------------------------------------------- estimated visible area (only when 2 or 3 corners are valid)
def estimate_by_rays(cam, corners, agl=None, vfov=None):
    """Fallback (no complete top/bottom edge received, or AGL / VFOV unknown). Estimate the missing corners. Direction: always the ray camera -> received corner on the same vertical edge
    (so it never disagrees with the received points). Distance along that ray:
      - missing TOP corners (received bottom ones): the depression angle drops by VFOV; if that is above the horizon
        (or AGL / VFOV unknown) the corner goes to EST_RANGE, otherwise at agl / tan(angle), at most EST_RANGE.
      - missing BOTTOM corners (received top ones): the depression angle grows by VFOV, so the corner is NEARER to the
        camera: agl / tan(angle), or the point below the camera (distance 0) when it points straight down or
        AGL / VFOV are unknown.
    Returns 4 (lat, lon) points or None."""
    if cam is None:
        return None
    k_lat, k_lon = 110540.0, 111320.0 * math.cos(math.radians(cam[0]))
    have = bool(agl and agl > 0 and vfov and vfov > 0)
    out = []
    for i, p in enumerate(corners):
        if p is not None:
            out.append(p)
            continue
        q = corners[CORNER_PAIR[i]]
        if q is None:
            return None
        dy, dx = (q[0] - cam[0]) * k_lat, (q[1] - cam[1]) * k_lon
        d = math.hypot(dx, dy)
        if d < 1e-3:
            return None
        if i in (0, 1):                                      # top corner missing, q is the bottom one
            t = EST_RANGE[0]
            if have:
                th = math.atan2(agl, d) - math.radians(vfov)
                if th > 1e-3:
                    t = min(EST_RANGE[0], agl / math.tan(th))
            t = max(t, d)
        else:                                                # bottom corner missing, q is the top one
            t = 0.0
            if have:
                th = math.atan2(agl, d) + math.radians(vfov)
                if th < math.pi / 2 - 1e-3:
                    t = min(d, agl / math.tan(th))
        out.append((cam[0] + dy / d * t / k_lat, cam[1] + dx / d * t / k_lon))
    return out


def estimate_area(cam, corners, agl=None, vfov=None, center=None):
    """Estimate the missing corners from the received ones (received corners are never changed).
    Used when a complete horizontal edge was received (P3+P4 or P1+P2) and AGL and VFOV are known:
      - forward direction f = camera -> received frame center (the optical axis on the ground); without a valid center,
        camera -> midpoint of the received edge;
      - the depression angle of the received edge (atan(AGL / forward distance)) gives the optical-axis depression:
        axis = edge angle -/+ VFOV/2;
      - the effective tan(HFOV/2) is taken from the measured width of the received edge (no HFOV tag needed);
      - every missing corner is the ray (1, +-tanH, +-tanV) of a pinhole camera at that depression, intersected with the
        ground (flat, AGL high). A ray that does not hit the ground, or hits beyond EST_RANGE, is cut at EST_RANGE forward.
    Roll is ignored. Otherwise falls back to estimate_by_rays()."""
    if cam is None:
        return None
    have = bool(agl and agl > 0 and vfov and vfov > 0)
    edge = None
    if have:
        if corners[2] is not None and corners[3] is not None:
            edge = (2, 3, 1)                                 # received bottom edge (image-down = +1)
        elif corners[0] is not None and corners[1] is not None:
            edge = (0, 1, -1)                                # received top edge
    if edge is None:
        return estimate_by_rays(cam, corners, agl, vfov)
    k_lat, k_lon = 110540.0, 111320.0 * math.cos(math.radians(cam[0]))
    enu = lambda p: ((p[1] - cam[1]) * k_lon, (p[0] - cam[0]) * k_lat)
    ia, ib, sz0 = edge
    A, B = enu(corners[ia]), enu(corners[ib])
    mid = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
    ref = mid
    if center is not None:
        c = enu(center)
        if math.hypot(*c) > 1e-3:
            ref = c                                          # direction of the received frame center
    nr = math.hypot(*ref)
    if nr < 1e-3:
        return estimate_by_rays(cam, corners, agl, vfov)
    f = (ref[0] / nr, ref[1] / nr)                           # forward (unit, east/north)
    fm = mid[0] * f[0] + mid[1] * f[1]                       # forward distance of the received edge
    if fm < 1e-3:
        return estimate_by_rays(cam, corners, agl, vfov)
    r = (f[1], -f[0])                                        # right of forward
    lm = abs((A[0] - B[0]) * r[0] + (A[1] - B[1]) * r[1]) / 2   # measured half width of the received edge
    V = math.radians(vfov) / 2
    tv = math.tan(V)
    delta = math.atan2(agl, fm) - sz0 * V                    # depression of the optical axis
    down0 = math.sin(delta) + sz0 * tv * math.cos(delta)
    if down0 < 1e-6:
        return estimate_by_rays(cam, corners, agl, vfov)
    th = lm * down0 / agl                                    # effective tan(HFOV/2)
    out = []
    for i, p in enumerate(corners):
        if p is not None:
            out.append(p)
            continue
        sz = -1 if i < 2 else 1                              # TL, TR above the center; BR, BL below
        qx, qy = enu(corners[CORNER_PAIR[i]])
        sigma = 1.0 if qx * r[0] + qy * r[1] >= 0 else -1.0  # same side as the received corner on this vertical edge
        fwd = math.cos(delta) - sz * tv * math.sin(delta)
        down = math.sin(delta) + sz * tv * math.cos(delta)
        sc = agl / down if down > 1e-6 else math.inf
        if fwd > 1e-6:
            sc = min(sc, EST_RANGE[0] / fwd)
        if not math.isfinite(sc):
            return estimate_by_rays(cam, corners, agl, vfov)
        F, L = sc * fwd, sigma * sc * th
        out.append((cam[0] + (f[1] * F + r[1] * L) / k_lat, cam[1] + (f[0] * F + r[0] * L) / k_lon))
    return out


def area_size(c):
    """Largest of: top edge, bottom edge, mean depth of a 4-corner footprint (m)."""
    return max(dist_m(c[0], c[1]), dist_m(c[3], c[2]), (dist_m(c[0], c[3]) + dist_m(c[1], c[2])) / 2)


def fin(v):
    return v if v is not None and math.isfinite(v) else None


def angdiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def held_estimate(cam, corners, agl, fov, center, att):
    """The center jitters a lot, so the estimated (missing) corners are not recomputed on every packet: they are kept for
    EST_HOLD seconds. They are refreshed at once when the gimbal / platform angles changed by more than EST_ANGLE degrees,
    the camera moved more than EST_MOVE meters, or the set of valid corners changed. Received corners are always live.
    Without the attitude tags (ids 16..18) only the timer and the camera move can trigger a refresh."""
    now = time.monotonic()
    mask = tuple(c is not None for c in corners)
    st = EST_STATE
    moved = bool(st and cam and st.get("cam") and dist_m(cam, st["cam"]) > EST_MOVE[0])
    turned = bool(st and att and st.get("att") and max(angdiff(a, b) for a, b in zip(att, st["att"])) > EST_ANGLE[0])
    if not st or st["mask"] != mask or now - st["t"] >= EST_HOLD[0] or moved or turned:
        full = estimate_area(cam, corners, agl, fov[1] if fov else None, center)
        if full is None:
            st.clear()
            return None
        st.clear()
        st.update(t=now, mask=mask, cam=cam, att=att, miss={i: full[i] for i in range(4) if corners[i] is None})
    return [corners[i] if corners[i] is not None else st["miss"][i] for i in range(4)]


def parse(text):
    pts = {}
    for item in text.split(";"):
        try:
            i, la, lo = item.split(",")
            pts[int(i)] = (float(la), float(lo))
        except ValueError:
            pass
    return pts


def build(pts):
    # Optional extras: 10 = camera lat,lon | 11 = altitude (MSL) | 12 = HFOV,VFOV
    #                  13 = target lat,lon | 14 = target elevation (MSL) | 15 = slant range
    #                  16 = platform heading,pitch | 17 = platform roll,sensor rel. roll | 18 = sensor rel. azimuth,elevation
    # Payload and target are validated on their own: a bad point never hides a good one.
    # Footprint: corners are drawn when at least MIN_CORNERS are valid; the center only when all 4 are valid.
    # The estimated area is computed ONLY when 2 or 3 corners are valid (from a complete P1+P2 or P3+P4 edge); with 4 valid corners the data is used as is.
    pts = dict(pts)
    cam = pts.pop(10, None)
    alt = pts.pop(11, (None,))[0]
    fov = pts.pop(12, None)
    tgt = pts.pop(13, None)
    talt = pts.pop(14, (None,))[0]
    rng = pts.pop(15, (None,))[0]
    a16, a17, a18 = pts.pop(16, None), pts.pop(17, None), pts.pop(18, None)
    # Compass widget: platform heading (id 16, 1st value) and sensor relative azimuth (id 18, 1st value)
    compass = {"hdg": fin(a16[0]) if a16 else None, "raz": fin(a18[0]) if a18 else None}
    if all(v is None for v in compass.values()):
        compass = None
    att = None                                       # only used to detect a gimbal / platform angle change
    if a16 and a17 and a18:
        att = (a16[0], a16[1], a17[0], a18[0], a18[1], a17[1])   # heading, pitch, roll, rel. azimuth, rel. elevation, rel. roll
        if not all(math.isfinite(v) for v in att):
            att = None
    if cam is None and CAM_DEFAULT:
        cam = tuple(CAM_DEFAULT[:2])
        if alt is None and len(CAM_DEFAULT) > 2:
            alt = CAM_DEFAULT[2]
    if not (pts or cam or tgt or rng is not None):
        return None                                  # empty packet

    cam = cam if cam and good(cam) else None
    if alt is not None and not math.isfinite(alt):
        alt = None
    if alt is not None:
        alt -= GROUND_ALT[0]                         # MSL -> height above ground (AGL)
    tgt = tgt if tgt and good(tgt) else None         # target is independent of everything else
    if talt is not None:
        talt = talt - GROUND_ALT[0] if math.isfinite(talt) else None
    if rng is not None and not math.isfinite(rng):
        rng = None
    # Footprint (frame center + 4 corners)
    if 6 in pts:                                     # legacy: 1 = aircraft, 2 = center, 3..6 = corners
        drone, center, ids = pts.get(1), pts.get(2), (3, 4, 5, 6)
    else:                                            # KLV: 1 = center, 2..5 = corners
        drone, center, ids = None, pts.get(1), KLV_CORNER_ORDER
    drone = drone if drone and good(drone) else None

    had_center = center is not None
    base = center                                    # KLV center is only the base for the corner offsets
    if base is not None and not good(base):
        base = None                                  # sentinel (90/180), NaN or (0,0): payload has no ground intersection
    elif base is not None and cam and RANGE_MAX[0] > 0 and dist_m(cam, base) > RANGE_MAX[0]:
        base = None                                  # camera looks near the horizon: center is meaningless
    corners = []
    for i in ids:
        p = pts.get(i)
        ok = (base is not None and p is not None and good(p)
              and abs(p[0] - base[0]) < MAX_CORNER_OFFSET
              and abs(p[1] - base[1]) < MAX_CORNER_OFFSET)
        corners.append(p if ok else None)            # None = this corner is not drawable
    n = sum(c is not None for c in corners)
    if n < MIN_CORNERS:
        corners, n = [None] * 4, 0
    full = n == 4
    full_corners = corners
    # A received footprint that is too large (camera near the horizon) is inaccurate: P1/P2 are dropped and the area is
    # estimated from the received bottom edge P3+P4 (if that is not possible, the received footprint is kept, flagged).
    big = full and BIG_AREA[0] > 0 and area_size(corners) > BIG_AREA[0]
    if big:
        corners, n, full = [None, None, corners[2], corners[3]], 2, False
    center = base if full else None                  # center is drawn only when all 4 corners are valid
    center_lost = had_center and not full

    est = None
    if n in (2, 3):                                  # 2 or 3 valid corners: estimate the missing ones from a complete P1+P2 / P3+P4 edge
        est = held_estimate(cam, corners, alt, fov, base, att)   # direction from the frame center, held for EST_HOLD s
    else:
        EST_STATE.clear()
    if big and est is None:                          # cannot estimate: trust the received footprint, flagged as oversize
        corners, n, full, center, center_lost = full_corners, 4, True, base, False

    if not (center or drone or cam or tgt or rng is not None or n):
        return {"valid": False}
    return {"valid": True, "drone": drone, "center": center,
            "camera": cam, "alt": alt, "hfov": fov[0] if fov else None,
            "target": tgt, "talt": talt, "range": rng,
            "center_lost": center_lost, "ncorners": n, "est": est, "oversize": big, "compass": compass,
            "corners": corners, "labels": ["P1", "P2", "P3", "P4"]}


def udp_loop(sock):
    while True:
        raw, _ = sock.recvfrom(4096)
        msg = build(parse(raw.decode(errors="ignore").strip()))
        if msg:
            with cond:
                state["seq"] += 1
                state["msg"] = json.dumps(msg)
                cond.notify_all()


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(min(1 << 16, n - len(buf)))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def video_conn(conn):
    with conn:
        while True:
            hdr = recv_exact(conn, 4)
            if hdr is None:
                return
            n = int.from_bytes(hdr, "big")
            if n < 4 or n > 8_000_000:                      # corrupt stream: drop the connection
                return
            jpg = recv_exact(conn, n)
            if jpg is None:
                return
            if jpg[:2] == b"\xff\xd8" and jpg[-2:] == b"\xff\xd9":   # complete JPEG only
                with vcond:
                    vbuf.append((time.monotonic(), jpg))
                    while len(vbuf) > 2:                       # only the latest frames are needed
                        vbuf.popleft()
                    vcond.notify_all()


def video_server(srv):
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=video_conn, args=(conn,), daemon=True).start()


@app.route("/video")
def video():
    def part(jpg):
        return (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() +
                b"\r\n\r\n" + jpg + b"\r\n")

    def gen():
        last_t, idle_since = None, time.monotonic()
        while True:
            out = None
            with vcond:
                if not vbuf or vbuf[-1][0] == last_t:
                    vcond.wait(timeout=0.5)               # woken as soon as a new frame arrives
                if vbuf and vbuf[-1][0] != last_t:
                    last_t, out = vbuf[-1]                # newest frame, no artificial delay
                newest = vbuf[-1][1] if vbuf else None
            now = time.monotonic()
            if out is not None:
                idle_since = now
            elif now - idle_since > 3:
                # No new frame for 3 s: resend the last one (or a placeholder). The write fails once the
                # browser has closed the connection, so idle generator threads never pile up.
                idle_since, out = now, (newest or PLACEHOLDER_JPG)
            if out is not None:
                yield part(out)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@app.route("/stream")
def stream():
    def gen():
        last = -1
        while True:
            with cond:
                cond.wait_for(lambda: state["seq"] != last, timeout=15)
                seq, msg = state["seq"], state["msg"]
            if seq == last:
                yield ": ping\n\n"
                continue
            last = seq
            yield f"data: {msg}\n\n"
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


# ---------------------------------------------------------------- tiles: cache, download, overzoom
def tile_path(layer, z, x, y):
    return os.path.join(CACHE, layer, str(z), str(x), f"{y}.img")


def is_cached(layer, z, x, y):
    p = tile_path(layer, z, x, y)
    return os.path.exists(p) and os.path.getsize(p) > 100


def read_cached(layer, z, x, y):
    if not is_cached(layer, z, x, y):
        return None
    with open(tile_path(layer, z, x, y), "rb") as f:
        return f.read()


def download_tile(layer, z, x, y, force=False):
    """Download a tile from the network and store it in the cache. Returns bytes or None."""
    if not force and time.time() < NET_DOWN[0]:
        return None                                   # network recently lost: don't wait for timeouts again
    url, _ = TILES[layer]
    data, err, conn_fail = None, None, False
    for _ in range(2):
        try:
            req = urllib.request.Request(url.format(z=z, x=x, y=y), headers={"User-Agent": "GPSMapApp/1.0"})
            with urllib.request.urlopen(req, timeout=8) as r:
                body = r.read()
                if r.status == 200 and r.headers.get("Content-Type", "").startswith("image/") and len(body) > 100:
                    data = body
                    break
                err = f"response is not an image ({r.status}, {len(body)} bytes)"
        except urllib.error.HTTPError as e:           # server answered with an error: network is up, tile just missing
            err, conn_fail = e, False
            break
        except Exception as e:                        # could not connect
            err, conn_fail = e, True
    if data is None:
        if conn_fail:
            NET_DOWN[0] = time.time() + 20
        print(f"[tile] {layer}/{z}/{x}/{y} failed: {err}")
        return None
    path = tile_path(layer, z, x, y)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.{threading.get_ident()}.tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return data


def overzoom(layer, z, x, y):
    """Tile missing: crop + upscale from a lower-zoom cached tile (up to 4 levels up)."""
    try:
        from PIL import Image
    except ImportError:
        return None
    for dz in range(1, 5):
        if z - dz < 0:
            break
        data = read_cached(layer, z - dz, x >> dz, y >> dz)
        if data is None:
            continue
        try:
            im = Image.open(io.BytesIO(data)).convert("RGB")
            s = im.width >> dz
            ox, oy = (x & ((1 << dz) - 1)) * s, (y & ((1 << dz) - 1)) * s
            im = im.crop((ox, oy, ox + s, oy + s)).resize((256, 256), Image.BICUBIC)
            out = io.BytesIO()
            im.save(out, "JPEG", quality=85)
            return out.getvalue()
        except Exception as e:
            print(f"[overzoom] {layer}/{z}/{x}/{y}: {e}")
    return None


@app.route("/tile/<layer>/<int:z>/<int:x>/<int:y>")
def tile(layer, z, x, y):
    if layer not in TILES:
        abort(404)
    mime = TILES[layer][1]
    hdr = {"Cache-Control": "max-age=86400"}
    data = read_cached(layer, z, x, y) or download_tile(layer, z, x, y)
    if data:
        return Response(data, mimetype=mime, headers=hdr)
    data = overzoom(layer, z, x, y)                   # not written to disk, so the real tile can still be fetched later
    if data:
        return Response(data, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})
    abort(Response("", 404, {"Cache-Control": "no-store"}))


# ---------------------------------------------------------------- area prefetch (3x3 km)
PF = {"running": False, "cancel": False, "done": 0, "total": 0, "fail": 0, "msg": ""}
PF_LOCK = threading.Lock()


def deg2tile(lat, lon, z):
    lat = max(-85.0511, min(85.0511, lat))
    n = 2 ** z
    return (int((lon + 180) / 360 * n),
            int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n))


def tiles_in_box(lat, lon, half_m, zooms):
    dlat = half_m / 110540
    dlon = half_m / (111320 * math.cos(math.radians(lat)))
    for z in zooms:
        x1, y1 = deg2tile(lat + dlat, lon - dlon, z)
        x2, y2 = deg2tile(lat - dlat, lon + dlon, z)
        for x in range(x1, x2 + 1):
            for y in range(y1, y2 + 1):
                yield z, x, y


def prefetch_run(layer, jobs):
    from concurrent.futures import ThreadPoolExecutor
    streak = [0]

    def one(j):
        if PF["cancel"]:
            return
        z, x, y = j
        if not is_cached(layer, z, x, y):
            ok = download_tile(layer, z, x, y, force=True) is not None
            with PF_LOCK:
                if ok:
                    streak[0] = 0
                else:
                    PF["fail"] += 1
                    streak[0] += 1
                    if streak[0] >= 20:               # 20 consecutive failures: most likely no network
                        PF["cancel"] = True
                        PF["msg"] = "Network error, download stopped"
        with PF_LOCK:
            PF["done"] += 1
    try:
        with ThreadPoolExecutor(6) as ex:
            list(ex.map(one, jobs))
    finally:
        PF["running"] = False


@app.route("/prefetch", methods=["POST"])
def prefetch():
    d = request.get_json(silent=True) or {}
    try:
        lat, lon = float(d["lat"]), float(d["lon"])
    except (KeyError, TypeError, ValueError):
        abort(400)
    layer = d.get("layer", "sat")
    if layer not in TILES or not good((lat, lon)):
        abort(400)
    with PF_LOCK:
        if PF["running"]:
            return jsonify(PF)
        jobs = list(tiles_in_box(lat, lon, PREFETCH_HALF_M, PREFETCH_ZOOMS))
        PF.update(running=True, cancel=False, done=0, total=len(jobs), fail=0, msg="")
    threading.Thread(target=prefetch_run, args=(layer, jobs), daemon=True).start()
    return jsonify(PF)


@app.route("/prefetch/status")
def prefetch_status():
    return jsonify(PF)


@app.route("/prefetch/cancel", methods=["POST"])
def prefetch_cancel():
    PF["cancel"] = True
    return jsonify(PF)


# Needs in ./static : maplibre-gl.js, maplibre-gl.css, three.min.js (r128)
HTML = r"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>GPS Map</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="/static/maplibre-gl.css">
<script src="/static/maplibre-gl.js"></script>
<script src="/static/three.min.js"></script>
<style>
 html,body{margin:0;height:100%;font-family:system-ui,"Segoe UI",sans-serif}
 #map{position:absolute;inset:0;background:#2b2f33}
 .ui{position:absolute;z-index:10}
 #status{top:12px;left:12px;display:flex;align-items:center;gap:10px;background:#fff;border-radius:24px;
  padding:10px 18px;box-shadow:0 2px 8px rgba(0,0,0,.35);font-size:16px;max-width:60vw}
 #dot{width:14px;height:14px;border-radius:50%;background:#9e9e9e;flex:none}
 #hud{left:12px;bottom:28px;background:rgba(255,255,255,.95);border-radius:12px;padding:10px 14px;
  box-shadow:0 2px 8px rgba(0,0,0,.35);font-size:15px;line-height:1.5;cursor:pointer}
 #hud small{color:#666} #warn{color:#d84315;font-weight:600}
 #cmp{right:12px;top:128px;width:176px;background:rgba(255,255,255,.95);border-radius:12px;padding:8px;
  box-shadow:0 2px 8px rgba(0,0,0,.35);font-size:14px;line-height:1.5}
 #cmp svg{display:block;margin:0 auto} #cmp b{font-variant-numeric:tabular-nums}
 #camLbl,#tgtLbl{display:none;pointer-events:none;background:rgba(255,255,255,.92);border:1px solid #555;border-radius:4px;
  padding:0 6px;font-size:14px;white-space:nowrap;transform:translate(-50%,-160%)}
 #vid{left:12px;top:64px;width:320px;max-width:45vw;display:none;background:#000;border-radius:12px;
  box-shadow:0 2px 8px rgba(0,0,0,.35);cursor:pointer}
 #vid.big{width:min(70vw,960px);max-width:none}
 #btns{right:12px;bottom:28px;display:flex;flex-direction:column;gap:10px}
 #btns button{min-width:150px;height:54px;border:0;border-radius:14px;font-size:17px;font-weight:600;
  background:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35);cursor:pointer}
 #btns button.on{background:#1976d2;color:#fff}
 .mk{width:14px;height:14px;border-radius:50%;border:2px solid #fff;box-shadow:0 0 3px rgba(0,0,0,.6)}
 .mk b{position:absolute;bottom:18px;left:50%;transform:translateX(-50%);white-space:nowrap;background:rgba(255,255,255,.92);
  border:1px solid #555;border-radius:4px;padding:0 6px;font-size:14px}
</style></head>
<body>
<div id="map"></div>
<div id="status" class="ui"><span id="dot"></span><span id="stxt">Waiting for device...</span></div>
<div id="camLbl" class="ui"></div>
<div id="tgtLbl" class="ui"></div>
<img id="vid" class="ui" alt="video" title="Click to enlarge / shrink">
<div id="hud" class="ui" title="Tap to copy coordinates" style="display:none"></div>
<div id="cmp" class="ui" style="display:none">
 <svg viewBox="0 0 160 160" width="160" height="160">
  <circle cx="80" cy="80" r="70" fill="#fff" stroke="#555" stroke-width="1.5"/>
  <g id="cmpTicks"></g>
  <path id="cmpWedge" fill="rgba(255,90,54,.25)" stroke="none" d=""/>
  <line id="cmpAxis" x1="80" y1="80" x2="80" y2="80" stroke="#ff5a36" stroke-width="3" stroke-linecap="round"/>
  <polygon id="cmpHdg" points="80,24 91,92 80,82 69,92" fill="#1976d2" stroke="#fff" stroke-width="1"/>
  <circle cx="80" cy="80" r="3.5" fill="#fff" stroke="#333"/>
 </svg>
 <div id="cmpT"></div>
</div>
<div id="btns" class="ui">
 <button id="bCenter">◎ Center</button>
 <button id="bFollow" class="on">Follow: On</button>
 <button id="bLayer">Basemap: Satellite</button>
 <button id="bView">View: 3D</button>
 <button id="bVideo">Video: Off</button>
 <button id="bDl">⬇ Download 3×3 km</button>
</div>
<script>
const $ = id => document.getElementById(id);
const rs = n => ({type:'raster', tiles:[location.origin + '/tile/' + n + '/{z}/{x}/{y}'], tileSize:256, maxzoom:19});
const START = /*START*/null;                     // [lat, lon, zoom] from --start, else null
const map = new maplibregl.Map({
  container:'map', center:START ? [START[1], START[0]] : [106,16], zoom:START ? START[2] : 5, maxZoom:22, maxPitch:85, pitch:55, attributionControl:false,
  style:{version:8, sources:{sat:rs('sat'), osm:rs('osm')}, layers:[
    {id:'bg', type:'background', paint:{'background-color':'#2b2f33'}},
    {id:'sat', type:'raster', source:'sat'},
    {id:'osm', type:'raster', source:'osm', layout:{visibility:'none'}}]}
});
map.addControl(new maplibregl.NavigationControl({visualizePitch:true}), 'top-right');
map.addControl(new maplibregl.ScaleControl({unit:'metric'}), 'bottom-right');

const ll = p => [p[1], p[0]];
const EMPTY = {type:'FeatureCollection', features:[]};
const fmtD = m => m >= 1000 ? (m/1000).toFixed(2) + ' km' : m.toFixed(1) + ' m';
const fmtLL = p => p[0].toFixed(7) + ', ' + p[1].toFixed(7);
const mid = (p, q) => [(p[0]+q[0])/2, (p[1]+q[1])/2];
const toENU = (p, o) => [(p[1]-o[1]) * Math.cos(o[0]*Math.PI/180) * 111320, (p[0]-o[0]) * 110540];
const dist = (a, b) => { const e = toENU(a, b); return Math.hypot(e[0], e[1]); };
const fullFp = m => !!(m && m.corners && m.corners.length === 4 && m.corners.every(Boolean));
const copyTarget = () => last && (last.center || last.drone || last.camera || last.target);
function bearing(a, b) {
  const r = Math.PI/180, dl = (b[1]-a[1])*r;
  const y = Math.sin(dl)*Math.cos(b[0]*r);
  const x = Math.cos(a[0]*r)*Math.sin(b[0]*r) - Math.sin(a[0]*r)*Math.cos(b[0]*r)*Math.cos(dl);
  return (Math.atan2(y, x)/r + 360) % 360;
}
function copyText(t, btn) {
  const done = () => { if (btn) btn.textContent = 'Copied ✓'; };
  if (navigator.clipboard) navigator.clipboard.writeText(t).then(done); else done();
}

// ---- compass: platform heading and sensor relative azimuth ----
// North is up. Blue arrow = platform heading. Orange axis + wedge = camera direction = heading + relative azimuth
// (azimuth clockwise from the platform nose, MISB ST 0601).
const SVGNS = 'http://www.w3.org/2000/svg';
const nrm = a => ((a % 360) + 360) % 360;
const sgn = a => { a = nrm(a); return a > 180 ? a - 360 : a; };
const pol = (cx, cy, r, deg) => [cx + r*Math.sin(deg*Math.PI/180), cy - r*Math.cos(deg*Math.PI/180)];
const sv = (tag, a) => { const e = document.createElementNS(SVGNS, tag); for (const k in a) e.setAttribute(k, a[k]); return e; };
(function initCompass() {
  const g = $('cmpTicks');
  for (let a = 0; a < 360; a += 10) {
    const big = a % 30 === 0, p1 = pol(80, 80, 70, a), p2 = pol(80, 80, big ? 62 : 66, a);
    g.appendChild(sv('line', {x1:p1[0], y1:p1[1], x2:p2[0], y2:p2[1], stroke:'#555', 'stroke-width':big ? 1.5 : 1}));
  }
  [['N',0], ['E',90], ['S',180], ['W',270]].forEach(([t, a]) => {
    const p = pol(80, 80, 52, a), e = sv('text', {x:p[0], y:p[1] + 4, 'text-anchor':'middle', 'font-size':13, 'font-weight':700, fill:t === 'N' ? '#d84315' : '#333'});
    e.textContent = t; g.appendChild(e);
  });
})();
function updateCompass(m) {
  const c = m.compass, box = $('cmp');
  if (!c) { box.style.display = 'none'; return; }
  box.style.display = 'block';
  const h = c.hdg, az = c.raz;
  const cam = h != null && az != null ? nrm(h + az) : null;
  $('cmpHdg').style.display = h != null ? '' : 'none';
  if (h != null) $('cmpHdg').setAttribute('transform', 'rotate(' + h + ' 80 80)');
  if (cam != null) {
    const half = (m.hfov || 60) / 2, pts = [];
    for (let i = 0; i <= 16; i++) pts.push(pol(80, 80, 58, cam - half + 2*half*i/16).map(v => v.toFixed(1)).join(' '));
    $('cmpWedge').setAttribute('d', 'M80 80 L' + pts.join(' L') + ' Z');
    const a = pol(80, 80, 64, cam);
    $('cmpAxis').setAttribute('x2', a[0]); $('cmpAxis').setAttribute('y2', a[1]);
  } else {
    $('cmpWedge').setAttribute('d', '');
    $('cmpAxis').setAttribute('x2', 80); $('cmpAxis').setAttribute('y2', 80);
  }
  const f = (v, s) => v == null ? '--' : (s && v > 0 ? '+' : '') + v.toFixed(1) + '°';
  $('cmpT').innerHTML =
    'Heading <b>' + f(h != null ? nrm(h) : null) + '</b><br>' +
    'Rel. azimuth <b>' + f(az != null ? sgn(az) : null, 1) + '</b>';
}

// ---- HTML markers (dot + permanent label) ----
function mkMarker(color, label) {
  const el = document.createElement('div'); el.className = 'mk'; el.style.background = color;
  if (label) el.innerHTML = '<b>' + label + '</b>'; else el.style.width = el.style.height = '8px';
  return new maplibregl.Marker({element:el});
}
const show = (mk, p) => { mk.setLngLat(ll(p)); if (!mk._on) { mk.addTo(map); mk._on = 1; } };
const hide = mk => { if (mk._on) { mk.remove(); mk._on = 0; } };
const cMk = mkMarker('#ff5a36', 'Center'), dMk = mkMarker('#448aff', 'Aircraft');
const cornerMk = {};
const allMk = () => Object.values(cornerMk).concat([cMk, dMk]);

// ---- three.js custom layer: camera sphere, lines to P3 and P4, target, estimated-area outline (ENU meters) ----
const T = {origin:null, group:null, mats:[], spheres:[]};
const mul = (a, b) => { const o = new Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let v = 0;
    for (let k = 0; k < 4; k++) v += a[k*4+r] * b[c*4+k]; o[c*4+r] = v; } return o; };
const customLayer = {
  id:'t3d', type:'custom', renderingMode:'3d',
  onAdd(m, gl) {
    this.cam = new THREE.Camera(); this.scene = new THREE.Scene();
    T.group = new THREE.Group(); this.scene.add(T.group);
    this.renderer = new THREE.WebGLRenderer({canvas:m.getCanvas(), context:gl, antialias:true});
    this.renderer.autoClear = false;
  },
  render(gl, args) {
    if (!T.origin || !T.group.children.length) return;
    const main = args && args.defaultProjectionData ? args.defaultProjectionData.mainMatrix : args;
    const mc = maplibregl.MercatorCoordinate.fromLngLat(T.origin, 0), s = mc.meterInMercatorCoordinateUnits();
    // ENU (x east, y north, z up, meters) -> mercator; folded in float64 to avoid jitter
    const L = [s,0,0,0, 0,-s,0,0, 0,0,s,0, mc.x,mc.y,mc.z,1];
    const M = mul(Array.from(main), L);
    this.cam.projectionMatrix.fromArray(M);
    const place = (el, p) => {                       // project a 3D point to screen for its label
      const w = p ? M[3]*p[0] + M[7]*p[1] + M[11]*p[2] + M[15] : 0;
      if (w > 0) {
        const x = M[0]*p[0] + M[4]*p[1] + M[8]*p[2] + M[12], y = M[1]*p[0] + M[5]*p[1] + M[9]*p[2] + M[13], cv = map.getCanvas();
        el.style.left = ((x/w*.5 + .5) * cv.clientWidth) + 'px'; el.style.top = ((.5 - y/w*.5) * cv.clientHeight) + 'px';
        el.style.display = 'block';
      } else el.style.display = 'none';
    };
    place($('camLbl'), T.cpos);
    place($('tgtLbl'), T.tpos);
    // spheres keep a constant size on screen (about 7 px radius), whatever the zoom or the distances involved
    const mpp = 78271.517 * Math.cos(T.origin[1] * Math.PI / 180) / Math.pow(2, map.getZoom());
    T.spheres.forEach(([o, k]) => o.scale.setScalar(mpp * 7 * k));
    this.renderer.resetState();
    this.renderer.render(this.scene, this.cam);
  }
};

function build3d(m) {
  const g = T.group; if (!g) return;
  // nothing 3D changed (e.g. a static payload): keep the existing objects untouched, no rebuild and no flicker
  const key = JSON.stringify(m ? [m.camera, m.alt, m.center, m.target, m.talt, m.corners, m.est] : null);
  if (key === T.key) return;
  T.key = key;
  while (g.children.length) { const o = g.children.pop(); o.geometry.dispose(); o.material.dispose(); }
  T.mats = []; T.spheres = []; T.cpos = null; T.tpos = null;
  // origin of the local ENU frame: any valid point, so payload and target never depend on each other
  const org = m && (m.center || m.camera || m.target || (m.corners || []).find(Boolean) || (m.est || [])[0]);
  if (!org) {
    T.origin = null; $('camLbl').style.display = $('tgtLbl').style.display = 'none'; map.triggerRepaint(); return;
  }
  T.origin = ll(org);
  const add = (obj, mat, op) => { mat.userData = {op}; T.mats.push(mat); g.add(obj); };
  const geo = arr => { const b = new THREE.BufferGeometry(); b.setAttribute('position', new THREE.Float32BufferAttribute(arr.flat(), 3)); return b; };
  let mat, C = null;
  if (m.camera) {                                   // payload: drawn whenever its position is valid
    const c = toENU(m.camera, org), h = m.alt != null ? Math.max(0, m.alt) : 0;
    C = [c[0], c[1], h]; T.cpos = C;
    $('camLbl').textContent = 'Payload' + (m.alt != null ? ' · ' + h.toFixed(0) + ' m' : '');
  } else $('camLbl').style.display = 'none';
  if (C && !fullFp(m) && m.est) {                   // estimated visible area (fewer than 4 valid corners): fainter than real data
    const E = m.est.map(p => { const e = toENU(p, org); return [e[0], e[1], 0]; });
    mat = new THREE.LineBasicMaterial({color:0xf0dc82, transparent:true, opacity:.7});   // outline on the ground
    add(new THREE.LineSegments(geo(E.flatMap((p, i) => [p, E[(i+1) % 4]])), mat), mat, .7);
  }
  const fp4 = fullFp(m) ? m.corners : m.est;        // received footprint, or received + estimated corners
  if (C && fp4) {                                   // payload -> P3 and P4 only (received, or estimated when missing)
    const E4 = [fp4[2], fp4[3]].map(p => { const e = toENU(p, org); return [e[0], e[1], 0]; });
    mat = new THREE.LineBasicMaterial({color:0xf0dc82, transparent:true});
    add(new THREE.LineSegments(geo([C, E4[0], C, E4[1]]), mat), mat, 1);
  }
  if (m.target) {                                   // target: sphere at its elevation, line from the payload if there is one
    const e = toENU(m.target, org), Tp = [e[0], e[1], Math.max(0, m.talt || 0)];
    T.tpos = Tp; $('tgtLbl').textContent = 'Target';
    if (C) {
      mat = new THREE.LineBasicMaterial({color:0x00e676, transparent:true});
      add(new THREE.LineSegments(geo([C, Tp]), mat), mat, 1);
    }
    mat = new THREE.MeshBasicMaterial({color:0x00e676, side:THREE.DoubleSide, transparent:true});
    const ts = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), mat); ts.position.set(...Tp);
    T.spheres.push([ts, 0.7]); add(ts, mat, 1);
  }
  if (!m.target) $('tgtLbl').style.display = 'none';
  if (C) {                                          // payload sphere
    mat = new THREE.MeshBasicMaterial({color:0xff5a36, side:THREE.DoubleSide, transparent:true});
    const sp = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), mat); sp.position.set(...C);
    T.spheres.push([sp, 1]); add(sp, mat, 1);
  }
  map.triggerRepaint();
}

// ---- state & helpers ----
let last = null, lastMsgT = 0, lastValidT = 0, follow = true, fitMode = 0, ready = false, lostPrev = false;
let viewMode = 'tilt';                         // tilt | top
const setFollow = v => { follow = v; $('bFollow').textContent = 'Follow: ' + (v ? 'On' : 'Off'); $('bFollow').classList.toggle('on', v); };

function fit() {
  if (!last) return;
  const pts = [...(last.corners || []).filter(Boolean), last.center, last.drone, last.camera, last.target].filter(Boolean);
  if (!pts.length) return;
  if (pts.length === 1) { map.jumpTo({center:ll(pts[0]), zoom:Math.max(map.getZoom(), 19)}); return; }
  const b = new maplibregl.LngLatBounds();
  pts.forEach(p => b.extend(ll(p)));
  map.fitBounds(b, {padding:100, maxZoom:fullFp(last) ? 20 : 21, animate:false});
}
$('bCenter').onclick = () => { setFollow(true); fit(); };
$('bFollow').onclick = () => { setFollow(!follow); const p = copyTarget(); if (follow && p) map.jumpTo({center:ll(p)}); };
let cur = 'sat';
$('bLayer').onclick = () => {
  map.setLayoutProperty(cur, 'visibility', 'none');
  cur = cur === 'sat' ? 'osm' : 'sat';
  map.setLayoutProperty(cur, 'visibility', 'visible');
  $('bLayer').textContent = 'Basemap: ' + (cur === 'sat' ? 'Satellite' : 'Street');
};
$('bView').onclick = () => {
  viewMode = viewMode === 'tilt' ? 'top' : 'tilt';
  $('bView').textContent = 'View: ' + {tilt:'3D', top:'Top'}[viewMode];
  if (viewMode === 'tilt') map.easeTo({pitch:55});
  else map.easeTo({pitch:0, bearing:0});
};
const BLANK = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';
$('bVideo').onclick = () => {
  const on = $('vid').style.display !== 'block', v = $('vid');
  $('bVideo').textContent = 'Video: ' + (on ? 'On' : 'Off');
  $('bVideo').classList.toggle('on', on);
  if (on) { v.src = '/video?t=' + Date.now(); v.style.display = 'block'; }
  else { v.src = BLANK; v.style.display = 'none'; }          // dropping the src closes the MJPEG connection
};
$('vid').onclick = () => $('vid').classList.toggle('big');

// ---- offline area download (3x3 km around the map center) ----
const DL_LABEL = '⬇ Download 3×3 km';
async function pollDl() {
  const b = $('bDl');
  let s;
  try { s = await (await fetch('/prefetch/status')).json(); } catch (e) { return; }
  if (s.running) {
    b.dataset.run = '1'; b.classList.add('on');
    b.textContent = 'Cancel · ' + Math.round(100 * s.done / Math.max(1, s.total)) + '%';
    setTimeout(pollDl, 1000);
    return;
  }
  b.dataset.run = '0'; b.classList.remove('on');
  b.textContent = s.msg ? '⚠ ' + s.msg : 'Saved ✓' + (s.fail ? ' (' + s.fail + ' failed)' : '');
  setTimeout(() => { if (b.dataset.run !== '1') b.textContent = DL_LABEL; }, 5000);
}
$('bDl').onclick = async () => {
  const b = $('bDl');
  if (b.dataset.run === '1') { await fetch('/prefetch/cancel', {method:'POST'}); return; }
  const c = map.getCenter();
  const r = await fetch('/prefetch', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({lat:c.lat, lon:c.lng, layer:cur})});
  if (r.ok) pollDl();
};

$('hud').onclick = () => { const p = copyTarget(); if (p) copyText(fmtLL(p)); };
map.on('dragstart', () => setFollow(false));

map.once('style.load', () => {
  ['fp', 'fp-est'].forEach(id => map.addSource(id, {type:'geojson', data:EMPTY}));
  // estimated visible area (only when fewer than 4 corners are valid): dashed outline, light fill
  map.addLayer({id:'fp-est-fill', type:'fill', source:'fp-est', paint:{'fill-color':'#f0dc82', 'fill-opacity':.12}});
  map.addLayer({id:'fp-est-line', type:'line', source:'fp-est', paint:{'line-color':'#f0dc82', 'line-width':2, 'line-dasharray':[2,2]}});
  map.addLayer({id:'fp-fill', type:'fill', source:'fp', paint:{'fill-color':'#f0dc82', 'fill-opacity':.3}});
  map.addLayer({id:'fp-line', type:'line', source:'fp', paint:{'line-color':'#f0dc82', 'line-width':3}});
  ready = true;
  const es = new EventSource('/stream');
  es.onmessage = e => {
    const m = JSON.parse(e.data), now = performance.now();
    lastMsgT = now;
    if (m.valid) { lastValidT = now; draw(m); }
  };
  try {
    if (typeof THREE === 'undefined') throw new Error('static/three.min.js not loaded');
    map.addLayer(customLayer);
  } catch (err) { console.error('3D layer disabled:', err); }
});

function drawCam(m) {
  const t = m.target ? '<br><small>Target: ' + fmtLL(m.target) +
    (m.talt != null ? ' · h ' + m.talt.toFixed(1) + ' m' : '') + '</small>' : '';
  build3d(m);
  return (m.camera && !m.center ? '<br><small>Payload: ' + fmtLL(m.camera) + '</small>' : '') + t;
}

let pend = 0;
function draw(m) {
  last = m;
  if (pend) return;
  pend = requestAnimationFrame(() => { pend = 0; drawNow(last); });
}
function drawNow(m) {
  last = m;
  if (!ready) return;
  updateCompass(m);
  const cs = m.corners || [], full = fullFp(m);
  // 4 valid corners: the received footprint is drawn as is. Otherwise the estimated area (if any) is drawn.
  map.getSource('fp').setData(full ? {type:'Feature', geometry:{type:'Polygon',
      coordinates:[[...cs.map(ll), ll(cs[0])]]}} : EMPTY);
  map.getSource('fp-est').setData(!full && m.est ? {type:'Feature', geometry:{type:'Polygon',
      coordinates:[[...m.est.map(ll), ll(m.est[0])]]}} : EMPTY);
  cs.forEach((c, i) => {                            // each valid corner is drawn on its own
    const k = m.labels[i];
    if (c) { if (!cornerMk[k]) cornerMk[k] = mkMarker('#f0dc82', k); show(cornerMk[k], c); }
    else if (cornerMk[k]) hide(cornerMk[k]);
  });
  if (m.center) show(cMk, m.center); else hide(cMk);
  if (m.drone)  show(dMk, m.drone);  else hide(dMk);

  const html0 = drawCam(m);
  if (fitMode === 0) { fit(); fitMode = full ? 2 : 1; }
  else if (fitMode === 1 && full) { fit(); fitMode = 2; }
  else if (follow && copyTarget()) map.jumpTo({center:ll(copyTarget())});

  let html = '';
  if (m.center) {
    html = '<b>' + fmtLL(m.center) + '</b>';
    if (full) {
      const far = dist(cs[0], cs[1]), near = dist(cs[3], cs[2]);
      const depth = (dist(cs[0], cs[3]) + dist(cs[1], cs[2])) / 2;
      html += '<br><small>Footprint ' + fmtD((far + near) / 2) + ' × ' + fmtD(depth) + '</small>' +
        (m.oversize ?
        '<br><span id="warn">Payload angle too low, observed area is inaccurate.</span>' : '');
    }
  } else if (m.est) {
    html = '<b>Visible area (estimated)</b>' + (m.oversize ? '<br><small>Received area too large, showing estimate</small>' :
      m.ncorners ? '<br><small>' + m.ncorners + '/4 corners valid</small>' : '');
  } else if (m.ncorners >= 2) {
    html = '<b>Partial footprint</b><br><small>' + m.ncorners + '/4 corners valid</small>';
  } else if (m.drone) html = '<b>' + fmtLL(m.drone) + '</b>';
  if (!m.center && m.center_lost && !m.est && m.ncorners < 2)
    html += (html ? '<br>' : '') + '<span id="warn">Frame center unavailable: payload does not hit the ground. Tilt the payload down.</span>';
  if (m.range != null)
    html += (html ? '<br>' : '') + 'Slant range: <b>' + m.range.toFixed(3) + ' m</b>';
  $('hud').style.display = 'block';
  $('hud').innerHTML = html + html0;
}

function setStatus(color, text) { $('dot').style.background = color; $('stxt').textContent = text; }
setInterval(() => {
  const now = performance.now();
  const lost = !!(last && now - lastMsgT > 3000);
  if (ready && lost !== lostPrev) {
    lostPrev = lost;
    const col = lost ? '#9e9e9e' : '#f0dc82';
    map.setPaintProperty('fp-fill', 'fill-color', col);
    map.setPaintProperty('fp-line', 'line-color', col);
    map.setPaintProperty('fp-est-fill', 'fill-color', col);
    map.setPaintProperty('fp-est-line', 'line-color', col);
    $('camLbl').style.opacity = $('tgtLbl').style.opacity = $('cmp').style.opacity = lost ? .4 : 1;
    allMk().forEach(mk => mk.getElement().style.opacity = lost ? .4 : 1);
    T.mats.forEach(mt => mt.opacity = mt.userData.op * (lost ? .35 : 1));
    map.triggerRepaint();
  }
  if (!lastMsgT && !last)            setStatus('#fb8c00', 'Waiting for device...');
  else if (lost)                     setStatus('#e53935', 'Signal lost. Showing last known position.');
  else if (now - lastValidT > 3000)  setStatus('#fb8c00', 'No GPS fix. Move the device to an open area and wait.');
  else                               setStatus('#43a047', 'Receiving data');
}, 500);
</script></body></html>
"""


@app.route("/")
def index():
    html = HTML.replace("/*START*/null", json.dumps(START)) if START else HTML
    return Response(html, mimetype="text/html")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--video-port", type=int, default=5006, help="TCP port on which the C++ app delivers JPEG preview frames")
    ap.add_argument("--cam", help="fallback camera position when the stream has none: lat,lon[,alt]")
    ap.add_argument("--ground-alt", type=float, default=0.0,
                    help="ground elevation (m, MSL) under the camera; subtracted from altitude to get height above ground")
    ap.add_argument("--start", help="initial map position until GPS data arrives: lat,lon[,zoom] (default zoom 17)")
    ap.add_argument("--max-range", type=float, default=10000.0,
                    help="frame center farther than this many meters from the camera is not drawn (default 10000, 0 = off)")
    ap.add_argument("--est-range", type=float, default=1000.0,
                    help="when fewer than 4 corners are valid, the estimated visible area extends at most this many meters from the camera (default 1000)")
    ap.add_argument("--big-area", type=float, default=2000.0,
                    help="meters: a received 4-corner footprint whose side or depth exceeds this is replaced by the estimated area (default 2000, 0 = off)")
    ap.add_argument("--est-hold", type=float, default=10.0,
                    help="seconds the estimated corners are kept before being recomputed from the frame center (default 10)")
    ap.add_argument("--est-angle", type=float, default=3.0,
                    help="degrees: a larger change of any gimbal / platform angle refreshes the estimate immediately (default 3)")
    ap.add_argument("--est-move", type=float, default=10.0,
                    help="meters: the camera moving farther than this refreshes the estimate immediately (default 10)")
    args = ap.parse_args()
    EST_HOLD[0], EST_ANGLE[0], EST_MOVE[0] = args.est_hold, args.est_angle, args.est_move
    BIG_AREA[0] = args.big_area
    GROUND_ALT[0] = args.ground_alt
    RANGE_MAX[0] = args.max_range
    EST_RANGE[0] = args.est_range

    if args.start:
        try:
            vals = [float(x) for x in args.start.replace(";", ",").split(",")]
            if len(vals) not in (2, 3) or not good(vals[:2]):
                raise ValueError
            zoom = vals[2] if len(vals) == 3 else 17.0
            START[:] = [vals[0], vals[1], max(0.0, min(22.0, zoom))]
        except ValueError:
            ap.error("--start must be lat,lon or lat,lon,zoom")

    if args.cam:
        try:
            vals = [float(x) for x in args.cam.split(",")]
            if len(vals) not in (2, 3):
                raise ValueError
            CAM_DEFAULT[:] = vals
        except ValueError:
            ap.error("--cam must be lat,lon or lat,lon,alt")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", args.udp_port))
    except OSError:
        print(f"Cannot open data port {args.udp_port}. The app may already be running in another window.")
        if sys.stdin and sys.stdin.isatty():
            input("Press Enter to exit...")
        return
    threading.Thread(target=udp_loop, args=(sock,), daemon=True).start()

    try:
        vsrv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        vsrv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        vsrv.bind(("127.0.0.1", args.video_port))
        vsrv.listen(2)
        threading.Thread(target=video_server, args=(vsrv,), daemon=True).start()
    except OSError:
        print(f"Cannot open video port {args.video_port}; video preview disabled.")

    url = f"http://127.0.0.1:{args.port}"
    print(f"GPS Map running at {url} (UDP data on port {args.udp_port})")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=args.port, threaded=True)


if __name__ == "__main__":
    main()