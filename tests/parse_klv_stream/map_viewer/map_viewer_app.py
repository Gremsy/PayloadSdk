import argparse, json, logging, math, os, socket, sys, threading, urllib.request, webbrowser
from flask import Flask, Response, abort

BASE = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(os.path.expanduser("~"), "GPSMapCache")
KLV_CORNER_ORDER = (2, 4, 5, 3)   # packet ids of the image corners TL, TR, BR, BL
MAX_CORNER_OFFSET = 0.0745        # KLV corner offsets saturate at 0.075 deg when a point is out of range
TILES = {
    "sat": ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", "image/jpeg"),
    "osm": ("https://tile.openstreetmap.org/{z}/{x}/{y}.png", "image/png"),
}

app = Flask(__name__, static_folder=os.path.join(BASE, "static"), static_url_path="/static")
logging.getLogger("werkzeug").setLevel(logging.ERROR)
cond = threading.Condition()
state = {"seq": 0, "msg": "{}"}


def good(p):
    return (all(math.isfinite(v) for v in p) and abs(p[0]) < 90 and abs(p[1]) < 180
            and (abs(p[0]) > 1e-9 or abs(p[1]) > 1e-9))


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
    if all(i in pts for i in range(1, 7)):
        drone, center, ids, base = pts[1], pts[2], (3, 4, 5, 6), 2
    elif all(i in pts for i in range(1, 6)):
        drone, center, ids, base = None, pts[1], KLV_CORNER_ORDER, 1
    elif set(pts) == {1}:
        drone, center, ids, base = None, pts[1], (), 1
    else:
        return None
    if not good(center) or (drone and not good(drone)):
        return {"valid": False}
    corners = [pts[i] for i in ids]
    drawable = bool(corners) and all(
        good(p) and abs(p[0] - center[0]) < MAX_CORNER_OFFSET and abs(p[1] - center[1]) < MAX_CORNER_OFFSET
        for p in corners)
    return {"valid": True, "drone": drone, "center": center,
            "corners": corners if drawable else None,
            "labels": [f"P{i - base}" for i in ids] if drawable else None}


def udp_loop(sock):
    while True:
        raw, _ = sock.recvfrom(4096)
        msg = build(parse(raw.decode(errors="ignore").strip()))
        if msg:
            with cond:
                state["seq"] += 1
                state["msg"] = json.dumps(msg)
                cond.notify_all()


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


@app.route("/tile/<layer>/<int:z>/<int:x>/<int:y>")
def tile(layer, z, x, y):
    if layer not in TILES:
        abort(404)
    url, mime = TILES[layer]
    path = os.path.join(CACHE, layer, str(z), str(x), f"{y}.img")
    data = None
    if os.path.exists(path) and os.path.getsize(path) > 100:
        with open(path, "rb") as f:
            data = f.read()
    if data is None:
        err = None
        for _ in range(2):
            try:
                req = urllib.request.Request(url.format(z=z, x=x, y=y), headers={"User-Agent": "GPSMapApp/1.0"})
                with urllib.request.urlopen(req, timeout=8) as r:
                    body = r.read()
                    if r.status == 200 and r.headers.get("Content-Type", "").startswith("image/") and len(body) > 100:
                        data = body
                        break
                    err = f"response is not an image ({r.status}, {len(body)} bytes)"
            except Exception as e:
                err = e
        if data is None:
            print(f"[tile] {layer}/{z}/{x}/{y} failed: {err}")
            abort(Response("", 404, {"Cache-Control": "no-store"}))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = f"{path}.{threading.get_ident()}.tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)
    return Response(data, mimetype=mime, headers={"Cache-Control": "max-age=86400"})


HTML = r"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>GPS Map</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="/static/leaflet.css">
<script src="/static/leaflet.js"></script>
<style>
 html,body{margin:0;height:100%;font-family:system-ui,"Segoe UI",sans-serif}
 #map{position:absolute;inset:0;background:#2b2f33}
 .ui{position:absolute;z-index:1000}
 #status{top:12px;left:56px;display:flex;align-items:center;gap:10px;background:#fff;border-radius:24px;
  padding:10px 18px;box-shadow:0 2px 8px rgba(0,0,0,.35);font-size:16px;max-width:60vw}
 #dot{width:14px;height:14px;border-radius:50%;background:#9e9e9e;flex:none}
 #hud{left:12px;bottom:28px;background:rgba(255,255,255,.95);border-radius:12px;padding:10px 14px;
  box-shadow:0 2px 8px rgba(0,0,0,.35);font-size:15px;line-height:1.5;cursor:pointer}
 #hud small{color:#666} #warn{color:#d84315;font-weight:600}
 #btns{right:12px;bottom:28px;display:flex;flex-direction:column;gap:10px}
 #btns button{min-width:150px;height:54px;border:0;border-radius:14px;font-size:17px;font-weight:600;
  background:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35);cursor:pointer}
 #btns button.on{background:#1976d2;color:#fff}
 .leaflet-popup-content button{margin-top:6px;padding:6px 12px;font-size:14px}
 .lbl{background:rgba(255,255,255,.92);border:1px solid #555;border-radius:4px;padding:0 6px;font-size:14px;font-weight:700}
</style></head>
<body>
<div id="map"></div>
<div id="status" class="ui"><span id="dot"></span><span id="stxt">Waiting for device...</span></div>
<div id="hud" class="ui" title="Tap to copy coordinates" style="display:none"></div>
<div id="btns" class="ui">
 <button id="bCenter">◎ Center</button>
 <button id="bFollow" class="on">Follow: On</button>
 <button id="bLayer">Basemap: Satellite</button>
</div>
<script>
const $ = id => document.getElementById(id);
const map = L.map('map', {maxZoom:22, zoomSnap:.25, zoomDelta:.5, zoomAnimation:false, attributionControl:false})
             .setView([16, 106], 5);
const layers = {
  sat: L.tileLayer('/tile/sat/{z}/{x}/{y}', {maxZoom:22, maxNativeZoom:19}),
  osm: L.tileLayer('/tile/osm/{z}/{x}/{y}', {maxZoom:22, maxNativeZoom:19})
};
let cur = 'sat'; layers.sat.addTo(map);
Object.values(layers).forEach(l => l.on('tileerror', e => {
  const t = e.tile; t._n = (t._n || 0) + 1;
  if (t._n <= 3) setTimeout(() => { t.src = t.src.split('?')[0] + '?r=' + t._n; }, 1500 * t._n);
}));
L.control.scale({metric:true, imperial:false}).addTo(map);

const poly  = L.polygon([], {color:'#ff1744', weight:3, fillColor:'#ff1744', fillOpacity:.12}).addTo(map);
const trail = L.polyline([], {color:'#29b6f6', weight:3, opacity:.9, dashArray:'8 8'}).addTo(map);
const cm = L.circleMarker([0,0], {radius:6, color:'#fff', weight:2, fillColor:'#ff1744', fillOpacity:1});
const dm = L.circleMarker([0,0], {radius:9, color:'#fff', weight:3, fillColor:'#1976d2', fillOpacity:1});

const PCOL = ['#1e88e5', '#43a047', '#fb8c00', '#8e24aa'];
const cornerMk = {};
function decorate(mk, name, dir) {
  mk.bindTooltip(name, {permanent:true, direction:dir, offset: dir === 'bottom' ? [0, 8] : [0, -8], className:'lbl'});
  mk.bindPopup(layer => {
    const ll = layer.getLatLng(), t = ll.lat.toFixed(7) + ', ' + ll.lng.toFixed(7);
    return '<b>' + name + '</b><br>' + t + '<br><button onclick="copyText(\'' + t + '\', this)">Copy</button>';
  });
}
decorate(cm, 'Center', 'bottom');
decorate(dm, 'Aircraft', 'bottom');

let last = null, lastMsgT = 0, lastValidT = 0, follow = true, fitMode = 0;
const fmtD = m => m >= 1000 ? (m/1000).toFixed(2) + ' km' : m.toFixed(1) + ' m';
const fmtLL = p => p[0].toFixed(7) + ', ' + p[1].toFixed(7);
const copyTarget = () => last && (last.corners ? last.center : last.drone);

function copyText(t, btn) {
  const done = () => { if (btn) btn.textContent = 'Copied ✓'; };
  if (navigator.clipboard) navigator.clipboard.writeText(t).then(done); else done();
}
window.copyText = copyText;

function setFollow(v) {
  follow = v;
  $('bFollow').textContent = 'Follow: ' + (v ? 'On' : 'Off');
  $('bFollow').classList.toggle('on', v);
}
function fit() {
  if (!last) return;
  if (last.corners) map.fitBounds(L.polygon(last.corners).getBounds(), {padding:[100, 100], maxZoom:21});
  else map.setView(last.drone || last.center, Math.max(map.getZoom(), 19));
}
$('bCenter').onclick = () => { setFollow(true); fit(); };
$('bFollow').onclick = () => {
  setFollow(!follow);
  const p = copyTarget();
  if (follow && p) map.panTo(p, {animate:false});
};
$('bLayer').onclick = () => {
  map.removeLayer(layers[cur]);
  cur = cur === 'sat' ? 'osm' : 'sat';
  layers[cur].addTo(map);
  $('bLayer').textContent = 'Basemap: ' + (cur === 'sat' ? 'Satellite' : 'Street');
};
$('hud').onclick = () => { const p = copyTarget(); if (p) copyText(fmtLL(p)); };
map.on('dragstart', () => setFollow(false));
map.on('click', e => {
  const t = e.latlng.lat.toFixed(7) + ', ' + e.latlng.lng.toFixed(7);
  const p = copyTarget();
  const d = p ? 'Distance to ' + (last.corners ? 'center' : 'aircraft') + ': <b>' + fmtD(map.distance(e.latlng, p)) + '</b><br>' : '';
  L.popup().setLatLng(e.latlng)
    .setContent('<b>' + t + '</b><br>' + d + '<button onclick="copyText(\'' + t + '\', this)">Copy</button>')
    .openOn(map);
});

function clearFootprint() {
  poly.setLatLngs([]);
  Object.keys(cornerMk).forEach(k => { map.removeLayer(cornerMk[k]); delete cornerMk[k]; });
  if (map.hasLayer(cm)) map.removeLayer(cm);
}

function draw(m) {
  last = m;
  if (m.corners) {
    poly.setLatLngs(m.corners);
    m.corners.forEach((c, i) => {
      const key = m.labels[i];
      if (cornerMk[key]) { cornerMk[key].setLatLng(c); return; }
      cornerMk[key] = L.circleMarker(c, {radius:7, color:'#fff', weight:2,
                                         fillColor: PCOL[(+key.slice(1) - 1) % 4], fillOpacity:1}).addTo(map);
      decorate(cornerMk[key], key, 'top');
    });
    if (!map.hasLayer(cm)) cm.addTo(map);
    cm.setLatLng(m.center);
    const tl = trail.getLatLngs(), q = tl[tl.length - 1];
    if (!q || q.lat !== m.center[0] || q.lng !== m.center[1]) {
      trail.addLatLng(m.center);
      if (tl.length > 500) trail.setLatLngs(trail.getLatLngs().slice(-500));
    }
  } else {
    clearFootprint();
  }
  if (m.drone) { if (!map.hasLayer(dm)) dm.addTo(map); dm.setLatLng(m.drone); }

  if (fitMode === 0) { fit(); fitMode = m.corners ? 2 : 1; }
  else if (fitMode === 1 && m.corners) { fit(); fitMode = 2; }
  else if (follow && copyTarget()) map.panTo(copyTarget(), {animate:false});

  let html;
  if (m.corners) {
    const w = map.distance(m.corners[0], m.corners[1]), h = map.distance(m.corners[0], m.corners[3]);
    html = '<b>' + fmtLL(m.center) + '</b><br><small>Observed area ≈ ' + fmtD(w) + ' × ' + fmtD(h) +
      ' · tap to copy</small>' + (Math.max(w, h) > 2000 ?
      '<br><span id="warn">Camera angle too low, observed area is inaccurate.</span>' : '');
  } else {
    html = (m.drone ? '<b>' + fmtLL(m.drone) + '</b><br>' : '') +
      '<span id="warn">Values not suitable for drawing the footprint.</span>';
  }
  $('hud').style.display = 'block';
  $('hud').innerHTML = html;
}

const es = new EventSource('/stream');
es.onmessage = e => {
  const m = JSON.parse(e.data), now = performance.now();
  lastMsgT = now;
  if (m.valid) { lastValidT = now; draw(m); }
};

function setStatus(color, text) { $('dot').style.background = color; $('stxt').textContent = text; }
setInterval(() => {
  const now = performance.now();
  const lost = last && now - lastMsgT > 3000;
  poly.setStyle(lost ? {color:'#9e9e9e', fillColor:'#9e9e9e', dashArray:'8 6'}
                     : {color:'#ff1744', fillColor:'#ff1744', dashArray:null});
  Object.values(cornerMk).concat([cm, dm]).forEach(k => k.setStyle({fillOpacity: lost ? .4 : 1}));
  if (!lastMsgT && !last)            setStatus('#fb8c00', 'Waiting for device...');
  else if (lost)                     setStatus('#e53935', 'Signal lost. Showing last known position.');
  else if (now - lastValidT > 3000)  setStatus('#fb8c00', 'No GPS fix. Move the device to an open area and wait.');
  else if (last && !last.corners)    setStatus('#fb8c00', 'Footprint not drawn: values not suitable.');
  else                               setStatus('#43a047', 'Receiving data');
}, 500);
</script></body></html>
"""


@app.route("/")
def index():
    return Response(HTML, mimetype="text/html")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", args.udp_port))
    except OSError:
        print(f"Cannot open data port {args.udp_port}. The app may already be running in another window.")
        if sys.stdin and sys.stdin.isatty():
            input("Press Enter to exit...")
        return
    threading.Thread(target=udp_loop, args=(sock,), daemon=True).start()

    url = f"http://127.0.0.1:{args.port}"
    print(f"GPS Map running at {url} (UDP data on port {args.udp_port})")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=args.port, threaded=True)


if __name__ == "__main__":
    main()