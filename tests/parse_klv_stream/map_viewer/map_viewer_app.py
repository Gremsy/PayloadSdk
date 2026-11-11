import argparse, json, logging, math, os, socket, sys, threading, urllib.request, webbrowser
from flask import Flask, Response, abort

BASE = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(os.path.expanduser("~"), "GPSMapCache")
KLV_CORNER_ORDER = (2, 3, 4, 5)   # packet ids of KLV Point 1..4 = image corners TL, TR, BR, BL
MAX_CORNER_OFFSET = 0.0745        # KLV corner offsets saturate at 0.075 deg when a point is out of range
CAM_DEFAULT = []                  # [lat, lon, alt] from --cam, used when the stream has no camera position
GROUND_ALT = [0.0]                # ground elevation (MSL) under the camera, from --ground-alt
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
    # Optional extras: 10 = camera lat,lon | 11 = altitude (MSL) | 12 = HFOV,VFOV
    #                  13 = target lat,lon | 14 = target elevation (MSL)
    pts = dict(pts)
    cam = pts.pop(10, None)
    alt = pts.pop(11, (None,))[0]
    fov = pts.pop(12, None)
    tgt = pts.pop(13, None)
    talt = pts.pop(14, (None,))[0]
    if cam is None and CAM_DEFAULT:
        cam = tuple(CAM_DEFAULT[:2])
        if alt is None and len(CAM_DEFAULT) > 2:
            alt = CAM_DEFAULT[2]
    cam = cam if cam and good(cam) else None
    if alt is not None and not math.isfinite(alt):
        alt = None
    if alt is not None:
        alt -= GROUND_ALT[0]                 # MSL -> height above ground (AGL)
    tgt = tgt if tgt and good(tgt) else None
    if talt is not None:
        talt = talt - GROUND_ALT[0] if math.isfinite(talt) else None
    if not pts and (cam or tgt):             # no footprint yet: camera and/or target only
        pts = {1: cam or tgt}

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
            "camera": cam, "alt": alt, "hfov": fov[0] if fov else None,
            "target": tgt, "talt": talt,
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
 #camLbl{display:none;pointer-events:none;background:rgba(255,255,255,.92);border:1px solid #555;border-radius:4px;
  padding:0 6px;font-size:14px;white-space:nowrap;transform:translate(-50%,-160%)}
 #btns{right:12px;bottom:28px;display:flex;flex-direction:column;gap:10px}
 #btns button{min-width:150px;height:54px;border:0;border-radius:14px;font-size:17px;font-weight:600;
  background:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35);cursor:pointer}
 #btns button.on{background:#1976d2;color:#fff}
 .maplibregl-popup-content button{margin-top:6px;padding:6px 12px;font-size:14px}
 .mk{width:14px;height:14px;border-radius:50%;border:2px solid #fff;box-shadow:0 0 3px rgba(0,0,0,.6)}
 .mk b{position:absolute;bottom:18px;left:50%;transform:translateX(-50%);white-space:nowrap;background:rgba(255,255,255,.92);
  border:1px solid #555;border-radius:4px;padding:0 6px;font-size:14px}
</style></head>
<body>
<div id="map"></div>
<div id="status" class="ui"><span id="dot"></span><span id="stxt">Waiting for device...</span></div>
<div id="camLbl" class="ui"></div>
<div id="hud" class="ui" title="Tap to copy coordinates" style="display:none"></div>
<div id="btns" class="ui">
 <button id="bCenter">◎ Center</button>
 <button id="bFollow" class="on">Follow: On</button>
 <button id="bLayer">Basemap: Satellite</button>
 <button id="bView">View: 3D</button>
</div>
<script>
const $ = id => document.getElementById(id);
const rs = n => ({type:'raster', tiles:[location.origin + '/tile/' + n + '/{z}/{x}/{y}'], tileSize:256, maxzoom:19});
const map = new maplibregl.Map({
  container:'map', center:[106,16], zoom:5, maxZoom:22, maxPitch:85, pitch:55, attributionControl:false,
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
const copyTarget = () => last && (last.corners ? last.center : (last.drone || last.camera));
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
window.copyText = copyText;

// ---- HTML markers (dot + permanent label) ----
function mkMarker(color, label) {
  const el = document.createElement('div'); el.className = 'mk'; el.style.background = color;
  if (label) el.innerHTML = '<b>' + label + '</b>'; else el.style.width = el.style.height = '8px';
  return new maplibregl.Marker({element:el});
}
const show = (mk, p) => { mk.setLngLat(ll(p)); if (!mk._on) { mk.addTo(map); mk._on = 1; } };
const hide = mk => { if (mk._on) { mk.remove(); mk._on = 0; } };
const cMk = mkMarker('#ff5a36', 'Center'), dMk = mkMarker('#448aff', 'Aircraft'), tMk = mkMarker('#00e676', 'Target');
const cornerMk = {};
const allMk = () => Object.values(cornerMk).concat([cMk, dMk, tMk]);

// ---- three.js custom layer: view pyramid, rays, line of sight, camera sphere (ENU meters) ----
const T = {origin:null, group:null, mats:[]};
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
    const M = mul(Array.from(main), L), c = T.cpos, el = $('camLbl');
    this.cam.projectionMatrix.fromArray(M);
    const w = M[3]*c[0] + M[7]*c[1] + M[11]*c[2] + M[15];           // project the sphere to screen for its label
    if (w > 0) {
      const x = M[0]*c[0] + M[4]*c[1] + M[8]*c[2] + M[12], y = M[1]*c[0] + M[5]*c[1] + M[9]*c[2] + M[13], cv = map.getCanvas();
      el.style.left = ((x/w*.5 + .5) * cv.clientWidth) + 'px'; el.style.top = ((.5 - y/w*.5) * cv.clientHeight) + 'px';
      el.style.display = 'block';
    } else el.style.display = 'none';
    this.renderer.resetState();
    this.renderer.render(this.scene, this.cam);
  }
};

function build3d(m) {
  const g = T.group; if (!g) return;
  while (g.children.length) { const o = g.children.pop(); o.geometry.dispose(); o.material.dispose(); }
  T.mats = [];
  // origin of the local ENU frame: frame center when the footprint exists, else the target
  const org = m && m.camera && m.alt > 0 ? (m.corners ? m.center : m.target) : null;
  if (!org) { T.origin = null; $('camLbl').style.display = 'none'; map.triggerRepaint(); return; }
  T.origin = ll(org);
  const c = toENU(m.camera, org), C = [c[0], c[1], m.alt];
  T.cpos = C; $('camLbl').textContent = 'Payload · ' + m.alt.toFixed(0) + ' m';
  const add = (obj, mat, op) => { mat.userData = {op}; T.mats.push(mat); g.add(obj); };
  const geo = arr => { const b = new THREE.BufferGeometry(); b.setAttribute('position', new THREE.Float32BufferAttribute(arr.flat(), 3)); return b; };
  const r = Math.max(1, Math.hypot(...C) * 0.012);
  let mat;
  if (m.corners) {
    const G = m.corners.map(p => { const e = toENU(p, org); return [e[0], e[1], 0]; });
    // translucent pyramid
    const tri = []; for (let i = 0; i < 4; i++) tri.push(C, G[i], G[(i+1) % 4]);
    mat = new THREE.MeshBasicMaterial({color:0xf0dc82, transparent:true, opacity:.18, side:THREE.DoubleSide, depthWrite:false});
    add(new THREE.Mesh(geo(tri), mat), mat, .18);
    // rays
    mat = new THREE.LineBasicMaterial({color:0xf0dc82, transparent:true});
    add(new THREE.LineSegments(geo(G.flatMap(p => [C, p])), mat), mat, 1);
    // line of sight to the frame center
    mat = new THREE.LineBasicMaterial({color:0xff5a36, transparent:true});
    add(new THREE.LineSegments(geo([C, [0,0,0]]), mat), mat, 1);
  }
  // drop line
  mat = new THREE.LineBasicMaterial({color:0xffffff, transparent:true, opacity:.6});
  add(new THREE.LineSegments(geo([C, [C[0], C[1], 0]]), mat), mat, .6);
  // target: line from the payload + sphere at the target elevation
  if (m.target) {
    const e = toENU(m.target, org), Tp = [e[0], e[1], Math.max(0, m.talt || 0)];
    mat = new THREE.LineBasicMaterial({color:0x00e676, transparent:true});
    add(new THREE.LineSegments(geo([C, Tp]), mat), mat, 1);
    mat = new THREE.MeshBasicMaterial({color:0x00e676, side:THREE.DoubleSide, transparent:true});
    const ts = new THREE.Mesh(new THREE.SphereGeometry(r * 0.7, 16, 12), mat); ts.position.set(...Tp);
    add(ts, mat, 1);
  }
  // camera sphere
  mat = new THREE.MeshBasicMaterial({color:0xff5a36, side:THREE.DoubleSide, transparent:true});
  const sp = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), mat); sp.position.set(...C);
  add(sp, mat, 1);
  map.triggerRepaint();
}

// ---- state & helpers ----
let last = null, lastMsgT = 0, lastValidT = 0, follow = true, fitMode = 0, ready = false, lostPrev = false;
let viewMode = 'tilt';                         // tilt | top
const setFollow = v => { follow = v; $('bFollow').textContent = 'Follow: ' + (v ? 'On' : 'Off'); $('bFollow').classList.toggle('on', v); };

function fit() {
  if (!last) return;
  if (last.corners || last.target) {
    const b = new maplibregl.LngLatBounds();
    (last.corners || []).forEach(p => b.extend(ll(p)));
    if (last.camera) b.extend(ll(last.camera));
    if (last.target) b.extend(ll(last.target));
    map.fitBounds(b, {padding:100, maxZoom:last.corners ? 20 : 21, animate:false});
  } else map.jumpTo({center:ll(last.drone || last.camera || last.center), zoom:Math.max(map.getZoom(), 19)});
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
$('hud').onclick = () => { const p = copyTarget(); if (p) copyText(fmtLL(p)); };
map.on('dragstart', () => setFollow(false));
map.on('click', e => {
  const p = [e.lngLat.lat, e.lngLat.lng], t = fmtLL(p), tp = copyTarget();
  const what = last && (last.corners ? 'center' : last.drone ? 'aircraft' : 'payload');
  const d = tp ? 'Distance to ' + what + ': <b>' + fmtD(dist(p, tp)) + '</b><br>' : '';
  new maplibregl.Popup().setLngLat(e.lngLat)
    .setHTML('<b>' + t + '</b><br>' + d + '<button onclick="copyText(\'' + t + '\', this)">Copy</button>').addTo(map);
});

map.once('style.load', () => {
  ['fp'].forEach(id => map.addSource(id, {type:'geojson', data:EMPTY}));
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

function clearFootprint() {
  map.getSource('fp').setData(EMPTY);
  Object.values(cornerMk).forEach(hide); hide(cMk);
}
function drawCam(m) {
  const t = m.target ? '<br><small>Target: ' + fmtLL(m.target) + '</small>' : '';
  if (!m.camera) { build3d(null); return t; }
  build3d(m);
  return (m.corners ? '' : '<br><small>Payload: ' + fmtLL(m.camera) + '</small>') + t;
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
  if (m.corners) {
    map.getSource('fp').setData({type:'Feature', geometry:{type:'Polygon',
      coordinates:[[...m.corners.map(ll), ll(m.corners[0])]]}});
    m.corners.forEach((c, i) => {
      const k = m.labels[i];
      if (!cornerMk[k]) cornerMk[k] = mkMarker('#f0dc82', k);
      show(cornerMk[k], c);
    });
    show(cMk, m.center);
  } else clearFootprint();
  if (m.drone) show(dMk, m.drone);
  if (m.target) show(tMk, m.target); else hide(tMk);

  const html0 = drawCam(m);
  if (fitMode === 0) { fit(); fitMode = m.corners ? 2 : 1; }
  else if (fitMode === 1 && m.corners) { fit(); fitMode = 2; }
  else if (follow && copyTarget()) map.jumpTo({center:ll(copyTarget())});

  let html;
  if (m.corners) {
    const far = dist(m.corners[0], m.corners[1]), near = dist(m.corners[3], m.corners[2]);
    const depth = (dist(m.corners[0], m.corners[3]) + dist(m.corners[1], m.corners[2])) / 2;
    html = '<b>' + fmtLL(m.center) + '</b><br><small>Footprint ' + fmtD((far + near) / 2) + ' × ' + fmtD(depth) +
      '</small>' + (Math.max(far, near, depth) > 2000 ?
      '<br><span id="warn">Payload angle too low, observed area is inaccurate.</span>' : '');
  } else {
    html = (m.drone ? '<b>' + fmtLL(m.drone) + '</b><br>' : '') +
      '<span id="warn">Values not suitable for drawing the footprint.</span>';
  }
  $('hud').style.display = 'block';
  $('hud').innerHTML = html + html0;
}

function setStatus(color, text) { $('dot').style.background = color; $('stxt').textContent = text; }
setInterval(() => {
  const now = performance.now();
  const lost = !!(last && now - lastMsgT > 3000);
  if (ready && lost !== lostPrev) {
    lostPrev = lost;
    map.setPaintProperty('fp-fill', 'fill-color', lost ? '#9e9e9e' : '#f0dc82');
    map.setPaintProperty('fp-line', 'line-color', lost ? '#9e9e9e' : '#f0dc82');
    $('camLbl').style.opacity = lost ? .4 : 1;
    allMk().forEach(mk => mk.getElement().style.opacity = lost ? .4 : 1);
    T.mats.forEach(mt => mt.opacity = mt.userData.op * (lost ? .35 : 1));
    map.triggerRepaint();
  }
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
    ap.add_argument("--cam", help="fallback camera position when the stream has none: lat,lon[,alt]")
    ap.add_argument("--ground-alt", type=float, default=0.0,
                    help="ground elevation (m, MSL) under the camera; subtracted from altitude to get height above ground")
    args = ap.parse_args()
    GROUND_ALT[0] = args.ground_alt

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

    url = f"http://127.0.0.1:{args.port}"
    print(f"GPS Map running at {url} (UDP data on port {args.udp_port})")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=args.port, threaded=True)


if __name__ == "__main__":
    main()