// Geodesy helpers: WGS84 <-> ECEF <-> local scene frame, and web-mercator tiles.
//
// The scene uses an East-North-Up tangent frame at a chosen origin, mapped to
// Three.js axes as x = east, y = up, z = south. Everything (terrain, runways,
// aircraft) is placed through exact ECEF math, so the earth's curvature and
// directions are correct anywhere within a few hundred kilometres.
import * as THREE from 'three';

export const DEG = Math.PI / 180;
export const FT = 0.3048;
export const NM = 1852;
const A = 6378137.0;
const F = 1 / 298.257223563;
const E2 = F * (2 - F);

export function toEcef(latDeg, lonDeg, h, out = [0, 0, 0]) {
  const lat = latDeg * DEG, lon = lonDeg * DEG;
  const s = Math.sin(lat), c = Math.cos(lat);
  const n = A / Math.sqrt(1 - E2 * s * s);
  out[0] = (n + h) * c * Math.cos(lon);
  out[1] = (n + h) * c * Math.sin(lon);
  out[2] = (n * (1 - E2) + h) * s;
  return out;
}

// Meters per degree at a latitude (for small local offsets).
export function metersPerDeg(latDeg) {
  const p = latDeg * DEG;
  return {
    lat: 111132.92 - 559.82 * Math.cos(2 * p) + 1.175 * Math.cos(4 * p),
    lon: 111412.84 * Math.cos(p) - 93.5 * Math.cos(3 * p),
  };
}

// Local east/north/up unit vectors (ECEF) at a location.
export function enuBasis(latDeg, lonDeg) {
  const lat = latDeg * DEG, lon = lonDeg * DEG;
  const sl = Math.sin(lat), cl = Math.cos(lat), so = Math.sin(lon), co = Math.cos(lon);
  return {
    east: [-so, co, 0],
    north: [-sl * co, -sl * so, cl],
    up: [cl * co, cl * so, sl],
  };
}

// The scene frame. Call setOrigin() before building anything.
export const frame = {
  lat: 0,
  lon: 0,
  ecef: [0, 0, 0],
  basis: enuBasis(0, 0),
};

export function setOrigin(latDeg, lonDeg) {
  frame.lat = latDeg;
  frame.lon = lonDeg;
  toEcef(latDeg, lonDeg, 0, frame.ecef);
  frame.basis = enuBasis(latDeg, lonDeg);
}

const tmp = [0, 0, 0];
// Geodetic -> scene coordinates (meters).
export function toScene(latDeg, lonDeg, h, out = new THREE.Vector3()) {
  toEcef(latDeg, lonDeg, h, tmp);
  const dx = tmp[0] - frame.ecef[0], dy = tmp[1] - frame.ecef[1], dz = tmp[2] - frame.ecef[2];
  const { east, north, up } = frame.basis;
  out.x = dx * east[0] + dy * east[1] + dz * east[2];
  out.y = dx * up[0] + dy * up[1] + dz * up[2];
  out.z = -(dx * north[0] + dy * north[1] + dz * north[2]);
  return out;
}

// ECEF direction -> scene direction
function dirToScene(v, out) {
  const { east, north, up } = frame.basis;
  out.x = v[0] * east[0] + v[1] * east[1] + v[2] * east[2];
  out.y = v[0] * up[0] + v[1] * up[1] + v[2] * up[2];
  out.z = -(v[0] * north[0] + v[1] * north[1] + v[2] * north[2]);
  return out;
}

// Rotation from a location's local frame (x east, y up, z south) into the scene frame.
const _m = new THREE.Matrix4(), _e = new THREE.Vector3(), _u = new THREE.Vector3(), _s = new THREE.Vector3();
export function localFrameQuat(latDeg, lonDeg, out = new THREE.Quaternion()) {
  const b = enuBasis(latDeg, lonDeg);
  dirToScene(b.east, _e);
  dirToScene(b.up, _u);
  dirToScene(b.north, _s).negate();
  _m.makeBasis(_e, _u, _s);
  return out.setFromRotationMatrix(_m);
}

// Local "up" direction in the scene at a location.
export function upAt(latDeg, lonDeg, out = new THREE.Vector3()) {
  return dirToScene(enuBasis(latDeg, lonDeg).up, out);
}

// Inverse: scene position -> approximate lat/lon/h (iterative, for cameras etc.)
export function fromScene(p) {
  const { east, north, up } = frame.basis;
  const x = frame.ecef[0] + p.x * east[0] + p.y * up[0] - p.z * north[0];
  const y = frame.ecef[1] + p.x * east[1] + p.y * up[1] - p.z * north[1];
  const z = frame.ecef[2] + p.x * east[2] + p.y * up[2] - p.z * north[2];
  const lon = Math.atan2(y, x);
  const pr = Math.hypot(x, y);
  let lat = Math.atan2(z, pr * (1 - E2)), h = 0;
  for (let i = 0; i < 4; i++) {
    const s = Math.sin(lat);
    const n = A / Math.sqrt(1 - E2 * s * s);
    h = pr / Math.cos(lat) - n;
    lat = Math.atan2(z, pr * (1 - (E2 * n) / (n + h)));
  }
  return { lat: lat / DEG, lon: lon / DEG, h };
}

// Great-circle distance (m) and initial bearing (deg) between two points.
export function distance(lat1, lon1, lat2, lon2) {
  const p1 = lat1 * DEG, p2 = lat2 * DEG, dp = p2 - p1, dl = (lon2 - lon1) * DEG;
  const a = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return 2 * 6371008.8 * Math.asin(Math.min(1, Math.sqrt(a)));
}
export function bearing(lat1, lon1, lat2, lon2) {
  const p1 = lat1 * DEG, p2 = lat2 * DEG, dl = (lon2 - lon1) * DEG;
  const y = Math.sin(dl) * Math.cos(p2);
  const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
  return ((Math.atan2(y, x) / DEG) + 360) % 360;
}
// Destination point from a start, bearing (deg) and distance (m).
export function destination(lat, lon, brgDeg, dist) {
  const d = dist / 6371008.8, b = brgDeg * DEG, p1 = lat * DEG, l1 = lon * DEG;
  const p2 = Math.asin(Math.sin(p1) * Math.cos(d) + Math.cos(p1) * Math.sin(d) * Math.cos(b));
  const l2 = l1 + Math.atan2(Math.sin(b) * Math.sin(d) * Math.cos(p1), Math.cos(d) - Math.sin(p1) * Math.sin(p2));
  return [p2 / DEG, ((l2 / DEG + 540) % 360) - 180];
}

// --- Web mercator tiles
export function tileOf(latDeg, lonDeg, z) {
  const n = 2 ** z;
  const x = ((lonDeg + 180) / 360) * n;
  const s = Math.sin(latDeg * DEG);
  const y = (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * n;
  return [x, y]; // fractional
}
export function tileLon(x, z) {
  return (x / 2 ** z) * 360 - 180;
}
export function tileLat(y, z) {
  const n = Math.PI - (2 * Math.PI * y) / 2 ** z;
  return (Math.atan(Math.sinh(n)) * 180) / Math.PI;
}
export const wrapDeg = (a) => ((a % 360) + 360) % 360;
export const angleDiff = (a, b) => ((a - b + 540) % 360) - 180;
