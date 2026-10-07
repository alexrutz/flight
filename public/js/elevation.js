// Terrain elevation from Terrarium tiles (AWS Open Data terrain tiles),
// streamed through the server's tile cache, with runway flattening.
//
// elevationAt() is the single source of truth for both the rendered terrain
// and the ground patch handed to JSBSim.
import { tileOf, tileLat, tileLon } from './geo.js';
import { runwaysNear, runwayCoords } from './airports.js';

export const DETAIL_ZOOM = 12; // ~25 m samples in mid latitudes
const tiles = new Map(); // "z/x/y" -> { data: Float32Array | null, promise }

function decodeTerrarium(img) {
  const cv = typeof OffscreenCanvas !== 'undefined' ? new OffscreenCanvas(256, 256) : Object.assign(document.createElement('canvas'), { width: 256, height: 256 });
  const g = cv.getContext('2d', { willReadFrequently: true });
  g.drawImage(img, 0, 0);
  const px = g.getImageData(0, 0, 256, 256).data;
  const out = new Float32Array(256 * 256);
  for (let i = 0; i < out.length; i++) {
    let h = px[i * 4] * 256 + px[i * 4 + 1] + px[i * 4 + 2] / 256 - 32768;
    if (h < -10) h = 0; // open water: use the sea surface
    out[i] = h;
  }
  return out;
}

export function loadElevationTile(z, x, y) {
  const key = `${z}/${x}/${y}`;
  let t = tiles.get(key);
  if (!t) {
    t = { data: null };
    t.promise = fetch(`tiles/elev/${key}.png`)
      .then((r) => (r.ok ? r.blob() : Promise.reject(new Error(r.status))))
      .then((b) => createImageBitmap(b, { colorSpaceConversion: 'none', premultiplyAlpha: 'none' }))
      .then((img) => {
        t.data = decodeTerrarium(img);
        img.close?.();
        return t.data;
      })
      .catch(() => {
        t.data = new Float32Array(256 * 256); // missing tile: sea level
        return t.data;
      });
    tiles.set(key, t);
  }
  return t.promise;
}

export function hasTile(z, x, y) {
  return !!tiles.get(`${z}/${x}/${y}`)?.data;
}

// Raw elevation at a zoom level (tile must be loaded; otherwise null).
export function rawAt(lat, lon, z) {
  const [fx, fy] = tileOf(lat, lon, z);
  const tx = Math.floor(fx), ty = Math.floor(fy);
  const t = tiles.get(`${z}/${tx}/${ty}`);
  if (!t?.data) return null;
  // pixel centers are at +0.5
  const px = Math.min(Math.max((fx - tx) * 256 - 0.5, 0), 255), py = Math.min(Math.max((fy - ty) * 256 - 0.5, 0), 255);
  const i = Math.min(254, Math.floor(px)), j = Math.min(254, Math.floor(py));
  const u = px - i, v = py - j, d = t.data, k = j * 256 + i;
  const a = d[k], b = d[k + 1], c = d[k + 256], e = d[k + 257];
  return a + (b - a) * u + (c - a) * v + (a - b - c + e) * u * v;
}

// Best available elevation: the requested zoom, else the nearest loaded coarser level.
export function rawBest(lat, lon, z) {
  for (let zz = z; zz >= 4; zz--) {
    const h = rawAt(lat, lon, zz);
    if (h !== null) return h;
  }
  return 0;
}

// True when all detail tiles covering a box are loaded.
export function detailReady(latS, latN, lonW, lonE, z = DETAIL_ZOOM) {
  const [x0, y0] = tileOf(latN, lonW, z), [x1, y1] = tileOf(latS, lonE, z);
  for (let x = Math.floor(x0); x <= Math.floor(x1); x++)
    for (let y = Math.floor(y0); y <= Math.floor(y1); y++) if (!hasTile(z, x, y)) return false;
  return true;
}

// Ensures the detail tiles covering a lat/lon box are loaded.
export function ensureDetail(latS, latN, lonW, lonE, z = DETAIL_ZOOM) {
  const [x0, y0] = tileOf(latN, lonW, z), [x1, y1] = tileOf(latS, lonE, z);
  const jobs = [];
  for (let x = Math.floor(x0); x <= Math.floor(x1); x++)
    for (let y = Math.floor(y0); y <= Math.floor(y1); y++) jobs.push(loadElevationTile(z, x, y));
  return Promise.all(jobs);
}

// ----------------------------------------------------------------- runways
const FLAT_EXTRA = 40; // m of flat shoulder beside the runway
const FLAT_END = 80; // m flat beyond the runway ends
const BLEND = 160; // m blend back into the natural terrain

// Fits a straight surface profile to the terrain along the runway.
// Requires the detail tiles around it to be loaded.
export async function prepareRunway(rw) {
  if (rw.profile) return rw.profile;
  const pad = (rw.lengthM / 2 + 600) / rw.mLat;
  const padLon = (rw.lengthM / 2 + 600) / rw.mLon;
  await ensureDetail(rw.centerLat - pad, rw.centerLat + pad, rw.centerLon - padLon, rw.centerLon + padLon);
  if (rw.profile) return rw.profile;
  const n = 21, ss = [], hs = [];
  for (let i = 0; i < n; i++) {
    const s = (rw.lengthM * i) / (n - 1);
    const lat = rw.le.lat + (s * rw.uy) / rw.mLat, lon = rw.le.lon + (s * rw.ux) / rw.mLon;
    ss.push(s);
    hs.push(rawAt(lat, lon, DETAIL_ZOOM) ?? 0);
  }
  // least squares line h = a + b s, slope limited to 1.5 %
  const ms = ss.reduce((p, q) => p + q, 0) / n, mh = hs.reduce((p, q) => p + q, 0) / n;
  let num = 0, den = 0;
  for (let i = 0; i < n; i++) {
    num += (ss[i] - ms) * (hs[i] - mh);
    den += (ss[i] - ms) ** 2;
  }
  const b = Math.max(-0.015, Math.min(0.015, den ? num / den : 0));
  const a = mh - b * ms;
  rw.profile = (s) => a + b * Math.min(Math.max(s, -FLAT_END - BLEND), rw.lengthM + FLAT_END + BLEND);
  return rw.profile;
}

// Blends the natural terrain into the runway surfaces nearby.
function flatten(lat, lon, h, runways) {
  let best = h, bestW = 0;
  for (const rw of runways) {
    if (!rw.profile) continue;
    const { s, c } = runwayCoords(rw, lat, lon);
    const ds = s < -FLAT_END ? -FLAT_END - s : s > rw.lengthM + FLAT_END ? s - rw.lengthM - FLAT_END : 0;
    const dc = Math.max(0, Math.abs(c) - rw.widthM / 2 - FLAT_EXTRA);
    const d = Math.max(ds, dc);
    if (d >= BLEND) continue;
    const t = d / BLEND;
    const w = 1 - t * t * (3 - 2 * t);
    if (w > bestW) {
      bestW = w;
      best = h * (1 - w) + rw.profile(s) * w;
    }
  }
  return best;
}

// Returns a sampler for an area (runways pre-filtered for speed).
export function makeSampler(z, centerLat, centerLon, radiusM) {
  const runways = runwaysNear(centerLat, centerLon, radiusM + 1000).filter((r) => r.profile);
  return (lat, lon) => {
    const h = rawBest(lat, lon, z);
    return runways.length ? flatten(lat, lon, h, runways) : h;
  };
}

// Full-detail elevation (meters) used for collisions.
export function elevationAt(lat, lon) {
  return makeSampler(DETAIL_ZOOM, lat, lon, 0)(lat, lon);
}

export { tileLat, tileLon };
