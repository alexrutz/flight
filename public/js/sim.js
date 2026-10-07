// JSBSim (compiled to WebAssembly) wrapped in a small property-based API,
// plus the terrain patch that JSBSim's ground model samples.
import createJSBSim from '../jsbsim/jsbsim.js';
import { metersPerDeg } from './geo.js';
import { makeSampler, ensureDetail, detailReady, DETAIL_ZOOM } from './elevation.js';

const PATCH_N = 193; // samples per side
const PATCH_SIZE = 1500; // meters per side (~8 m resolution)

let modulePromise = null;
function wasm() {
  if (!modulePromise) {
    modulePromise = createJSBSim({
      locateFile: (file) => new URL(`../jsbsim/${file}`, import.meta.url).href,
    });
  }
  return modulePromise;
}

export async function createSim(model) {
  const M = await wasm();
  const load = M.cwrap('jsb_load', 'number', ['string']);
  const lookup = M.cwrap('jsb_prop', 'number', ['string', 'number']);
  if (!load(model)) throw new Error(`JSBSim could not load ${model}`);

  const ids = new Map();
  const id = (name) => {
    let i = ids.get(name);
    if (i === undefined) {
      i = lookup(name, 1);
      if (i < 0) throw new Error(`Unknown JSBSim property: ${name}`);
      ids.set(name, i);
    }
    return i;
  };
  const patchBuf = M._malloc(PATCH_N * PATCH_N * 4);
  const patch = { lat: null, lon: null, complete: false, dirty: false };

  const sim = {
    dt: M._jsb_dt(),
    model,
    get: (name) => M._jsb_get(id(name)),
    set: (name, value) => M._jsb_set(id(name), value),
    runIC: () => M._jsb_run_ic() === 1,
    trim: (mode = 1) => M._jsb_trim(mode) === 1,
    seedFcc: (elevatorNoseUp, airborne) => M._jsb_fcc_seed(elevatorNoseUp, airborne ? 1 : 0),
    step: (n) => M._jsb_step(n) === 1,

    // Rebuilds the ground patch around a point (the detail tiles must be loaded).
    setPatch(lat, lon) {
      const m = metersPerDeg(lat);
      const dLat = PATCH_SIZE / (PATCH_N - 1) / m.lat, dLon = PATCH_SIZE / (PATCH_N - 1) / m.lon;
      const latN = lat + ((PATCH_N - 1) / 2) * dLat, lonW = lon - ((PATCH_N - 1) / 2) * dLon;
      const heap = M.HEAPF32, base = patchBuf / 4;
      const sample = makeSampler(DETAIL_ZOOM, lat, lon, PATCH_SIZE);
      for (let j = 0; j < PATCH_N; j++)
        for (let i = 0; i < PATCH_N; i++) heap[base + j * PATCH_N + i] = sample(latN - j * dLat, lonW + i * dLon);
      M._jsb_set_patch(patchBuf, PATCH_N, PATCH_N, latN, lonW, dLat, dLon);
      patch.lat = lat;
      patch.lon = lon;
    },
    // Keeps the patch centred on the aircraft. Rebuilds immediately with the
    // best data available, and again once the detail tiles have arrived.
    updatePatch(lat, lon) {
      const m = metersPerDeg(lat);
      const moved = patch.lat === null || Math.abs(lat - patch.lat) * m.lat > PATCH_SIZE / 4 || Math.abs(lon - patch.lon) * m.lon > PATCH_SIZE / 4;
      if (!moved && !patch.dirty) return;
      const r = PATCH_SIZE / 2 / m.lat, ro = PATCH_SIZE / 2 / m.lon;
      patch.dirty = false;
      patch.complete = detailReady(lat - r, lat + r, lon - ro, lon + ro);
      sim.setPatch(lat, lon);
      if (!patch.complete) ensureDetail(lat - r, lat + r, lon - ro, lon + ro).then(() => (patch.dirty = true));
    },
    patchAt: (lat, lon) => M._jsb_patch_at(lat, lon),
  };
  return sim;
}
