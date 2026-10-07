// Headless JSBSim harness (Node build) for tuning the A320 model and control laws.
import createJSBSim from '../out-node/jsbsim.mjs';

export const FT = 0.3048;

export async function createHarness({ model = 'a320', lat = 48.35, lon = 11.78, elev = 450 } = {}) {
  const M = await createJSBSim();
  // flat terrain patch around the test location
  const rows = 64, cols = 64, dLat = 0.01, dLon = 0.015;
  const ptr = M._malloc(rows * cols * 4);
  M.HEAPF32.fill(elev, ptr / 4, ptr / 4 + rows * cols);
  M._jsb_set_patch(ptr, rows, cols, lat + (rows / 2) * dLat, lon - (cols / 2) * dLon, dLat, dLon);
  M._free(ptr);
  const load = M.cwrap('jsb_load', 'number', ['string']);
  const lookup = M.cwrap('jsb_prop', 'number', ['string', 'number']);
  if (!load(model)) throw new Error('load failed');
  const ids = new Map();
  const id = (n) => {
    if (!ids.has(n)) {
      const i = lookup(n, 1);
      if (i < 0) throw new Error('no prop ' + n);
      ids.set(n, i);
    }
    return ids.get(n);
  };
  const sim = {
    M,
    elev,
    get: (n) => M._jsb_get(id(n)),
    set: (n, v) => M._jsb_set(id(n), v),
    exists: (n) => lookup(n, 0) >= 0,
    runIC: () => M._jsb_run_ic() === 1,
    trim: (m = 1) => M._jsb_trim(m) === 1,
    seed: (e, air) => M._jsb_fcc_seed(e, air ? 1 : 0),
    step: (n) => M._jsb_step(n) === 1,
    secs(s, each) {
      const n = Math.round(s * 120);
      for (let i = 0; i < n; i++) {
        if (each) each(i / 120);
        if (!sim.step(1)) throw new Error('step failed');
      }
    },
    snap(extra = []) {
      const g = sim.get;
      const o = {
        t: g('simulation/sim-time-sec').toFixed(1),
        kcas: g('velocities/vc-kts').toFixed(1),
        alt: g('position/h-sl-ft').toFixed(0),
        agl: (g('position/h-agl-ft')).toFixed(1),
        vs: (g('velocities/h-dot-fps') * 60).toFixed(0),
        pitch: g('attitude/theta-deg').toFixed(1),
        bank: g('attitude/phi-deg').toFixed(1),
        hdg: g('attitude/psi-deg').toFixed(1),
        alpha: g('aero/alpha-deg').toFixed(1),
        nz: g('accelerations/Nz').toFixed(2),
        elev: g('fcs/elevator-cmd-norm').toFixed(3),
        thr: g('fcs/throttle-cmd-norm[0]').toFixed(2),
        n1: g('propulsion/engine[0]/n1').toFixed(1),
        law: g('fbw/law'),
      };
      for (const e of extra) o[e] = +g(e).toFixed(3);
      return Object.entries(o).map(([k, v]) => `${k}=${v}`).join(' ');
    },
  };
  return sim;
}

// Common A320 initialisation
export function setupA320(sim, { airborne, kts = 250, altFt = 5000, conf = 0, gear = 0, psi = 0, gamma = 0, throttle = 0.6 }) {
  sim.set('fbw/active', 1);
  sim.set('ic/lat-geod-deg', 48.35);
  sim.set('ic/long-gc-deg', 11.78);
  sim.set('ic/psi-true-deg', psi);
  if (airborne) {
    sim.set('ic/h-sl-ft', altFt);
    sim.set('ic/vc-kts', kts);
    sim.set('ic/gamma-deg', gamma);
  } else {
    sim.set('ic/h-agl-ft', 15.0);
    sim.set('ic/vc-kts', 0);
  }
  if (!sim.runIC()) throw new Error('runIC');
  sim.set('fcs/flap-cmd-norm', conf / 4); // lever 0..4 normalised
  sim.set('gear/gear-cmd-norm', gear);
  sim.set('propulsion/set-running', -1);
  for (const p of ['fbw/stick-pitch', 'fbw/stick-roll', 'fbw/pedals', 'athr/active']) sim.set(p, 0);
  if (airborne) {
    sim.set('fcs/throttle-cmd-norm[0]', throttle);
    sim.set('fcs/throttle-cmd-norm[1]', throttle);
    const ok = sim.trim(1);
    const thr = sim.get('fcs/throttle-cmd-norm[0]');
    sim.set('athr/manual-thrust', thr);
    // hand the trimmed elevator over to the flight control computer
    sim.seed(-(sim.get('fcs/elevator-cmd-norm') + sim.get('fcs/pitch-trim-cmd-norm')), true);
    sim.set('fcs/pitch-trim-cmd-norm', 0);
    return ok;
  }
  sim.set('athr/manual-thrust', 0);
  sim.seed(0, false);
  return true;
}
