// Exercises the A320 model and flight control laws. Usage: node test/a320-laws.mjs [case]
import { createHarness, setupA320 } from './harness.mjs';

const only = process.argv[2];
const cases = {
  async props(sim) {
    for (const p of ['accelerations/Nz', 'fcs/steer-cmd-norm', 'propulsion/engine[0]/n1', 'propulsion/engine[0]/reverser-angle-rad',
      'gear/unit[0]/WOW', 'contact/unit[3]/WOW', 'fcs/conf-pos', 'fcs/flap-pos-deg', 'fcs/slat-pos-deg', 'flight-path/gamma-rad'])
      console.log(p, sim.exists(p));
    setupA320(sim, { airborne: true });
    console.log('weight lbs', sim.get('inertia/weight-lbs').toFixed(0), 'Nz level', sim.get('accelerations/Nz').toFixed(3));
  },
  async cruise(sim) {
    console.log('trim', setupA320(sim, { airborne: true, kts: 250, altFt: 5000 }));
    console.log(sim.snap());
    sim.secs(20);
    console.log('hands off 20s', sim.snap());
    sim.set('fbw/stick-pitch', 0.5);
    for (let i = 0; i < 6; i++) { sim.secs(0.5); console.log(' pull 0.5', sim.snap(['fbw/nz-cmd'])); }
    sim.set('fbw/stick-pitch', 0);
    sim.secs(5);
    console.log('released 5s', sim.snap(['flight-path/gamma-rad']));
    sim.secs(10);
    console.log('released 15s', sim.snap(['flight-path/gamma-rad']));
  },
  async roll(sim) {
    setupA320(sim, { airborne: true, kts: 250, altFt: 5000 });
    sim.set('fbw/stick-roll', 1);
    for (let i = 0; i < 4; i++) { sim.secs(0.5); console.log(' full right', sim.snap(['fbw/phi-target-deg'])); }
    sim.set('fbw/stick-roll', 0);
    sim.secs(10);
    console.log('released 10s', sim.snap(['fbw/phi-target-deg', 'aero/beta-deg']));
    sim.set('fbw/stick-roll', 1);
    sim.secs(2.5);
    console.log('to ~60', sim.snap());
    sim.set('fbw/stick-roll', 0);
    sim.secs(8);
    console.log('released (expect 33)', sim.snap(['fbw/phi-target-deg']));
    sim.set('fbw/stick-roll', -1); sim.secs(3); sim.set('fbw/stick-roll', 0); sim.secs(8);
    console.log('rolled back', sim.snap());
  },
  async alphaprot(sim) {
    setupA320(sim, { airborne: true, kts: 200, altFt: 8000 });
    sim.set('athr/manual-thrust', 0);
    sim.set('fbw/stick-pitch', 1);
    for (let i = 0; i < 10; i++) { sim.secs(3); console.log(' full back, idle', sim.snap(['fbw/alpha-prot-active', 'fbw/alpha-max-deg'])); }
  },
  async athr(sim) {
    setupA320(sim, { airborne: true, kts: 250, altFt: 5000 });
    sim.set('athr/active', 1); sim.set('athr/thrust-limit', 0.9); sim.set('athr/speed-cmd-kts', 220);
    for (let i = 0; i < 8; i++) { sim.secs(5); console.log(' athr 220', sim.snap()); }
    sim.set('athr/speed-cmd-kts', 260);
    for (let i = 0; i < 8; i++) { sim.secs(5); console.log(' athr 260', sim.snap()); }
  },
  async takeoff(sim) {
    setupA320(sim, { airborne: false, conf: 1, gear: 1, psi: 260 });
    sim.secs(3);
    console.log('static', sim.snap(['gear/unit[0]/WOW', 'gear/unit[1]/WOW', 'gear/unit[0]/compression-ft', 'gear/unit[1]/compression-ft']));
    sim.set('athr/manual-thrust', 1);
    let rotated = false, t0 = 0;
    for (let i = 0; i < 80; i++) {
      sim.secs(0.5);
      const v = sim.get('velocities/vc-kts');
      if (v > 145 && !rotated) { rotated = true; sim.set('fbw/stick-pitch', 0.6); console.log('ROTATE', sim.snap()); }
      if (rotated && sim.get('attitude/theta-deg') > 14) sim.set('fbw/stick-pitch', 0);
      if (sim.get('position/h-agl-ft') > 100) sim.set('gear/gear-cmd-norm', 0);
      if (i >= 60 || rotated) console.log(' ', sim.snap(['gear/unit[1]/WOW', 'contact/unit[3]/WOW']));
      if (sim.get('contact/unit[3]/WOW')) console.log('TAIL STRIKE');
    }
  },
  async approach(sim) {
    const elevFt = sim.elev / 0.3048;
    console.log('trim', setupA320(sim, { airborne: true, kts: 137, altFt: elevFt + 1500, conf: 4, gear: 1, gamma: -3 }));
    sim.set('athr/active', 1); sim.set('athr/thrust-limit', 0.9); sim.set('athr/speed-cmd-kts', 137);
    sim.secs(2);
    console.log('trimmed', sim.snap(['fcs/conf-pos', 'flight-path/gamma-rad']));
    let flared = false;
    let last = '';
    for (let i = 0; i < 3000; i++) {
      sim.secs(0.05);
      const ra = sim.get('position/h-agl-ft') - 11;
      if (ra < 30 && !flared) { flared = true; sim.set('athr/active', 0); sim.set('athr/manual-thrust', 0); sim.set('fbw/stick-pitch', 0.35); console.log('FLARE', sim.snap()); }
      if (i % 400 === 0 || (ra < 60 && i % 6 === 0)) console.log(' ', sim.snap(['fbw/law']));
      if (sim.get('gear/unit[1]/WOW')) { console.log('TOUCHDOWN', sim.snap()); break; }
    }
    sim.set('fbw/stick-pitch', 0);
    sim.set('fcs/spoiler-cmd-norm', 1);
    sim.set('fcs/left-brake-cmd-norm', 0.5); sim.set('fcs/right-brake-cmd-norm', 0.5);
    for (let i = 0; i < 8; i++) { sim.secs(2); console.log(' rollout', sim.snap(['gear/unit[0]/WOW'])); }
  },
};

for (const [name, fn] of Object.entries(cases)) {
  if (only && name !== only) continue;
  console.log(`\n=== ${name}`);
  const sim = await createHarness();
  try { await fn(sim); } catch (e) { console.log('ERROR', e.message); }
}
