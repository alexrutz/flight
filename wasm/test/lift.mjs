import { createHarness, setupA320 } from './harness.mjs';
const sim = await createHarness();
for (const [conf, kts] of [[0, 220], [1, 150], [2, 140], [4, 137]]) {
  setupA320(sim, { airborne: true, kts, altFt: 3000, conf, gear: conf ? 1 : 0 });
  sim.secs(1);
  const f = (n) => sim.get(n).toFixed(0);
  console.log(`conf ${conf} ${kts}kt: alpha ${sim.get('aero/alpha-deg').toFixed(2)}  conf-pos ${sim.get('fcs/conf-pos').toFixed(2)}  CLbasic ${f('aero/coefficient/CLbasic')} CLge ${f('aero/coefficient/CLge')} CLmach ${f('aero/coefficient/CLmach')} CLde ${f('aero/coefficient/CLde')} CLq ${f('aero/coefficient/CLq')} weight ${f('inertia/weight-lbs')} qS ${(sim.get('aero/qbar-psf') * 1319.7).toFixed(0)}`);
}
