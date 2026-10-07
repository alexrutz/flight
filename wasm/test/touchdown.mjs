import { createHarness, setupA320 } from './harness.mjs';
for (const vsTarget of [-300, -600, -900]) {
  const sim = await createHarness();
  const elevFt = sim.elev / 0.3048;
  setupA320(sim, { airborne: true, kts: 135, altFt: elevFt + 11 + 40, conf: 4, gear: 1, gamma: -3 });
  sim.set('athr/manual-thrust', 0.3);
  // fly a constant sink rate into the ground by commanding pitch through the stick
  let nzMax = 0, td = null, nzF = null, nzFMax = 0;
  for (let i = 0; i < 120 * 15; i++) {
    const vs = sim.get('velocities/h-dot-fps') * 60;
    sim.set('fbw/stick-pitch', Math.max(-0.5, Math.min(0.5, (vs - vsTarget) * -0.002)));
    sim.step(1);
    if (sim.get('gear/unit[1]/WOW') && !td) td = { vs, t: i };
    nzF = (nzF ?? 1) + (sim.get('accelerations/Nz') - (nzF ?? 1)) * (1 / 12);
    if (td && i - td.t < 120) { nzMax = Math.max(nzMax, sim.get('accelerations/Nz')); nzFMax = Math.max(nzFMax, nzF); }
    if (td && i - td.t > 240) break;
  }
  console.log(`target ${vsTarget} fpm: touchdown ${td?.vs.toFixed(0)} fpm, peak Nz ${nzMax.toFixed(2)} g (filtered ${nzFMax.toFixed(2)}), settled agl ${sim.get('position/h-agl-ft').toFixed(1)} ft`);
}
