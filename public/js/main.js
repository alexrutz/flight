// Sky Hopper flight training simulator: boot, scenario setup, main loop.
import * as THREE from 'three';
import { setOrigin, toScene, localFrameQuat, upAt, FT, NM, DEG, destination, angleDiff, wrapDeg, fromScene } from './geo.js';
import { loadAirports, findAirport, searchAirports, runwayEnds, ilsDeviation, runwayCoords } from './airports.js';
import { prepareRunway, ensureDetail, elevationAt } from './elevation.js';
import { Terrain } from './terrain.js';
import { Runways } from './runways.js';
import { Atmosphere } from './sky.js';
import { createSim } from './sim.js';
import { createPFD } from './pfd.js';
import { createAudio } from './audio.js';
import { Controls } from './controls.js';
import { createA320 } from './aircraft/a320-model.js';
import { createC172 } from './aircraft/c172-model.js';
import { A320Systems, FLAP_LABELS, DETENTS, speedsFor } from './aircraft/a320-systems.js';
import { C172Systems, C172_FLAP_LABELS } from './aircraft/c172-systems.js';

const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

// ---------------------------------------------------------------------------
// Aircraft definitions
// ---------------------------------------------------------------------------
const AIRCRAFT = {
  a320: {
    name: 'Airbus A320', fdm: 'a320', jet: true, model: createA320, raOffsetFt: 11, groundAglFt: 11.2,
    flapLabels: FLAP_LABELS, chase: [62, 13], hardLandingFpm: 600, crashFpm: 1000,
    contacts: { 3: 'tail', 4: 'Engine pod struck the ground', 5: 'Engine pod struck the ground', 6: 'Wingtip struck the ground', 7: 'Wingtip struck the ground', 8: 'Nose struck the ground', 9: 'Belly landing — gear not down' },
    starts: ['takeoff', 'intercept', 'ils10', 'final4'],
  },
  c172: {
    name: 'Cessna 172', fdm: 'c172p', jet: false, model: createC172, raOffsetFt: 4.4, groundAglFt: 4.7,
    flapLabels: C172_FLAP_LABELS, chase: [18, 4.5], hardLandingFpm: 500, crashFpm: 900,
    contacts: { 3: 'Propeller strike', 4: 'tail', 5: 'Wingtip struck the ground', 6: 'Wingtip struck the ground' },
    starts: ['takeoff', 'ils10', 'final4'],
  },
};
const START_LABELS = {
  takeoff: ['Takeoff', 'Lined up, ready'],
  intercept: ['ILS intercept', '15 NM, 3000 ft, AP on'],
  ils10: ['ILS 10 NM', 'Established, AP on'],
  final4: ['Final 4 NM', 'Hand-flown landing'],
};

// ---------------------------------------------------------------------------
// Rendering setup
// ---------------------------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance', logarithmicDepthBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
$('app').appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.3, 2e6);
const atmosphere = new Atmosphere(scene, renderer);
const pfd = createPFD($('pfd'));
const audio = createAudio();
const controls = new Controls();

let terrain = null, runways = null;
let model = null, acDef = AIRCRAFT.a320, systems = null, sim = null;

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------
const settings = {
  aircraft: 'a320', airport: 'EDDM', runway: '', start: 'final4', time: 'noon', vis: 'cavok', wind: 'calm',
  sound: true, invert: false, tilt: false, fpv: true,
};
try { Object.assign(settings, JSON.parse(localStorage.getItem('trainerSettings') || '{}')); } catch {}
const saveSettings = () => { try { localStorage.setItem('trainerSettings', JSON.stringify(settings)); } catch {} };

const WINDS = {
  calm: () => ({ dir: 0, kts: 0, turb: 0 }),
  head: (rwy) => ({ dir: rwy, kts: 12, turb: 1 }),
  cross: (rwy) => ({ dir: wrapDeg(rwy + 60), kts: 15, turb: 1 }),
  gusty: (rwy) => ({ dir: wrapDeg(rwy - 40), kts: 22, turb: 3 }),
};

// ---------------------------------------------------------------------------
// Flight state
// ---------------------------------------------------------------------------
const f = {
  lat: 0, lon: 0, altFt: 0, aglFt: 0, ra: 0, phi: 0, theta: 0, psi: 0, heading: 0, track: 0, alpha: 0, beta: 0,
  ias: 0, tas: 0, gs: 0, mach: 0, vs: 0, fpa: 0, nz: 1, nzF: 1, onGround: true, wowMain: true, weight: 0,
  confPos: 0, flapDeg: 0, gearPos: 1, spoiler: 0, speedbrake: 0, n1: [0, 0], rpm: 0, compression: [0, 0, 0],
  pos: new THREE.Vector3(), quat: new THREE.Quaternion(),
};
const flight = {
  mode: 'menu', // menu | loading | flying | paused | ended
  scenario: null,
  simAcc: 0,
  elapsed: 0,
  camMode: 'chase',
  airborneTime: 0,
  lastRa: 1e9,
  callouts: new Set(),
  touchdown: null,
  stopped: false,
  tailStrike: false,
  patchBusy: false,
};

function readState() {
  const g = sim.get;
  f.lat = g('position/lat-geod-deg');
  f.lon = g('position/long-gc-deg');
  f.altFt = g('position/h-sl-ft');
  f.aglFt = g('position/h-agl-ft');
  f.ra = f.aglFt - acDef.raOffsetFt;
  f.phi = g('attitude/phi-deg');
  f.theta = g('attitude/theta-deg');
  f.psi = g('attitude/psi-deg');
  f.heading = wrapDeg(f.psi);
  f.alpha = g('aero/alpha-deg');
  f.beta = g('aero/beta-deg');
  f.ias = g('velocities/vc-kts');
  f.tas = g('velocities/vtrue-kts');
  f.gs = g('velocities/vg-fps') / 1.68781;
  f.mach = g('velocities/mach');
  f.vs = g('velocities/h-dot-fps') * 60;
  f.fpa = g('flight-path/gamma-rad') / DEG;
  const vn = g('velocities/v-north-fps'), ve = g('velocities/v-east-fps');
  f.track = f.gs > 5 ? wrapDeg(Math.atan2(ve, vn) / DEG) : f.heading;
  f.nz = g('accelerations/Nz');
  f.nzF += (f.nz - f.nzF) * 0.25; // ~0.1 s filter at 30-60 fps, like a recorded load factor
  f.wowNose = g('gear/unit[0]/WOW') > 0;
  f.wowMain = g('gear/unit[1]/WOW') + g('gear/unit[2]/WOW') > 0;
  f.onGround = f.wowNose || f.wowMain;
  f.weight = g('inertia/weight-lbs');
  f.flapDeg = g('fcs/flap-pos-deg');
  if (acDef.jet) {
    f.confPos = g('fcs/conf-pos');
    f.gearPos = g('gear/gear-pos-norm');
    f.spoiler = g('fcs/spoiler-pos-norm');
    f.speedbrake = g('fcs/speedbrake-pos-norm');
    f.n1[0] = g('propulsion/engine[0]/n1');
    f.n1[1] = g('propulsion/engine[1]/n1');
  } else {
    f.rpm = g('propulsion/engine/engine-rpm');
    f.gearPos = 1;
  }
  for (let i = 0; i < 3; i++) f.compression[i] = g(`gear/unit[${i}]/compression-ft`) * FT;
  toScene(f.lat, f.lon, f.altFt * FT, f.pos);
  const local = new THREE.Quaternion().setFromEuler(new THREE.Euler(f.theta * DEG, -f.psi * DEG, -f.phi * DEG, 'YXZ'));
  localFrameQuat(f.lat, f.lon, f.quat).multiply(local);
}

// ---------------------------------------------------------------------------
// Scenario setup
// ---------------------------------------------------------------------------
function setLoading(text, progress) {
  $('loading').classList.remove('hidden');
  $('loadText').textContent = text;
  $('loadBar').style.width = `${Math.round(progress * 100)}%`;
}

async function startFlight() {
  const apt = findAirport(settings.airport);
  if (!apt) return showMenu('Airport not found.');
  const ends = runwayEnds(apt);
  const end = ends.find((e) => e.ident === settings.runway) || ends[0];
  acDef = AIRCRAFT[settings.aircraft] || AIRCRAFT.a320;
  let start = settings.start;
  if (!acDef.starts.includes(start)) start = 'final4';
  flight.mode = 'loading';
  $('menu').classList.add('hidden');
  try {
    await setupScenario(apt, end, start);
  } catch (err) {
    console.error(err);
    $('loading').classList.add('hidden');
    return showMenu(`Could not start: ${err.message}`);
  }
  $('loading').classList.add('hidden');
  enterFlight();
}

async function setupScenario(apt, end, start) {
  setLoading('Loading flight model…', 0.05);
  const simPromise = createSim(acDef.fdm);
  // New scene origin at the runway threshold
  setOrigin(end.thrLat, end.thrLon);
  terrain?.dispose();
  if (runways) scene.remove(runways.group);
  terrain = new Terrain(scene, renderer);
  runways = new Runways(scene);
  atmosphere.set(settings.time, settings.vis);
  if (model) scene.remove(model.group);
  model = acDef.model();
  scene.add(model.group);

  setLoading(`Preparing ${apt.ident} runways…`, 0.15);
  await Promise.all(apt.runways.map(prepareRunway));

  // Start position
  const rw = end.runway, dir = end === rw.le ? 1 : -1;
  const gsTan = Math.tan(3 * DEG);
  const thrElev = rw.profile(end.thresholdS);
  let p;
  if (start === 'takeoff') {
    const s = end.s + dir * 35;
    const [lat, lon] = rwPoint(rw, s);
    p = { lat, lon, ground: true, heading: end.heading };
  } else {
    const distNm = start === 'final4' ? 4 : start === 'ils10' ? 10 : 15;
    const dist = distNm * NM + (end.ils.gpiS - end.thresholdS) * dir;
    let [lat, lon] = destination(end.thrLat, end.thrLon, (end.heading + 180) % 360, distNm * NM);
    let heading = end.heading;
    let altM = thrElev + 50 * FT + distNm * NM * gsTan;
    if (start === 'intercept') {
      [lat, lon] = destination(lat, lon, wrapDeg(end.heading + 90), 2.5 * NM);
      heading = wrapDeg(end.heading - 30);
      altM = thrElev + 3000 * FT;
    }
    p = { lat, lon, ground: false, heading, altFt: altM / FT, dist };
  }
  setLoading('Loading terrain…', 0.25);
  const pad = 0.03;
  await ensureDetail(p.lat - pad, p.lat + pad, p.lon - pad * 1.5, p.lon + pad * 1.5);

  sim = await simPromise;
  sim.setPatch(p.lat, p.lon);
  systems = acDef.jet ? new A320Systems(sim) : new C172Systems(sim);
  configureSim(apt, end, start, p);

  readState();
  flight.scenario = { apt, end, start };
  flight.simAcc = 0;
  flight.elapsed = 0;
  flight.airborneTime = p.ground ? 0 : 100;
  flight.lastRa = f.ra;
  flight.callouts = new Set();
  flight.touchdown = null;
  flight.stopped = false;
  flight.tailStrike = false;
  flight.patchBusy = false;
  $('report').classList.add('hidden');
  resetCamera();
  updateCamera(0.016);

  // Warm up the terrain around the start position
  for (let i = 0; i < 400; i++) {
    terrain.update(camera.position);
    runways.update(camera.position, performance.now() + i * 10000);
    const pending = terrain.loadQueue.length + terrain.loading + terrain.buildQueue.length;
    setLoading(`Loading terrain… ${terrain.stats.visible} tiles`, 0.3 + 0.7 * Math.min(1, i / 60 + (pending ? 0 : 1)));
    if (!pending && i > 10) break;
    await new Promise((r) => setTimeout(r, 30));
  }
}

function rwPoint(rw, s) {
  const e = s * rw.ux, n = s * rw.uy;
  return [rw.le.lat + n / rw.mLat, rw.le.lon + e / rw.mLon];
}

function configureSim(apt, end, start, p) {
  const jet = acDef.jet;
  // weather
  const w = (WINDS[settings.wind] || WINDS.calm)(end.heading);
  const fps = w.kts * 1.68781;
  sim.set('atmosphere/wind-north-fps', -fps * Math.cos(w.dir * DEG));
  sim.set('atmosphere/wind-east-fps', -fps * Math.sin(w.dir * DEG));
  sim.set('atmosphere/turb-type', w.turb ? 3 : 0);
  sim.set('atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps', fps);
  sim.set('atmosphere/turbulence/milspec/severity', w.turb);
  flight.wind = w;

  sim.set('fbw/active', jet ? 1 : 0);
  for (const prop of ['fbw/stick-pitch', 'fbw/stick-roll', 'fbw/pedals', 'athr/active', 'fcs/aileron-cmd-norm', 'fcs/elevator-cmd-norm', 'fcs/rudder-cmd-norm', 'fcs/pitch-trim-cmd-norm'])
    sim.set(prop, 0);
  sim.set('ic/lat-geod-deg', p.lat);
  sim.set('ic/long-gc-deg', p.lon);
  sim.set('ic/psi-true-deg', p.heading);
  sim.set('ic/phi-deg', 0);
  sim.set('ic/theta-deg', 0);
  if (p.ground) {
    sim.set('ic/h-agl-ft', acDef.groundAglFt);
    sim.set('ic/vc-kts', 0);
    sim.set('ic/gamma-deg', 0);
  } else {
    sim.set('ic/h-sl-ft', p.altFt);
    const kts = speedForStart(start, end);
    sim.set('ic/vc-kts', kts);
    sim.set('ic/gamma-deg', start === 'intercept' ? 0 : -3);
  }
  if (!sim.runIC()) console.warn('RunIC failed');
  if (!p.ground) {
    // The weight includes fuel only after the first initialisation: redo it with the right speed.
    sim.set('ic/vc-kts', speedForStart(start, end));
    sim.set('ic/gamma-deg', start === 'intercept' ? 0 : -3);
    sim.runIC();
  }
  sim.set('propulsion/set-running', -1);
  if (!jet) {
    sim.set('fcs/mixture-cmd-norm', 1);
    sim.set('propulsion/magneto_cmd', 3);
  }

  if (p.ground) {
    // settle on the gear with the takeoff flaps set and the brakes on
    sim.set('fcs/flap-cmd-norm', jet ? 1 / 4 : 0);
    sim.set('gear/gear-cmd-norm', 1);
    sim.set('fcs/left-brake-cmd-norm', 1);
    sim.set('fcs/right-brake-cmd-norm', 1);
    sim.step(120 * 8);
    if (jet) {
      systems.setupGround(end, sim.get('inertia/weight-lbs'));
      sim.seedFcc(0, false);
    } else systems.setupGround(end);
    systems.update(0, readStateQuick(), { roll: 0, pitch: 0, yaw: 0, brake: 0, trim: 0 });
    return;
  }

  // airborne: configure, then trim
  const conf = jet ? ({ intercept: 1, ils10: 2, final4: 4 }[start]) : ({ ils10: 0, final4: 1 }[start]);
  const gearDown = start === 'final4' || !jet;
  if (jet) {
    sim.set('fcs/flap-cmd-norm', conf / 4);
    sim.set('gear/gear-cmd-norm', gearDown ? 1 : 0);
    sim.set('fcs/throttle-cmd-norm[0]', 0.5);
    sim.set('fcs/throttle-cmd-norm[1]', 0.5);
  } else {
    sim.set('fcs/flap-cmd-norm', conf / 3);
    sim.set('fcs/throttle-cmd-norm', 0.6);
  }
  if (!sim.trim(1)) console.warn('trim did not converge');
  if (jet) {
    const elev = -(sim.get('fcs/elevator-cmd-norm') + sim.get('fcs/pitch-trim-cmd-norm'));
    sim.set('fcs/pitch-trim-cmd-norm', 0);
    sim.seedFcc(elev, true);
    sim.set('athr/manual-thrust', sim.get('fcs/throttle-cmd-norm[0]'));
    const weight = sim.get('inertia/weight-lbs');
    const ils = { course: end.heading, gs: 3, end };
    if (start === 'intercept') {
      systems.setupCruise(p.heading, p.altFt, Math.round(speedForStart(start, end)));
      systems.flapsLever = 1;
      systems.gearLever = 0;
      systems.ils = ils;
      systems.latArmed = 'LOC';
      systems.vertArmed = 'G/S';
      systems.fcu.alt = Math.round((p.altFt) / 100) * 100;
      systems.fcu.spdManaged = true;
      systems.ap = true;
    } else {
      systems.setupApproach(end, weight, ils);
      systems.flapsLever = conf;
      systems.gearLever = gearDown ? 1 : 0;
      systems.ap = start === 'ils10';
      systems.autobrake = 'MED';
    }
  } else {
    systems.flapsLever = conf;
    systems.setupAirborne();
  }
}

function speedForStart(start, end) {
  if (!acDef.jet) return start === 'final4' ? 70 : 90;
  const w = sim.get('inertia/weight-lbs');
  const sp = speedsFor(w, start === 'final4' ? 4 : start === 'ils10' ? 2 : 1);
  return start === 'final4' ? sp.vapp : start === 'ils10' ? sp.f + 5 : Math.max(sp.s + 10, 200);
}

function readStateQuick() {
  readState();
  return f;
}

// ---------------------------------------------------------------------------
// Main simulation step
// ---------------------------------------------------------------------------
function simulate(dt) {
  const input = controls.read();
  if (acDef.jet) {
    if (input.throttleUp) setLever(systems.lever + dt * 0.4);
    if (input.throttleDown) setLever(systems.lever - dt * 0.4);
  } else {
    if (input.throttleUp) setLever(systems.lever + dt * 0.5);
    if (input.throttleDown) setLever(systems.lever - dt * 0.5);
  }
  // ILS deviations for guidance and display
  if (systems.ils) {
    systems.ilsData = ilsDeviation(systems.ils.end, f.lat, f.lon, f.altFt);
  }
  systems.update(dt, f, input);

  flight.simAcc += dt;
  const n = Math.min(30, Math.floor(flight.simAcc / sim.dt));
  flight.simAcc = Math.min(flight.simAcc - n * sim.dt, sim.dt);
  if (!sim.step(n) || !Number.isFinite(sim.get('position/h-sl-ft'))) return end('crash', 'The flight model failed.');
  readState();
  flight.elapsed += dt;

  sim.updatePatch(f.lat, f.lon);
  // prefetch detail elevation ~4 km ahead along the track
  if (flight.elapsed - (flight.lastPrefetch || 0) > 1 && f.aglFt < 15000) {
    flight.lastPrefetch = flight.elapsed;
    const [la, lo] = destination(f.lat, f.lon, f.track, 4000);
    ensureDetail(la - 0.02, la + 0.02, lo - 0.03, lo + 0.03);
  }
  checkEvents(dt);
  handleSystemEvents();
}

function setLever(v) {
  systems.lever = clamp(v, 0, 1);
  updateLeverUI();
}

// ---------------------------------------------------------------------------
// Events: contacts, touchdown, callouts, warnings
// ---------------------------------------------------------------------------
function checkEvents(dt) {
  for (const [unit, what] of Object.entries(acDef.contacts)) {
    if (sim.get(`contact/unit[${unit}]/WOW`) <= 0) continue;
    if (what === 'tail') {
      if (!flight.tailStrike) {
        flight.tailStrike = true;
        toast('TAIL STRIKE');
        audio.chime(1);
      }
    } else return end('crash', `${what}!`);
  }

  // touchdown
  if (!f.wowMain) {
    flight.airborneTime += dt;
    flight.stopped = false;
  } else if (flight.airborneTime > 3 && !flight.touchdown) {
    const fpm = Math.round(-flight.lastVs);
    flight.touchdown = { fpm, t: flight.elapsed, ias: f.ias, pitch: f.theta, bank: f.phi, nzMax: f.nzF, lat: f.lat, lon: f.lon, track: f.track };
    flight.airborneTime = 0;
    if (fpm > acDef.crashFpm) return end('crash', `Touchdown at ${fpm} ft/min — the landing gear collapsed.`);
  } else if (f.wowMain) flight.airborneTime = 0;
  if (flight.touchdown && !flight.touchdown.reported) {
    flight.touchdown.nzMax = Math.max(flight.touchdown.nzMax, f.nzF);
    if (flight.elapsed - flight.touchdown.t > 1.2) {
      flight.touchdown.reported = true;
      showLandingReport(flight.touchdown);
    }
  }
  if (f.onGround && !f.wowMain && !f.wowNose) {
    /* nothing */
  }
  if (flight.touchdown && f.onGround && f.gs < 1 && !flight.stopped) {
    flight.stopped = true;
    toast('Aircraft stopped');
  }
  flight.lastVs = f.vs;

  // callouts
  const ra = f.ra, last = flight.lastRa;
  const once = (key, fn) => {
    if (!flight.callouts.has(key)) {
      flight.callouts.add(key);
      fn();
    }
  };
  const crossed = (h) => last > h && ra <= h && f.vs < 0 && !f.onGround;
  if (acDef.jet) {
    if (f.onGround && !flight.touchdown) {
      if (f.ias > 100) once('100kt', () => audio.say('one hundred knots'));
      const vs = systems.vspeeds;
      if (vs && f.ias > vs.v1) once('v1', () => audio.say('V one'));
      if (vs && f.ias > vs.vr) once('vr', () => audio.say('rotate'));
    }
    if (!f.onGround && f.vs > 300 && ra > 30 && ra < 1500 && !flight.touchdown && flight.scenario.start === 'takeoff') once('posclimb', () => audio.say('positive climb'));
    for (const [h, words] of [[2500, 'two thousand five hundred'], [1000, 'one thousand'], [500, 'five hundred'], [400, 'four hundred'], [300, 'three hundred'], [100, 'one hundred'], [50, 'fifty'], [40, 'forty'], [30, 'thirty'], [10, 'ten']]) {
      if (crossed(h) && f.gearPos > 0.9) once(`ra${h}`, () => audio.say(words, h <= 50));
    }
    if (crossed(200) && f.gearPos > 0.9) once('mins', () => audio.say('minimum', true));
    if (crossed(20) && f.gearPos > 0.9) once('ra20', () => audio.say(systems.lever > 0.02 ? 'retard, retard' : 'twenty', true));
    if (!f.onGround && ra < 30 && f.theta > 9.5 && f.vs < 0) once('pitch', () => audio.say('pitch, pitch', true));
    if (!f.onGround && f.gearPos < 0.5 && ra < 500 && f.vs < -100 && f.ias < 190) once('gear', () => { audio.say('too low, gear', true); toast('TOO LOW GEAR'); });
    if (flight.touchdown && f.onGround) {
      if (f.n1[0] < 40 && systems.reverserPos > 0.99) once('revgreen', () => audio.say('reverse green'));
      if (systems.decel > 1.5) once('decel', () => audio.say('decel'));
      if (f.ias < 70 && f.ias > 60) once('70kt', () => audio.say('seventy knots'));
    }
  } else {
    for (const [h, words] of [[500, 'five hundred'], [100, 'one hundred'], [50, 'fifty'], [10, 'ten']]) if (crossed(h)) once(`ra${h}`, () => audio.say(words));
  }
  // GPWS mode 1
  if (!f.onGround && ra < 2450 && ra > 30) {
    const pull = f.vs < -(1700 + ra * 1.0), sink = f.vs < -(1200 + ra * 0.6);
    const now = performance.now();
    if ((pull || sink) && now - (flight.lastGpws || 0) > 3000) {
      flight.lastGpws = now;
      audio.say(pull ? 'pull up' : 'sink rate', true);
      toast(pull ? 'PULL UP' : 'SINK RATE');
    }
  }
  flight.lastRa = ra;
}

function handleSystemEvents() {
  const ev = systems.events;
  while (ev.length) {
    const e = ev.shift();
    if (e === 'ap-off') { audio.cavalry(); toast('AP OFF'); }
    else if (e === 'ap-on') toast('AP1 engaged');
    else if (e === 'athr-off') { audio.chime(1); toast('A/THR OFF'); }
    else if (e === 'alpha-floor') { audio.crc(); toast('A.FLOOR'); }
    else if (e === 'spoilers') audio.say('spoilers');
  }
}

function showLandingReport(td) {
  const { apt } = flight.scenario;
  // find the runway end we landed on
  let best = null;
  for (const rw of apt.runways) {
    for (const endObj of [rw.le, rw.he]) {
      if (Math.abs(angleDiff(td.track, endObj.heading)) > 40) continue;
      const { s, c } = runwayCoords(rw, td.lat, td.lon);
      if (s < -200 || s > rw.lengthM + 200 || Math.abs(c) > 150) continue;
      const dir = endObj === rw.le ? 1 : -1;
      best = { end: endObj, dist: (s - endObj.thresholdS) * dir, offset: c * dir, onRunway: s >= 0 && s <= rw.lengthM && Math.abs(c) <= rw.widthM / 2 };
    }
  }
  const rows = [];
  const grade = (ok, warn) => (ok ? 'good' : warn ? 'warn' : 'bad');
  const fpm = td.fpm;
  rows.push(['Sink rate', `${fpm} ft/min`, grade(fpm <= 360, fpm <= acDef.hardLandingFpm)]);
  rows.push(['Load factor', `${td.nzMax.toFixed(2)} g`, grade(td.nzMax < 1.6, td.nzMax < 2.1)]);
  if (acDef.jet) {
    const vapp = speedsFor(f.weight, Math.round(f.confPos)).vapp;
    const d = td.ias - (vapp - 5);
    rows.push(['Speed', `${Math.round(td.ias)} kt (${d >= 0 ? '+' : ''}${Math.round(d)})`, grade(Math.abs(d) <= 7, Math.abs(d) <= 15)]);
    rows.push(['Pitch', `${td.pitch.toFixed(1)}°`, grade(td.pitch < 7.5 && td.pitch > 0, td.pitch < 10)]);
  }
  rows.push(['Bank', `${Math.abs(td.bank).toFixed(1)}°`, grade(Math.abs(td.bank) < 3, Math.abs(td.bank) < 6)]);
  if (best) {
    rows.push(['Touchdown', `${Math.round(best.dist)} m past threshold`, grade(best.dist > 150 && best.dist < 600, best.dist > 0 && best.dist < 900)]);
    rows.push(['Centreline', `${Math.abs(best.offset).toFixed(1)} m ${best.offset > 0 ? 'right' : 'left'}`, grade(Math.abs(best.offset) < 4, Math.abs(best.offset) < 10)]);
  } else rows.push(['Runway', 'Off the runway!', 'bad']);
  const score = rows.reduce((n, r) => n + (r[2] === 'good' ? 2 : r[2] === 'warn' ? 1 : 0), 0) / (rows.length * 2);
  const bad = rows.filter((r) => r[2] === 'bad').length;
  const title = bad ? (bad > 1 ? 'Rough landing' : 'Landing — needs work') : score > 0.85 ? 'Excellent landing' : score > 0.6 ? 'Good landing' : 'Acceptable landing';
  $('report').innerHTML = `<h3>${title}${best ? ` · RWY ${best.end.ident}` : ''}</h3><table>${rows
    .map(([k, v, g]) => `<tr class="${g}"><td>${k}</td><td>${v}</td></tr>`)
    .join('')}</table><small>Tap to close</small>`;
  $('report').classList.remove('hidden');
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
const camPos = new THREE.Vector3();
const camUp = new THREE.Vector3(0, 1, 0);
const tmp = new THREE.Vector3(), tmp2 = new THREE.Vector3(), localUp = new THREE.Vector3();
const forward = new THREE.Vector3(), up = new THREE.Vector3();
let towerPos = null;

function resetCamera() {
  forward.set(0, 0, -1).applyQuaternion(f.quat);
  const [dist, h] = acDef.chase;
  upAt(f.lat, f.lon, localUp);
  camPos.copy(f.pos).addScaledVector(forward, -dist).addScaledVector(localUp, h);
  camUp.copy(localUp);
  towerPos = null;
}

function updateCamera(dt) {
  forward.set(0, 0, -1).applyQuaternion(f.quat);
  up.set(0, 1, 0).applyQuaternion(f.quat);
  upAt(f.lat, f.lon, localUp);
  $('cockpitFrame').classList.toggle('hidden', flight.camMode !== 'cockpit' || !acDef.jet);
  model.group.visible = flight.camMode !== 'cockpit';
  if (flight.camMode === 'cockpit') {
    camera.position.copy(model.eyepoint).applyQuaternion(f.quat).add(f.pos);
    camera.quaternion.copy(f.quat);
    camera.rotateX(acDef.jet ? -4 * DEG : 0);
    camera.fov = 62;
  } else if (flight.camMode === 'tower') {
    if (!towerPos || towerPos.distanceTo(f.pos) > 12000) {
      // a spot beside the nearest runway end, ~15 m up
      const end = flight.scenario.end, rw = end.runway;
      const s = rw.lengthM / 2;
      const e = s * rw.ux + (rw.widthM / 2 + 220) * rw.uy, n = s * rw.uy - (rw.widthM / 2 + 220) * rw.ux;
      const lat = rw.le.lat + n / rw.mLat, lon = rw.le.lon + e / rw.mLon;
      towerPos = toScene(lat, lon, Math.max(rw.profile(s), elevationAt(lat, lon)) + 18);
      if (towerPos.distanceTo(f.pos) > 12000) {
        towerPos = f.pos.clone().addScaledVector(forward, 900).addScaledVector(localUp, -20);
      }
    }
    camera.position.copy(towerPos);
    camera.up.copy(localUp);
    camera.lookAt(f.pos);
    const d = towerPos.distanceTo(f.pos);
    camera.fov = clamp((2 * Math.atan((model.length * 1.6) / d)) / DEG, 2, 60);
  } else {
    const [dist, h] = acDef.chase;
    const target = tmp.copy(f.pos).addScaledVector(forward, -dist).addScaledVector(localUp, h).addScaledVector(up, h * 0.3);
    camPos.lerp(target, 1 - Math.exp(-4 * dt));
    // keep the chase camera above the ground
    const cp = fromScene(camPos), ground = elevationAt(cp.lat, cp.lon) + 3;
    if (cp.h < ground) camPos.addScaledVector(localUp, ground - cp.h);
    camUp.lerp(tmp2.copy(up).add(localUp).add(localUp).normalize(), 1 - Math.exp(-3 * dt)).normalize();
    camera.position.copy(camPos);
    camera.up.copy(camUp);
    camera.lookAt(tmp.copy(f.pos).addScaledVector(forward, dist * 0.4));
    camera.fov = 55;
  }
  camera.updateProjectionMatrix();
}

function cycleCamera() {
  const modes = ['chase', 'cockpit', 'tower'];
  flight.camMode = modes[(modes.indexOf(flight.camMode) + 1) % modes.length];
  resetCamera();
  toast(flight.camMode === 'chase' ? 'Chase view' : flight.camMode === 'cockpit' ? 'Cockpit view' : 'Tower view');
}

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------
const lastText = new Map();
function setText(el, text) {
  if (lastText.get(el) !== text) {
    el.textContent = text;
    lastText.set(el, text);
  }
}
let toastTimer = 0;
function toast(text) {
  const el = $('toast');
  el.textContent = text;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 3500);
}

function updateHud() {
  const jet = acDef.jet;
  const s = systems;
  let ils = null;
  if (s.ils && s.ilsData) {
    const d = s.ilsData;
    ils = { ident: `ILS${s.ils.end.ident}`, dme: d.distM / NM, locDots: d.locDots, gsDots: d.gsDots, valid: d.valid, gsValid: d.gsValid, course: s.ils.course };
  } else if (flight.scenario?.end) {
    // display the runway's localizer when a runway is selected
    const end = flight.scenario.end;
    const d = ilsDeviation(end, f.lat, f.lon, f.altFt);
    if (d.valid && d.distM < 30 * NM) ils = { ident: `ILS${end.ident}`, dme: d.distM / NM, locDots: d.locDots, gsDots: d.gsDots, valid: d.valid, gsValid: d.gsValid, course: end.heading };
  }
  const data = {
    pitch: f.theta, roll: f.phi, ias: f.ias, accel: jet ? s.accel : 0, alt: f.altFt, vs: f.vs, hdg: f.heading, track: f.track, fpa: f.fpa,
    driftDeg: angleDiff(f.track, f.heading), beta: f.beta, ra: f.ra, mach: f.mach, onGround: f.onGround, conf: f.confPos, ils, showFpv: settings.fpv && !f.onGround,
    minimums: jet ? 200 : 0,
  };
  if (jet) {
    data.speeds = s.speeds;
    data.speedTarget = s.speedTarget;
    data.spdManaged = s.fcu.spdManaged;
    data.altSel = s.fcu.alt;
    data.hdgSel = s.fcu.hdg;
    data.vspeeds = s.vspeeds;
    const lvr = !f.onGround && s.lever > DETENTS.CL + 0.02 && f.ra > 1400 && !s.takeoffPhase ? 'LVR CLB' : '';
    const vertDisplay = s.vert === 'G/S' && f.ra < 50 ? 'FLARE' : s.vert;
    data.fma = {
      athr: s.athrMode, lvr, vert: vertDisplay, vertArmed: s.vertArmed, lat: s.lat, latArmed: s.latArmed, cat: s.ils ? 'CAT1' : '',
      ap: s.ap, fd: s.fd, athrState: s.athrActive || s.athrArmed ? 'A/THR' : '', athrActive: s.athrActive,
      athrSince: s.modeChanged.athrMode, vertSince: s.modeChanged.vert, latSince: s.modeChanged.lat,
    };
    if (s.fd && !s.ap && !f.onGround) data.fdBars = { roll: (s.phiCmd - f.phi) / 25, pitch: -(s.fpaCmd - f.fpa) / 6 };
    else if (s.fd && f.onGround && s.vert === 'SRS') data.fdBars = { roll: 0, pitch: -(12 - f.theta) / 10 };
    setText($('ecam'), `N1 ${f.n1[0].toFixed(1)} ${f.n1[1].toFixed(1)} · FLAPS ${FLAP_LABELS[s.flapsLever]}${Math.abs(f.confPos - s.flapsLever) > 0.05 ? '…' : ''} · GEAR ${f.gearPos > 0.99 ? 'DN' : f.gearPos < 0.01 ? 'UP' : '⇅'} · SPD BRK ${s.speedbrakeLabel()} · A/BRK ${s.autobrake}${s.reverserPos > 0.99 ? ' · REV' : ''}`);
  } else {
    const trim = s.trim < -0.01 ? `▲${Math.round(-s.trim * 100)}` : s.trim > 0.01 ? `▼${Math.round(s.trim * 100)}` : '0';
    setText($('ecam'), `RPM ${Math.round(f.rpm / 10) * 10} · FLAPS ${C172_FLAP_LABELS[s.flapsLever]} · TRIM ${trim}`);
  }
  pfd.draw(data);
  setText($('flapVal'), acDef.flapLabels[s.flapsLever]);

  let warn = '';
  if (jet && f.ias > (s.speeds?.vmax ?? 999) + 4 && !f.onGround) warn = 'OVERSPEED';
  if (!jet && !f.onGround && f.alpha > 15) warn = 'STALL';
  setText($('warn'), warn);
  if (warn === 'OVERSPEED' && performance.now() - (flight.lastWarnSound || 0) > 2500) {
    flight.lastWarnSound = performance.now();
    audio.crc();
  }
  if (jet) updateFcuUI();
  audio.update({ jet, n1: Math.max(f.n1[0], f.n1[1]), rpm: f.rpm, ias: f.ias, rolling: f.onGround ? f.gs : 0, cockpit: flight.camMode === 'cockpit' });
}

// ---------------------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------------------
const leverEl = $('thrust');
controls.bindLever(leverEl, (v) => setLever(v), Object.values(DETENTS));
function updateLeverUI() {
  if (!systems) return;
  const v = systems.lever;
  $('thrFill').style.height = `${v * 100}%`;
  if (acDef.jet) {
    const name = Object.entries(DETENTS).find(([, d]) => Math.abs(d - v) < 0.01)?.[0];
    setText($('thrLabel'), systems.reverse ? 'REV' : name ? (name === 'FLX' ? 'FLX/MCT' : name) : `${Math.round(v * 100)}%`);
  } else setText($('thrLabel'), `${Math.round(v * 100)}%`);
}

function bindButton(id, fn) {
  $(id).addEventListener('click', (e) => {
    e.preventDefault();
    if (flight.mode === 'flying') fn();
  });
}
bindButton('gearBtn', () => acDef.jet && systems.toggleGear(f));
bindButton('flapUp', () => systems.setFlaps(systems.flapsLever - 1));
bindButton('flapDn', () => systems.setFlaps(systems.flapsLever + 1));
bindButton('sbBtn', () => acDef.jet && systems.cycleSpeedbrake());
bindButton('abBtn', () => acDef.jet && systems.cycleAutobrake());
bindButton('revBtn', () => acDef.jet && systems.toggleReverse(f));
controls.bindHold($('brakeBtn'), () => (controls.brakeHeld = true), () => (controls.brakeHeld = false));
controls.bindHold($('trimUp'), () => (controls.trimDir = -1), () => (controls.trimDir = 0));
controls.bindHold($('trimDn'), () => (controls.trimDir = 1), () => (controls.trimDir = 0));
bindButton('camBtn', cycleCamera);
$('pauseBtn').addEventListener('click', () => togglePause());
$('fcuBtn').addEventListener('click', () => $('fcu').classList.toggle('hidden'));
$('report').addEventListener('click', () => $('report').classList.add('hidden'));

// FCU
const fcuAct = {
  spdDn: () => systems.adjustSpeed(-5), spdUp: () => systems.adjustSpeed(5), spdMng: () => systems.managedSpeed(),
  hdgDn: () => systems.adjustHeading(-5, f), hdgUp: () => systems.adjustHeading(5, f), hdgPull: () => systems.pullHeading(f),
  altDn: () => systems.adjustAltitude(-500), altUp: () => systems.adjustAltitude(500), lvlch: () => systems.pressLvlCh(f),
  vsDn: () => systems.adjustVs(-100, f), vsUp: () => systems.adjustVs(100, f),
  apBtn: () => systems.toggleAP(f), athrBtn: () => systems.toggleATHR(), locBtn: () => { systems.ils ||= ilsFor(); systems.pressLOC(); }, apprBtn: () => { systems.ils ||= ilsFor(); systems.pressAPPR(); },
};
function ilsFor() {
  const end = flight.scenario.end;
  return { course: end.heading, gs: 3, end };
}
for (const [id, fn] of Object.entries(fcuAct)) {
  const el = $(id);
  let timer = null;
  const repeatable = /Up$|Dn$/.test(id);
  el.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (flight.mode !== 'flying' || !acDef.jet) return;
    fn();
    if (repeatable) {
      timer = setTimeout(function rep() {
        fn();
        timer = setTimeout(rep, 90);
      }, 450);
    }
  });
  const stop = () => clearTimeout(timer);
  el.addEventListener('pointerup', stop);
  el.addEventListener('pointercancel', stop);
  el.addEventListener('pointerleave', stop);
}
function updateFcuUI() {
  const s = systems;
  setText($('spdVal'), s.fcu.spdManaged ? '---' : String(s.fcu.spd));
  setText($('hdgVal'), String(s.fcu.hdg).padStart(3, '0'));
  setText($('altVal'), String(s.fcu.alt));
  setText($('vsVal'), s.vert === 'V/S' ? `${s.fcu.vs > 0 ? '+' : ''}${s.fcu.vs}` : '-----');
  $('apBtn').classList.toggle('on', s.ap);
  $('athrBtn').classList.toggle('on', s.athrArmed || s.athrActive);
  $('locBtn').classList.toggle('on', s.latArmed === 'LOC' || s.lat.startsWith('LOC'));
  $('apprBtn').classList.toggle('on', s.vertArmed === 'G/S' || s.vert.startsWith('G/S'));
  setText($('gearBtn').querySelector('b'), s.gearLever ? 'DOWN' : 'UP');
  setText($('sbBtn').querySelector('b'), s.speedbrakeLabel());
  setText($('abBtn').querySelector('b'), s.autobrake);
  $('revBtn').classList.toggle('on', s.reverse);
}

controls.onKey = (code) => {
  if (code === 'Escape' || code === 'KeyP') togglePause();
  if (flight.mode !== 'flying') return;
  if (code === 'KeyC') cycleCamera();
  if (code === 'KeyF') systems.setFlaps(systems.flapsLever + 1);
  if (code === 'KeyV') systems.setFlaps(systems.flapsLever - 1);
  if (code === 'KeyG' && acDef.jet) systems.toggleGear(f);
  if (code === 'KeyR' && acDef.jet) systems.toggleReverse(f);
  if (code === 'KeyX' && acDef.jet) systems.toggleAP(f);
  if (code === 'Digit1') setLever(0);
  if (code === 'Digit2') setLever(acDef.jet ? DETENTS.CL : 0.5);
  if (code === 'Digit3') setLever(acDef.jet ? DETENTS.FLX : 0.75);
  if (code === 'Digit4') setLever(1);
};

// ---------------------------------------------------------------------------
// Menu
// ---------------------------------------------------------------------------
function showMenu(msg, canResume = false) {
  $('menuMsg').textContent = msg || '';
  $('menuMsg').classList.toggle('hidden', !msg);
  $('resumeBtn').classList.toggle('hidden', !canResume);
  $('menu').classList.remove('hidden');
  document.body.classList.remove('flying');
  audio.mute();
}

function togglePause() {
  if (flight.mode === 'flying') {
    flight.mode = 'paused';
    showMenu('Paused', true);
  } else if (flight.mode === 'paused') enterFlight();
}

async function enterFlight() {
  try {
    if (!document.fullscreenElement && document.documentElement.requestFullscreen) await document.documentElement.requestFullscreen({ navigationUI: 'hide' });
    await screen.orientation?.lock?.('landscape');
  } catch {}
  if (settings.tilt && typeof DeviceMotionEvent?.requestPermission === 'function') {
    try { await DeviceMotionEvent.requestPermission(); } catch {}
  }
  controls.tilt.enabled = settings.tilt;
  controls.tilt.cal = null;
  controls.invert = settings.invert;
  audio.setEnabled(settings.sound);
  audio.start();
  $('menu').classList.add('hidden');
  $('hud').classList.remove('hidden');
  document.body.classList.add('flying');
  document.body.classList.toggle('jet', acDef.jet);
  pfd.resize();
  controls.defaultStick();
  updateLeverUI();
  flight.simAcc = 0;
  flight.mode = 'flying';
}

function end(kind, msg) {
  flight.mode = 'ended';
  if (kind === 'crash') {
    audio.crash();
    const el = document.createElement('div');
    el.className = 'flash';
    document.body.appendChild(el);
    setTimeout(() => el.remove(), 700);
  }
  setTimeout(() => showMenu(msg), 900);
}

function renderMenu() {
  const acSel = $('acSel'), startSel = $('startSel');
  acSel.querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.v === settings.aircraft));
  const ac = AIRCRAFT[settings.aircraft];
  startSel.innerHTML = ac.starts.map((s) => `<button data-v="${s}" class="${s === settings.start ? 'on' : ''}"><b>${START_LABELS[s][0]}</b><small>${START_LABELS[s][1]}</small></button>`).join('');
  if (!ac.starts.includes(settings.start)) {
    settings.start = ac.starts[0];
    startSel.firstElementChild?.classList.add('on');
  }
  const apt = findAirport(settings.airport);
  $('aptName').textContent = apt ? `${apt.ident}${apt.iata ? ' / ' + apt.iata : ''} — ${apt.name}${apt.city ? ', ' + apt.city : ''}` : 'Airport not found';
  const rwySel = $('rwySel');
  if (apt) {
    const ends = runwayEnds(apt);
    if (!ends.some((e) => e.ident === settings.runway)) settings.runway = ends[0]?.ident || '';
    rwySel.innerHTML = ends.map((e) => `<option value="${e.ident}" ${e.ident === settings.runway ? 'selected' : ''}>${e.ident} · ${Math.round(e.runway.lengthM)} m · ${String(Math.round(e.heading)).padStart(3, '0')}°</option>`).join('');
  }
  $('flyBtn').disabled = !apt;
}

function bindMenu() {
  $('acSel').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    settings.aircraft = b.dataset.v;
    saveSettings();
    renderMenu();
  });
  $('startSel').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    settings.start = b.dataset.v;
    saveSettings();
    renderMenu();
  });
  const input = $('aptSearch'), results = $('aptResults');
  input.value = settings.airport;
  input.addEventListener('input', () => {
    const list = searchAirports(input.value, 8);
    results.innerHTML = list.map((a) => `<button data-id="${a.ident}"><b>${a.ident}</b> ${a.name}<small>${a.city || ''} ${a.country}</small></button>`).join('');
    results.classList.toggle('hidden', !list.length || !input.value.trim());
  });
  results.addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    settings.airport = b.dataset.id;
    settings.runway = '';
    input.value = b.dataset.id;
    results.classList.add('hidden');
    saveSettings();
    renderMenu();
  });
  $('rwySel').addEventListener('change', (e) => { settings.runway = e.target.value; saveSettings(); });
  for (const [id, key] of [['timeSel', 'time'], ['visSel', 'vis'], ['windSel', 'wind']]) {
    $(id).value = settings[key];
    $(id).addEventListener('change', (e) => { settings[key] = e.target.value; saveSettings(); });
  }
  for (const [id, key] of [['optSound', 'sound'], ['optInvert', 'invert'], ['optTilt', 'tilt'], ['optFpv', 'fpv']]) {
    $(id).checked = settings[key];
    $(id).addEventListener('change', (e) => { settings[key] = e.target.checked; saveSettings(); });
  }
  if (window.isSecureContext && 'DeviceMotionEvent' in window) $('tiltOpt').classList.remove('hidden');
  $('flyBtn').addEventListener('click', () => startFlight());
  $('resumeBtn').addEventListener('click', () => enterFlight());
}

// Block page gestures (pinch zoom, pull to refresh, long-press menu).
document.addEventListener('touchmove', (e) => { if (!e.target.closest('.card')) e.preventDefault(); }, { passive: false });
document.addEventListener('gesturestart', (e) => e.preventDefault());
document.addEventListener('contextmenu', (e) => e.preventDefault());
document.addEventListener('visibilitychange', () => document.hidden && flight.mode === 'flying' && togglePause());

function onResize() {
  const w = window.innerWidth, h = window.innerHeight;
  camera.aspect = w / h;
  if (w > h) camera.setViewOffset(w, h, 0, h * 0.12, w, h);
  else camera.clearViewOffset();
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
  pfd.resize();
  controls.defaultStick?.();
}
addEventListener('resize', onResize);
onResize();

// ---------------------------------------------------------------------------
// Boot and loop
// ---------------------------------------------------------------------------
setOrigin(48.3538, 11.7861);
camera.position.set(0, 3000, 0);
camera.lookAt(0, 0, -30000);

loadAirports()
  .then(() => {
    bindMenu();
    renderMenu();
    $('loadMsg').classList.add('hidden');
    $('flyBtn').classList.remove('hidden');
    window.trainer = { flight, f, settings, startFlight, simulate, setLever, get sim() { return sim; }, get systems() { return systems; }, get terrain() { return terrain; }, controls };
  })
  .catch((err) => {
    $('loadMsg').textContent = `Could not load the airport database: ${err.message}`;
  });

let lastTime = performance.now();
renderer.setAnimationLoop((now) => {
  const dt = Math.min((now - lastTime) / 1000, 0.1);
  lastTime = now;
  if (flight.mode === 'flying') simulate(dt);
  if (sim && model && flight.mode !== 'loading') {
    model.group.position.copy(f.pos);
    model.group.quaternion.copy(f.quat);
    model.update({ gearPos: f.gearPos, flapDeg: f.flapDeg, spoiler: f.spoiler, speedbrake: f.speedbrake, onGround: f.onGround, compression: f.compression, rpm: flight.mode === 'flying' ? f.rpm : 0 }, dt);
    if (flight.mode === 'flying' || flight.mode === 'paused' || flight.mode === 'ended') {
      if (flight.mode === 'flying') updateCamera(dt);
      terrain.update(camera.position);
      runways.lightScale = atmosphere.lightScale;
      runways.update(camera.position, now);
      atmosphere.update(f.pos);
      if (flight.mode === 'flying') updateHud();
    }
  } else if (flight.mode === 'menu') {
    camera.position.set(0, 3000, 0);
  }
  renderer.render(scene, camera);
});
