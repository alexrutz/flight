// A320 systems: thrust levers, autothrust, autopilot / flight director outer
// loops and mode logic, high lift, gear, spoilers, autobrake, steering and
// characteristic speeds. The inner fly-by-wire loops run in C++ (wasm/fcc.h).
import { angleDiff, DEG } from '../geo.js';

const KT = 1.943844;
// Must match wasm/aircraft-src/gen_a320.py and wasm/fcc.h
const CL0 = [0.22, 0.55, 0.8, 0.95, 1.25];
const CLA = [0.087, 0.09, 0.093, 0.094, 0.096];
const CLMAX = [1.5, 2.15, 2.45, 2.6, 2.8];
const ALPHA_PROT = [11.0, 14.0, 14.0, 13.8, 12.5];
const ALPHA_MAX = [13.5, 16.5, 16.3, 16.0, 15.0];
const S_WING = 122.6;
export const FLAP_LABELS = ['0', '1+F', '2', '3', 'FULL'];
const VFE = [null, 215, 200, 185, 177];
const VMO = 350, MMO = 0.82, VLE = 280;

// Thrust lever detents (lever position 0..1)
export const DETENTS = { IDLE: 0, CL: 0.6, FLX: 0.8, TOGA: 1 };
const CL_THRUST = 0.88;
export function leverToThrust(l) {
  return l <= DETENTS.CL ? (l / DETENTS.CL) * CL_THRUST : CL_THRUST + ((l - DETENTS.CL) / (1 - DETENTS.CL)) * (1 - CL_THRUST);
}

const lerpConf = (t, c) => {
  const i = Math.min(3, Math.max(0, Math.floor(c))), f = Math.min(1, Math.max(0, c - i));
  return t[i] + (t[i + 1] - t[i]) * f;
};

export function speedsFor(weightLbs, conf, mach = 0, ias = 0) {
  const W = weightLbs * 0.45359237 * 9.80665;
  const v = (cl) => Math.sqrt((2 * W) / (1.225 * S_WING * cl)) * KT;
  const vs = (c) => v(CLMAX[c]);
  const ci = Math.round(conf);
  const vls = vs(ci) * (ci === 0 ? 1.28 : 1.23);
  const aProt = lerpConf(ALPHA_PROT, conf), aMax = lerpConf(ALPHA_MAX, conf);
  const cl0 = lerpConf(CL0, conf), cla = lerpConf(CLA, conf);
  let vmax = VMO;
  if (mach > 0.3 && ias > 0) vmax = Math.min(vmax, (ias * MMO) / mach);
  if (VFE[ci]) vmax = Math.min(vmax, VFE[ci]);
  const tonnes = weightLbs * 0.00045359237;
  return {
    vls,
    vAlphaProt: v(cl0 + cla * aProt),
    vAlphaMax: v(cl0 + cla * aMax),
    vmax,
    greenDot: 2 * tonnes + 85,
    s: vs(1) * 1.23,
    f: vs(2) * 1.22,
    vapp: vs(4) * 1.23 + 5,
    v2: vs(1) * 1.13,
    vr: vs(1) * 1.13 - 4,
    v1: vs(1) * 1.13 - 7,
    vle: VLE,
    alphaProt: aProt,
    alphaMax: aMax,
    alphaFloor: aProt + 0.6 * (aMax - aProt),
  };
}

export class A320Systems {
  constructor(sim) {
    this.sim = sim;
    this.lever = 0;
    this.reverse = false;
    this.reverserPos = 0;
    this.flapsLever = 0;
    this.gearLever = 1;
    this.speedbrake = 0; // 0, 0.5, 1
    this.groundSpoilersArmed = false;
    this.autobrake = 'OFF'; // OFF | LO | MED
    this.autobrakeActive = false;
    this.brakePedal = 0;
    this.fcu = { spd: 250, spdManaged: true, hdg: 0, alt: 5000, vs: 0 };
    this.ap = false;
    this.athrArmed = false;
    this.athrActive = false;
    this.alphaFloor = false;
    this.fd = true;
    this.lat = 'HDG';
    this.latArmed = '';
    this.vert = 'V/S';
    this.vertArmed = '';
    this.athrMode = '';
    this.phiCmd = 0;
    this.fpaCmd = 0;
    this.messages = [];
    this.events = [];
    this.modeChanged = {};
    this.ils = null;
    this.takeoffPhase = false;
    this.accelAltFt = 1500;
    this.captureVs = 0;
    this.lastIas = 0;
    this.accel = 0;
    this.decel = 0;
    this.lastGs = 0;
    this.srsSpeed = 160;
  }

  // ------------------------------------------------------------- setup
  setupGround(end, weight) {
    const sp = speedsFor(weight, 1);
    this.lever = 0;
    this.flapsLever = 1;
    this.gearLever = 1;
    this.speedbrake = 0;
    this.groundSpoilersArmed = true;
    this.autobrake = 'OFF';
    this.ap = false;
    this.athrArmed = true;
    this.fd = true;
    this.lat = 'RWY';
    this.latArmed = 'NAV';
    this.vert = 'SRS';
    this.vertArmed = 'CLB';
    this.takeoffPhase = true;
    this.fcu = { spd: Math.round(sp.v2), spdManaged: true, hdg: Math.round(end.heading), alt: 5000, vs: 0 };
    this.srsSpeed = sp.v2 + 10;
    this.vspeeds = { v1: Math.round(sp.v1), vr: Math.round(sp.vr), v2: Math.round(sp.v2) };
  }

  setupApproach(end, weight, ils) {
    const sp = speedsFor(weight, 4);
    this.lever = DETENTS.CL;
    this.flapsLever = 4;
    this.gearLever = 1;
    this.speedbrake = 0;
    this.groundSpoilersArmed = true;
    this.autobrake = 'MED';
    this.ap = false;
    this.athrArmed = true;
    this.athrActive = true;
    this.fd = true;
    this.ils = ils;
    this.lat = 'LOC';
    this.latArmed = '';
    this.vert = 'G/S';
    this.vertArmed = '';
    this.takeoffPhase = false;
    this.fcu = { spd: Math.round(sp.vapp), spdManaged: true, hdg: Math.round(end.heading), alt: 3000, vs: 0 };
    this.vspeeds = null;
  }

  setupCruise(heading, altFt, kts) {
    this.lever = DETENTS.CL;
    this.flapsLever = 0;
    this.gearLever = 0;
    this.groundSpoilersArmed = false;
    this.autobrake = 'OFF';
    this.athrArmed = true;
    this.athrActive = true;
    this.fd = true;
    this.lat = 'HDG';
    this.vert = 'ALT';
    this.latArmed = this.vertArmed = '';
    this.takeoffPhase = false;
    this.fcu = { spd: kts, spdManaged: false, hdg: Math.round(heading), alt: Math.round(altFt / 100) * 100, vs: 0 };
    this.vspeeds = null;
  }

  // ------------------------------------------------------------- pilot actions
  say(text) {
    this.events.push(text);
  }
  setMode(kind, mode) {
    if (this[kind] !== mode) {
      if (mode.startsWith('OP')) this.opInt = null;
      this[kind] = mode;
      this.modeChanged[kind] = performance.now();
    }
  }
  toggleAP(f) {
    if (this.ap) {
      this.ap = false;
      this.say('ap-off');
      return;
    }
    if (f.onGround || f.ra < 100) return;
    this.ap = true;
    this.fd = true;
    if (this.lat === 'RWY') this.setMode('lat', 'HDG');
    this.say('ap-on');
  }
  toggleATHR() {
    if (this.athrArmed || this.athrActive) {
      this.athrArmed = this.athrActive = false;
      this.alphaFloor = false;
      this.say('athr-off');
    } else {
      this.athrArmed = true;
    }
  }
  pressLOC() {
    this.latArmed = this.latArmed === 'LOC' && this.vertArmed !== 'G/S' ? '' : 'LOC';
    if (this.vertArmed === 'G/S' && this.latArmed !== 'LOC') this.vertArmed = '';
  }
  pressAPPR() {
    if (this.vertArmed === 'G/S' || this.vert === 'G/S') {
      this.vertArmed = '';
      if (this.latArmed === 'LOC') this.latArmed = '';
    } else {
      if (this.lat !== 'LOC') this.latArmed = 'LOC';
      this.vertArmed = 'G/S';
    }
  }
  pressLvlCh(f) {
    if (f.onGround) return;
    this.setMode('vert', this.fcu.alt > f.altFt ? 'OP CLB' : 'OP DES');
  }
  adjustSpeed(d) {
    if (this.fcu.spdManaged) this.fcu.spd = Math.round(this.speedTarget || this.fcu.spd);
    this.fcu.spdManaged = false;
    this.fcu.spd = Math.max(100, Math.min(350, this.fcu.spd + d));
  }
  managedSpeed() {
    this.fcu.spdManaged = true;
  }
  adjustHeading(d, f) {
    this.fcu.hdg = (((Math.round(this.fcu.hdg / Math.abs(d)) * Math.abs(d) + d) % 360) + 360) % 360;
    if (!f.onGround && this.lat !== 'HDG' && this.lat !== 'LOC*' && !this.ap) this.setMode('lat', 'HDG');
    if (!f.onGround && (this.lat === 'LOC' || this.lat === 'LOC*' || this.lat === 'RWY')) {
      /* keep LOC until the pilot deliberately pulls HDG */
    }
  }
  pullHeading(f) {
    if (f.onGround) return;
    this.setMode('lat', 'HDG');
    this.latArmed = '';
  }
  adjustAltitude(d) {
    this.fcu.alt = Math.max(0, Math.min(39000, Math.round((this.fcu.alt + d) / 100) * 100));
  }
  adjustVs(d, f) {
    if (this.vert !== 'V/S') this.fcu.vs = Math.round(f.vs / 100) * 100;
    this.fcu.vs = Math.max(-6000, Math.min(6000, this.fcu.vs + d));
    if (!f.onGround) this.setMode('vert', 'V/S');
  }
  cycleSpeedbrake() {
    // RET -> ARM (ground spoilers) -> 1/2 -> FULL -> RET
    if (this.speedbrake === 0 && !this.groundSpoilersArmed) this.groundSpoilersArmed = true;
    else if (this.speedbrake === 0) { this.groundSpoilersArmed = false; this.speedbrake = 0.5; }
    else if (this.speedbrake === 0.5) this.speedbrake = 1;
    else this.speedbrake = 0;
  }
  speedbrakeLabel() {
    return this.speedbrake === 1 ? 'FULL' : this.speedbrake === 0.5 ? '1/2' : this.groundSpoilersArmed ? 'ARM' : 'RET';
  }
  cycleAutobrake() {
    this.autobrake = this.autobrake === 'OFF' ? 'LO' : this.autobrake === 'LO' ? 'MED' : 'OFF';
    this.autobrakeActive = false;
  }
  toggleReverse(f) {
    if (!f.onGround || this.lever > 0.05) return;
    this.reverse = !this.reverse;
  }
  setFlaps(lever) {
    this.flapsLever = Math.max(0, Math.min(4, lever));
  }
  toggleGear(f) {
    if (this.gearLever && f.onGround) return;
    this.gearLever = this.gearLever ? 0 : 1;
  }

  // ------------------------------------------------------------- per frame
  update(dt, f, input) {
    const sim = this.sim;
    const sp = speedsFor(f.weight, f.confPos, f.mach, f.ias);
    this.speeds = sp;
    this.accel += ((f.ias - this.lastIas) / Math.max(dt, 1e-3) - this.accel) * Math.min(1, dt / 0.6);
    this.lastIas = f.ias;
    const gsMs = f.gs / KT;
    this.decel += ((this.lastGs - gsMs) / Math.max(dt, 1e-3) - this.decel) * Math.min(1, dt / 0.4);
    this.lastGs = gsMs;

    // stick inputs; strong stick input disconnects the autopilot
    if (this.ap && (Math.abs(input.pitch) > 0.35 || Math.abs(input.roll) > 0.35)) {
      this.ap = false;
      this.say('ap-off');
    }
    if (this.ap && f.ra < 160 && !f.onGround && (this.vert === 'G/S' || this.vert === 'G/S*')) {
      this.ap = false;
      this.say('ap-off');
    }
    sim.set('fbw/stick-pitch', input.pitch);
    sim.set('fbw/stick-roll', f.onGround ? 0 : input.roll);

    // ground steering: tiller at low speed, pedals at speed
    const tiller = f.gs < 20 ? 1 : f.gs > 80 ? 6 / 75 : 1 - ((f.gs - 20) / 60) * (1 - 6 / 75);
    sim.set('fcs/steer-cmd-norm', f.onGround ? input.roll * tiller + input.yaw * 0.08 : 0);
    sim.set('fbw/pedals', f.onGround ? input.roll + input.yaw : input.yaw);

    this.updateModes(dt, f, sp);
    this.updateThrust(dt, f, sp);
    this.updateHighLift(f);
    this.updateGroundSystems(dt, f, input);

    // autopilot inner-loop commands
    sim.set('fbw/ap-engaged', this.ap ? 1 : 0);
    sim.set('fbw/ap-phi-cmd-deg', this.phiCmd);
    sim.set('fbw/ap-fpa-cmd-deg', this.fpaCmd);
  }

  // Lateral and vertical guidance (also drives the flight director).
  updateModes(dt, f, sp) {
    const fcu = this.fcu;
    // takeoff -> climb transition
    if (this.takeoffPhase && !f.onGround && f.ra > this.accelAltFt) {
      this.takeoffPhase = false;
      this.setMode('vert', fcu.alt > f.altFt + 100 ? 'OP CLB' : 'V/S');
      this.vertArmed = '';
      if (this.lat === 'RWY') this.setMode('lat', 'HDG');
      this.latArmed = '';
    }
    if (this.lat === 'RWY' && !f.onGround && f.ra > 30) this.setMode('lat', 'RWY TRK');
    if (this.lat === 'RWY TRK' && f.ra > 30) {
      // hold the runway track until a heading is selected
    }

    // ILS capture
    const ils = this.ils && this.ilsData;
    if (this.latArmed === 'LOC' && ils?.valid && Math.abs(ils.locDots) < 1.6 && Math.abs(angleDiff(f.track, this.ils.course)) < 100) {
      this.setMode('lat', 'LOC*');
      this.latArmed = '';
    }
    if (this.lat === 'LOC*' && ils && Math.abs(ils.locDots) < 0.25 && Math.abs(angleDiff(f.track, this.ils.course)) < 6) this.setMode('lat', 'LOC');
    if (this.vertArmed === 'G/S' && ils?.gsValid && (this.lat === 'LOC' || this.lat === 'LOC*') && Math.abs(ils.gsDots) < 0.8) {
      this.setMode('vert', 'G/S*');
      this.vertArmed = '';
    }
    if (this.vert === 'G/S*' && ils && Math.abs(ils.gsDots) < 0.2) this.setMode('vert', 'G/S');

    // altitude capture from climbs/descents
    const altErr = fcu.alt - f.altFt;
    if (['OP CLB', 'OP DES', 'V/S', 'CLB'].includes(this.vert) && !this.takeoffPhase) {
      const window = Math.max(150, Math.abs(f.vs) / 6);
      const towards = (altErr > 0 && f.vs > 100) || (altErr < 0 && f.vs < -100);
      if (towards && Math.abs(altErr) < window) {
        this.captureVs = f.vs;
        this.setMode('vert', 'ALT*');
      }
      if (this.vert === 'OP CLB' && altErr < -50) this.setMode('vert', 'OP DES');
      if (this.vert === 'OP DES' && altErr > 50) this.setMode('vert', 'OP CLB');
    }
    if (this.vert === 'ALT*' && Math.abs(altErr) < 25 && Math.abs(f.vs) < 250) this.setMode('vert', 'ALT');

    // ------------------------------------------- lateral command (bank)
    let phi = 0;
    if (this.lat === 'HDG') {
      phi = clamp(angleDiff(fcu.hdg, f.heading) * 1.6, -25, 25);
    } else if (this.lat === 'RWY TRK') {
      phi = clamp(angleDiff(fcu.hdg, f.track) * 1.6, -15, 15);
    } else if ((this.lat === 'LOC' || this.lat === 'LOC*') && ils) {
      // fly towards the extended centreline: intercept angle proportional to the offset
      const intercept = clamp(-ils.cross * (this.lat === 'LOC' ? 0.06 : 0.045), -30, 30);
      const desiredTrack = this.ils.course + intercept;
      phi = clamp(angleDiff(desiredTrack, f.track) * 2.2, -25, 25);
    }
    this.phiCmd = phi;

    // ------------------------------------------- vertical command (flight path angle)
    const tasFpm = Math.max(80, f.tas) * 101.27;
    const vsToFpa = (vs) => Math.asin(clamp(vs / tasFpm, -0.5, 0.5)) / DEG;
    let fpa = f.fpa;
    const spdTarget = this.speedTarget ?? f.ias;
    switch (this.vert) {
      case 'SRS': {
        const tgt = this.srsSpeed;
        fpa = clamp(f.fpa + (f.ias - tgt) * 0.08, 3, 16);
        if (f.onGround) fpa = 12;
        break;
      }
      case 'OP CLB':
      case 'OP DES': {
        // pitch on speed: faster than the target -> steeper climb / shallower descent
        const [lo, hi] = this.vert === 'OP CLB' ? [0.5, 14] : [-7, -0.5];
        const err = f.ias - spdTarget;
        if (this.opInt === null) this.opInt = clamp(f.fpa, lo, hi);
        this.opInt = clamp(this.opInt + err * 0.015 * dt, lo, hi);
        fpa = clamp(this.opInt + err * 0.2 + this.accel * 3.5, lo, hi);
        break;
      }
      case 'V/S':
        fpa = vsToFpa(fcu.vs);
        break;
      case 'ALT*': {
        const vs = clamp(altErr * 5, -Math.abs(this.captureVs), Math.abs(this.captureVs));
        fpa = vsToFpa(vs);
        break;
      }
      case 'ALT':
        fpa = vsToFpa(clamp(altErr * 4, -1000, 1000));
        break;
      case 'G/S':
      case 'G/S*':
        if (ils) {
          const hErrFt = Math.tan(ils.gsAngle * DEG - this.ils.gs * DEG) * ils.along / 0.3048; // + = above the beam
          fpa = -this.ils.gs - clamp(hErrFt * 0.012, -2.5, 2.5);
        }
        break;
      default:
        fpa = f.fpa;
    }
    this.fpaCmd = fpa;
  }

  updateThrust(dt, f, sp) {
    const sim = this.sim;
    const lever = this.lever;
    // managed speed target
    let target;
    if (this.fcu.spdManaged) {
      const c = Math.round(f.confPos);
      if (this.takeoffPhase) target = this.srsSpeed;
      else if (c >= 4) target = sp.vapp;
      else if (c >= 2) target = sp.f;
      else if (c === 1) target = sp.s;
      else target = f.altFt < 10000 ? Math.max(250, sp.greenDot) : Math.min(300, sp.vmax - 10);
    } else target = this.fcu.spd;
    this.speedTarget = target;

    // alpha floor
    if (this.athrArmed || this.athrActive) {
      if (!f.onGround && f.alpha > sp.alphaFloor && f.ra > 100) {
        if (!this.alphaFloor) this.say('alpha-floor');
        this.alphaFloor = true;
      }
    }
    let thrust = leverToThrust(lever);
    let athrActive = false;
    let mode = '';
    if (this.alphaFloor) {
      thrust = 1;
      mode = f.alpha > sp.alphaProt - 1 ? 'A.FLOOR' : 'TOGA LK';
    } else if (this.athrArmed && lever > 0.02 && lever <= DETENTS.CL + 0.02 && !f.onGround) {
      athrActive = true;
      // open climb/descent use fixed thrust, the elevator flies the speed
      if (this.vert === 'OP CLB' || (this.vert === 'SRS' && !this.takeoffPhase)) {
        mode = 'THR CLB';
        athrActive = false;
        thrust = leverToThrust(lever);
      } else if (this.vert === 'OP DES') {
        mode = 'THR IDLE';
        athrActive = false;
        thrust = 0;
      } else mode = 'SPEED';
    } else if (lever > DETENTS.CL + 0.02) {
      mode = lever > DETENTS.FLX + 0.05 ? 'MAN TOGA' : 'MAN FLX';
    }
    if (this.athrArmed && lever < 0.02 && !f.onGround && this.athrActive) {
      // levers to idle disconnects the autothrust
      this.athrArmed = false;
      this.say('athr-off');
    }
    this.athrActive = athrActive;
    this.athrMode = mode;

    if (this.reverse) {
      this.reverserPos = Math.min(1, this.reverserPos + dt / 1.5);
      thrust = this.reverserPos >= 1 ? 0.8 : 0;
    } else {
      this.reverserPos = Math.max(0, this.reverserPos - dt / 1.5);
      if (this.reverserPos > 0) thrust = 0;
    }
    const revAngle = this.reverserPos > 0.99 ? 2.0 : 0;
    sim.set('propulsion/engine[0]/reverser-angle-rad', revAngle);
    sim.set('propulsion/engine[1]/reverser-angle-rad', revAngle);
    sim.set('athr/active', athrActive ? 1 : 0);
    sim.set('athr/speed-cmd-kts', target);
    sim.set('athr/thrust-limit', leverToThrust(Math.min(lever, DETENTS.CL)));
    sim.set('athr/manual-thrust', thrust);
  }

  updateHighLift(f) {
    this.sim.set('fcs/flap-cmd-norm', this.flapsLever / 4);
    this.sim.set('gear/gear-cmd-norm', this.gearLever);
  }

  updateGroundSystems(dt, f, input) {
    const sim = this.sim;
    // ground spoilers: armed + main gear on ground + levers at idle (or reverse)
    let spoilers = 0;
    if (f.wowMain && (this.groundSpoilersArmed || this.reverse) && this.lever < 0.05 && f.gs > 20) spoilers = 1;
    if (f.wowMain && this.reverse) spoilers = 1;
    if (spoilers && !this.spoilersOut) this.say('spoilers');
    this.spoilersOut = !!spoilers;
    sim.set('fcs/spoiler-cmd-norm', spoilers);
    // in-flight speedbrakes retract with high alpha or TOGA
    const sb = !f.onGround && (f.alpha > this.speeds.alphaProt - 1 || this.lever > DETENTS.FLX) ? 0 : this.speedbrake;
    sim.set('fcs/speedbrake-cmd-norm', sb);

    // brakes: pedals override the autobrake
    let brake = input.brake;
    if (this.autobrake !== 'OFF' && spoilers && !this.autobrakeActive && f.wowMain) this.autobrakeActive = true;
    if (this.autobrakeActive && (input.brake > 0.1 || this.lever > 0.1)) {
      this.autobrake = 'OFF';
      this.autobrakeActive = false;
    }
    if (this.autobrakeActive && f.gs < 3) brake = 1; // hold the aircraft once stopped
    else if (this.autobrakeActive) {
      const target = this.autobrake === 'LO' ? 1.7 : 3.0;
      this.abCmd = clamp((this.abCmd ?? 0.3) + (target - this.decel) * 0.4 * dt, 0, 1);
      brake = Math.max(brake, this.abCmd);
    } else this.abCmd = 0.3;
    sim.set('fcs/left-brake-cmd-norm', brake);
    sim.set('fcs/right-brake-cmd-norm', brake);
  }
}

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
