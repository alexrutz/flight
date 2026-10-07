// Cessna 172 systems: direct controls with optional pilot assists.
export const C172_FLAP_LABELS = ['0°', '10°', '20°', '30°'];

export class C172Systems {
  constructor(sim) {
    this.sim = sim;
    this.lever = 0;
    this.flapsLever = 0;
    this.trim = 0;
    this.levelI = 0;
    this.holdHeading = 0;
    this.assist = true;
    this.events = [];
  }

  setupGround(end) {
    this.lever = 0;
    this.flapsLever = 0;
    this.trim = -0.05;
    this.sim.set('fcs/pitch-trim-cmd-norm', this.trim);
    this.holdHeading = end.heading;
  }

  setupAirborne() {
    this.trim = this.sim.get('fcs/pitch-trim-cmd-norm');
    this.lever = this.sim.get('fcs/throttle-cmd-norm');
  }

  setFlaps(lever) {
    this.flapsLever = Math.max(0, Math.min(3, lever));
  }

  update(dt, f, input) {
    const sim = this.sim;
    let roll = input.roll, yaw = input.yaw;
    // Wing leveller and auto rudder (no pedals on a phone)
    if (this.assist && !f.onGround && Math.abs(roll) < 0.05) {
      this.levelI = Math.max(-20, Math.min(20, this.levelI + f.phi * dt));
      roll = Math.max(-0.4, Math.min(0.4, -f.phi * 0.05 - this.levelI * 0.02 - sim.get('velocities/p-rad_sec') * 0.6));
    } else this.levelI = 0;
    if (this.assist && !yaw) {
      if (f.onGround) {
        if (Math.abs(input.roll) > 0.05 || f.gs < 3) this.holdHeading = f.heading;
        const err = ((this.holdHeading - f.heading + 540) % 360) - 180;
        yaw = Math.max(-1, Math.min(1, input.roll + err * 0.14 - sim.get('velocities/r-rad_sec') * 2));
      } else {
        this.holdHeading = f.heading;
        yaw = Math.max(-1, Math.min(1, roll * 0.3 + sim.get('aero/beta-deg') * 0.08));
      }
    }
    sim.set('fcs/aileron-cmd-norm', roll);
    sim.set('fcs/elevator-cmd-norm', -input.pitch);
    sim.set('fcs/rudder-cmd-norm', -yaw);
    sim.set('fcs/throttle-cmd-norm', this.lever);
    sim.set('fcs/mixture-cmd-norm', 1);
    sim.set('fcs/flap-cmd-norm', this.flapsLever / 3);
    sim.set('fcs/left-brake-cmd-norm', input.brake);
    sim.set('fcs/right-brake-cmd-norm', input.brake);
    if (input.trim) {
      this.trim = Math.max(-1, Math.min(1, this.trim + input.trim * 0.12 * dt));
      sim.set('fcs/pitch-trim-cmd-norm', this.trim);
    }
  }
}
