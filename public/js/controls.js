// Touch and keyboard input: side stick (left), thrust lever (right), buttons.
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const expo = (v) => 0.3 * v + 0.7 * v * v * v;

export class Controls {
  constructor() {
    this.stick = { id: null, ox: 0, oy: 0, x: 0, y: 0 };
    this.keys = new Set();
    this.trimDir = 0;
    this.brakeHeld = false;
    this.invert = false;
    this.tilt = { enabled: false, g: null, cal: null };
    this.onKey = null;
    this.bindStick();
    this.bindKeys();
    this.bindTilt();
  }

  bindStick() {
    const zone = document.getElementById('stickZone'), base = document.getElementById('stickBase'), knob = document.getElementById('stickKnob');
    const R = 60;
    this.placeBase = (x, y) => {
      base.style.left = `${x}px`;
      base.style.top = `${y}px`;
    };
    this.defaultStick = () => {
      const r = zone.getBoundingClientRect();
      this.placeBase(Math.min(110, r.width / 2), r.height - 100);
    };
    zone.addEventListener('pointerdown', (e) => {
      if (this.stick.id !== null) return;
      this.stick.id = e.pointerId;
      zone.setPointerCapture(e.pointerId);
      const r = zone.getBoundingClientRect();
      this.stick.ox = e.clientX;
      this.stick.oy = e.clientY;
      this.placeBase(e.clientX - r.left, e.clientY - r.top);
      base.classList.add('active');
    });
    zone.addEventListener('pointermove', (e) => {
      if (e.pointerId !== this.stick.id) return;
      let dx = e.clientX - this.stick.ox, dy = e.clientY - this.stick.oy;
      const len = Math.hypot(dx, dy);
      if (len > R) {
        dx *= R / len;
        dy *= R / len;
      }
      this.stick.x = dx / R;
      this.stick.y = dy / R;
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
    });
    const release = (e) => {
      if (e.pointerId !== this.stick.id) return;
      this.stick.id = null;
      this.stick.x = this.stick.y = 0;
      knob.style.transform = '';
      base.classList.remove('active');
      this.defaultStick();
    };
    zone.addEventListener('pointerup', release);
    zone.addEventListener('pointercancel', release);
  }

  // Generic vertical lever (thrust). snap: detent positions.
  bindLever(el, onChange, detents = []) {
    let id = null;
    const set = (e) => {
      const r = el.getBoundingClientRect();
      let v = clamp(1 - (e.clientY - r.top) / r.height, 0, 1);
      for (const d of detents) if (Math.abs(v - d) < 0.045) v = d;
      onChange(v);
    };
    el.addEventListener('pointerdown', (e) => {
      id = e.pointerId;
      el.setPointerCapture(e.pointerId);
      set(e);
    });
    el.addEventListener('pointermove', (e) => e.pointerId === id && set(e));
    el.addEventListener('pointerup', () => (id = null));
    el.addEventListener('pointercancel', () => (id = null));
  }

  bindHold(el, down, up) {
    el.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      el.setPointerCapture(e.pointerId);
      down();
    });
    el.addEventListener('pointerup', up);
    el.addEventListener('pointercancel', up);
  }

  bindKeys() {
    addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
      if (!e.repeat) this.onKey?.(e.code);
      this.keys.add(e.code);
    });
    addEventListener('keyup', (e) => this.keys.delete(e.code));
  }

  bindTilt() {
    addEventListener('devicemotion', (e) => {
      const a = e.accelerationIncludingGravity;
      if (a && a.x !== null) this.tilt.g = [a.x, a.y, a.z];
    });
    screen.orientation?.addEventListener?.('change', () => (this.tilt.cal = null));
  }

  readTilt() {
    const t = this.tilt;
    if (!t.g) return null;
    const [gx, gy, gz] = t.g;
    const angle = ((screen.orientation && screen.orientation.angle) ?? window.orientation ?? 0) * (Math.PI / 180);
    const c = Math.cos(angle), s = Math.sin(angle);
    const sx = gx * c - gy * s, sy = gx * s + gy * c;
    if (!t.cal) {
      const roll0 = Math.atan2(sx, sy);
      const dir = [Math.sin(roll0), Math.cos(roll0)];
      t.cal = { roll0, dir, pitch0: Math.atan2(gz, sx * dir[0] + sy * dir[1]) };
    }
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    const roll = wrap(Math.atan2(sx, sy) - t.cal.roll0);
    const pitch = wrap(Math.atan2(gz, sx * t.cal.dir[0] + sy * t.cal.dir[1]) - t.cal.pitch0);
    return { roll: clamp(-roll / 0.45, -1, 1), pitch: clamp(-pitch / 0.45, -1, 1) };
  }

  // Returns the pilot's inputs for this frame.
  read() {
    const k = this.keys;
    let roll = 0, pitch = 0, yaw = 0;
    if (this.stick.id !== null) {
      roll = expo(this.stick.x);
      pitch = expo(this.stick.y);
    } else if (this.tilt.enabled) {
      const t = this.readTilt();
      if (t) {
        roll = t.roll;
        pitch = t.pitch;
      }
    }
    if (this.invert) pitch = -pitch;
    if (k.has('ArrowLeft') || k.has('KeyA')) roll = -1;
    if (k.has('ArrowRight') || k.has('KeyD')) roll = 1;
    if (k.has('ArrowDown') || k.has('KeyS')) pitch = 1;
    if (k.has('ArrowUp') || k.has('KeyW')) pitch = -1;
    if (k.has('KeyQ')) yaw = -1;
    if (k.has('KeyE')) yaw = 1;
    const trim = k.has('BracketLeft') ? -1 : k.has('BracketRight') ? 1 : this.trimDir;
    const brake = this.brakeHeld || k.has('KeyB') || k.has('Space') ? 1 : 0;
    return { roll, pitch, yaw, trim, brake, throttleUp: k.has('ShiftLeft') || k.has('ShiftRight'), throttleDown: k.has('ControlLeft') || k.has('ControlRight') };
  }
}
