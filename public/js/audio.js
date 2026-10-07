// Procedural sounds (engines, wind, warnings) and spoken callouts.
export function createAudio() {
  let ctx = null, n = null;
  let enabled = true;
  const spoken = [];

  function noiseBuffer(seconds = 2) {
    const buf = ctx.createBuffer(1, ctx.sampleRate * seconds, ctx.sampleRate);
    const d = buf.getChannelData(0);
    let b = 0;
    for (let i = 0; i < d.length; i++) {
      b = 0.97 * b + 0.03 * (Math.random() * 2 - 1); // brownish
      d[i] = b * 6 + (Math.random() * 2 - 1) * 0.15;
    }
    return buf;
  }

  function build() {
    const master = ctx.createGain();
    master.gain.value = 0;
    master.connect(ctx.destination);
    const src = (type, freq) => {
      const o = ctx.createOscillator();
      o.type = type;
      o.frequency.value = freq;
      o.start();
      return o;
    };
    const gain = (v, dest) => {
      const g = ctx.createGain();
      g.gain.value = v;
      g.connect(dest);
      return g;
    };
    // jet: fan whine (two detuned tones) + broadband rumble
    const jetWhineGain = gain(0, master);
    const whine1 = src('sawtooth', 800), whine2 = src('triangle', 1210);
    const whineFilter = ctx.createBiquadFilter();
    whineFilter.type = 'bandpass';
    whineFilter.Q.value = 3;
    whine1.connect(whineFilter);
    whine2.connect(whineFilter);
    whineFilter.connect(jetWhineGain);
    const rumble = ctx.createBufferSource();
    rumble.buffer = noiseBuffer();
    rumble.loop = true;
    rumble.start();
    const rumbleFilter = ctx.createBiquadFilter();
    rumbleFilter.type = 'lowpass';
    rumbleFilter.frequency.value = 300;
    const rumbleGain = gain(0, master);
    rumble.connect(rumbleFilter).connect(rumbleGain);
    // piston engine
    const pistonGain = gain(0, master);
    const piston = src('sawtooth', 80), pistonSub = src('square', 40);
    const pistonFilter = ctx.createBiquadFilter();
    pistonFilter.type = 'lowpass';
    pistonFilter.frequency.value = 500;
    piston.connect(pistonFilter);
    pistonSub.connect(gain(0.4, pistonFilter));
    pistonFilter.connect(pistonGain);
    // wind
    const wind = ctx.createBufferSource();
    wind.buffer = noiseBuffer();
    wind.loop = true;
    wind.start();
    const windFilter = ctx.createBiquadFilter();
    windFilter.type = 'bandpass';
    windFilter.frequency.value = 700;
    windFilter.Q.value = 0.5;
    const windGain = gain(0, master);
    wind.connect(windFilter).connect(windGain);
    return { master, jetWhineGain, whine1, whine2, whineFilter, rumbleGain, rumbleFilter, pistonGain, piston, pistonSub, pistonFilter, windGain, windFilter };
  }

  function tone(freq, start, dur, vol = 0.18, type = 'sine') {
    const o = ctx.createOscillator(), g = ctx.createGain();
    o.type = type;
    o.frequency.value = freq;
    g.gain.setValueAtTime(0.0001, start);
    g.gain.exponentialRampToValueAtTime(vol, start + 0.015);
    g.gain.exponentialRampToValueAtTime(0.0001, start + dur);
    o.connect(g).connect(ctx.destination);
    o.start(start);
    o.stop(start + dur + 0.05);
  }

  const api = {
    start() {
      if (!enabled) return;
      try {
        if (!ctx) {
          ctx = new (window.AudioContext || window.webkitAudioContext)();
          n = build();
        }
        ctx.resume();
        n.master.gain.setTargetAtTime(0.5, ctx.currentTime, 0.3);
      } catch {
        ctx = null;
      }
    },
    setEnabled(on) {
      enabled = on;
      if (!on) api.mute();
    },
    mute() {
      if (ctx) n.master.gain.setTargetAtTime(0, ctx.currentTime, 0.05);
      window.speechSynthesis?.cancel();
    },
    update({ jet, n1 = 0, rpm = 0, ias = 0, rolling = 0, cockpit = false }) {
      if (!ctx || !enabled) return;
      const t = ctx.currentTime;
      if (jet) {
        const k = n1 / 100;
        n.whine1.frequency.setTargetAtTime(500 + k * 2600, t, 0.2);
        n.whine2.frequency.setTargetAtTime(760 + k * 3900, t, 0.2);
        n.whineFilter.frequency.setTargetAtTime(700 + k * 3000, t, 0.2);
        n.jetWhineGain.gain.setTargetAtTime(n1 > 5 ? (cockpit ? 0.015 : 0.03) + k * 0.05 : 0, t, 0.3);
        n.rumbleGain.gain.setTargetAtTime(n1 > 5 ? 0.1 + k * k * (cockpit ? 0.25 : 0.6) : 0, t, 0.3);
        n.rumbleFilter.frequency.setTargetAtTime(150 + k * 600, t, 0.3);
        n.pistonGain.gain.setTargetAtTime(0, t, 0.1);
      } else {
        const fire = (rpm / 60) * 2;
        n.piston.frequency.setTargetAtTime(Math.max(fire, 1), t, 0.05);
        n.pistonSub.frequency.setTargetAtTime(Math.max(fire / 2, 1), t, 0.05);
        n.pistonFilter.frequency.setTargetAtTime(250 + rpm * 0.25, t, 0.1);
        n.pistonGain.gain.setTargetAtTime(rpm > 100 ? 0.1 + (rpm / 2700) * 0.18 : 0, t, 0.1);
        n.jetWhineGain.gain.setTargetAtTime(0, t, 0.1);
        n.rumbleGain.gain.setTargetAtTime(0, t, 0.1);
      }
      n.windGain.gain.setTargetAtTime(Math.min(0.35, ias * 0.0016 + rolling * 0.003), t, 0.3);
      n.windFilter.frequency.setTargetAtTime(400 + ias * 5, t, 0.3);
    },
    // Airbus autopilot disconnect "cavalry charge"
    cavalry() {
      if (!ctx) return;
      const t = ctx.currentTime;
      [523, 659, 784, 1047, 784, 1047].forEach((f, i) => tone(f, t + i * 0.11, 0.12, 0.15, 'triangle'));
    },
    chime(times = 1) {
      if (!ctx) return;
      const t = ctx.currentTime;
      for (let i = 0; i < times; i++) tone(1046, t + i * 0.5, 0.6, 0.18, 'sine');
    },
    crc() {
      if (!ctx) return;
      const t = ctx.currentTime;
      for (let i = 0; i < 6; i++) tone(i % 2 ? 1100 : 1400, t + i * 0.12, 0.11, 0.15, 'square');
    },
    crash() {
      if (!ctx) return;
      const t = ctx.currentTime;
      const s = ctx.createBufferSource();
      s.buffer = noiseBuffer(1.5);
      const g = ctx.createGain();
      g.gain.setValueAtTime(1.2, t);
      g.gain.exponentialRampToValueAtTime(0.001, t + 1.4);
      s.connect(g).connect(ctx.destination);
      s.start(t);
    },
    // Spoken callout (synthetic voice); skips if the same text is still pending.
    say(text, urgent = false) {
      if (!enabled || !window.speechSynthesis) return;
      if (urgent) window.speechSynthesis.cancel();
      const u = new SpeechSynthesisUtterance(text);
      u.rate = 1.25;
      u.pitch = 0.8;
      u.volume = 1;
      u.lang = 'en-US';
      spoken.push(text);
      window.speechSynthesis.speak(u);
    },
  };
  return api;
}
