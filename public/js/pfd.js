// Airbus-style primary flight display drawn on a 2D canvas.
const DEG = Math.PI / 180;
const C = {
  sky: '#2f7fd0', ground: '#83582a', white: '#ffffff', green: '#2ee05a', cyan: '#2fd6ff', magenta: '#ff52ff',
  amber: '#ffae1a', red: '#ff3030', yellow: '#ffe32e', tape: '#4b5563', black: '#000',
};

export function createPFD(canvas) {
  const ctx = canvas.getContext('2d');
  let W = 0, H = 0, dpr = 1;
  const font = (px, bold = false) => `${bold ? 'bold ' : ''}${px}px "Segoe UI", system-ui, sans-serif`;

  function resize() {
    const r = canvas.getBoundingClientRect();
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    W = r.width;
    H = r.height;
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
  }

  function text(t, x, y, color, px, align = 'center', bold = false, baseline = 'middle') {
    ctx.fillStyle = color;
    ctx.font = font(px, bold);
    ctx.textAlign = align;
    ctx.textBaseline = baseline;
    ctx.fillText(t, x, y);
  }

  function barber(x, y0, y1, w, a, b) {
    if (y1 <= y0) return;
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y0, w, y1 - y0);
    ctx.clip();
    ctx.fillStyle = a;
    ctx.fillRect(x, y0, w, y1 - y0);
    ctx.fillStyle = b;
    for (let yy = y0 - 10; yy < y1; yy += 8) ctx.fillRect(x, yy, w, 4);
    ctx.restore();
  }

  // --------------------------------------------------------------- FMA
  function fma(x, y, w, h, d) {
    const cols = [0.2, 0.22, 0.22, 0.14, 0.22];
    let cx = x;
    ctx.fillStyle = 'rgba(0,0,0,0.75)';
    ctx.fillRect(x, y, w, h);
    const now = performance.now();
    cols.forEach((cw, i) => {
      const colW = cw * w;
      if (i) {
        ctx.strokeStyle = '#555';
        ctx.beginPath();
        ctx.moveTo(cx, y + 2);
        ctx.lineTo(cx, y + h - 2);
        ctx.stroke();
      }
      const mid = cx + colW / 2;
      const f = d.fma;
      const box = (since) => since && now - since < 10000;
      const write = (t, color, line, boxed) => {
        if (!t) return;
        const ty = y + h * [0.2, 0.5, 0.8][line];
        text(t, mid, ty, color, Math.min(11.5, h * 0.27), 'center', false);
        if (boxed) {
          ctx.strokeStyle = C.white;
          ctx.lineWidth = 1;
          const tw = ctx.measureText(t).width + 6;
          ctx.strokeRect(mid - tw / 2, ty - h * 0.15, tw, h * 0.3);
        }
      };
      if (i === 0) write(f.athr, f.athr === 'A.FLOOR' || f.athr === 'TOGA LK' ? C.amber : C.green, 0, box(f.athrSince));
      if (i === 0 && f.lvr) write(f.lvr, C.white, 1, (now / 500) % 2 < 1);
      if (i === 1) {
        write(f.vert, C.green, 0, box(f.vertSince));
        write(f.vertArmed, C.cyan, 1);
      }
      if (i === 2) {
        write(f.lat, C.green, 0, box(f.latSince));
        write(f.latArmed, C.cyan, 1);
      }
      if (i === 3) write(f.cat, C.white, 0);
      if (i === 4) {
        write(f.ap ? 'AP1' : '', C.white, 0);
        write(f.fd ? '1 FD 2' : '', C.white, 1);
      }
      if (i === 4) write(f.athrState, f.athrActive ? C.white : C.cyan, 2);
      cx += colW;
    });
  }

  // --------------------------------------------------------------- attitude
  function attitude(ax, ay, aw, ah, d) {
    const cx = ax + aw / 2, cy = ay + ah / 2;
    const ppd = ah / 30; // pixels per degree of pitch
    ctx.save();
    ctx.beginPath();
    ctx.rect(ax, ay, aw, ah);
    ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(-d.roll * DEG);
    ctx.translate(0, d.pitch * ppd);
    ctx.fillStyle = C.sky;
    ctx.fillRect(-W, -H * 8, W * 2, H * 8);
    ctx.fillStyle = C.ground;
    ctx.fillRect(-W, 0, W * 2, H * 8);
    ctx.strokeStyle = C.white;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-W, 0);
    ctx.lineTo(W, 0);
    ctx.stroke();
    // heading marks on the horizon
    ctx.lineWidth = 1;
    for (let h = Math.ceil((d.hdg - 40) / 10) * 10; h <= d.hdg + 40; h += 10) {
      const x = (h - d.hdg) * ppd;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, 4);
      ctx.stroke();
    }
    ctx.font = font(9);
    ctx.fillStyle = C.white;
    ctx.textBaseline = 'middle';
    for (let p = -30; p <= 30; p += 2.5) {
      if (!p) continue;
      const y = -p * ppd;
      const major = p % 10 === 0, mid = p % 5 === 0;
      const half = major ? 22 : mid ? 12 : 6;
      ctx.beginPath();
      ctx.moveTo(-half, y);
      ctx.lineTo(half, y);
      ctx.stroke();
      if (major) {
        ctx.textAlign = 'right';
        ctx.fillText(Math.abs(p), -half - 2, y);
        ctx.textAlign = 'left';
        ctx.fillText(Math.abs(p), half + 2, y);
      }
    }
    ctx.restore();

    // bank scale
    ctx.save();
    ctx.translate(cx, cy);
    const r = ah * 0.42;
    ctx.strokeStyle = C.white;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.arc(0, 0, r, -150 * DEG, -30 * DEG);
    ctx.stroke();
    for (const b of [-45, -30, -20, -10, 10, 20, 30, 45]) {
      const a = (b - 90) * DEG, len = Math.abs(b) % 30 === 0 || Math.abs(b) === 45 ? 7 : 4;
      ctx.beginPath();
      ctx.moveTo(Math.cos(a) * r, Math.sin(a) * r);
      ctx.lineTo(Math.cos(a) * (r + len), Math.sin(a) * (r + len));
      ctx.stroke();
    }
    ctx.strokeStyle = C.green;
    for (const b of [-67, 67]) {
      const a = (b - 90) * DEG;
      for (const off of [2, 5]) {
        ctx.beginPath();
        ctx.moveTo(Math.cos(a) * (r + off) - 2, Math.sin(a) * (r + off));
        ctx.lineTo(Math.cos(a) * (r + off) + 2, Math.sin(a) * (r + off));
        ctx.stroke();
      }
    }
    ctx.fillStyle = C.yellow;
    ctx.beginPath();
    ctx.moveTo(0, -r);
    ctx.lineTo(-6, -r - 8);
    ctx.lineTo(6, -r - 8);
    ctx.fill();
    ctx.rotate(-d.roll * DEG);
    ctx.strokeStyle = C.yellow;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, -r + 1);
    ctx.lineTo(-6, -r + 9);
    ctx.lineTo(6, -r + 9);
    ctx.closePath();
    ctx.stroke();
    // sideslip index (trapezoid under the roll pointer)
    const beta = Math.max(-12, Math.min(12, (d.beta || 0) * 1.5));
    ctx.strokeRect(-6 + beta, -r + 11, 12, 3);
    ctx.restore();

    // flight director bars
    if (d.fdBars) {
      ctx.strokeStyle = C.green;
      ctx.lineWidth = 2.5;
      const px = Math.max(-1, Math.min(1, d.fdBars.roll)) * aw * 0.3;
      const py = Math.max(-1, Math.min(1, d.fdBars.pitch)) * ah * 0.3;
      ctx.beginPath();
      ctx.moveTo(cx + px, cy - ah * 0.3);
      ctx.lineTo(cx + px, cy + ah * 0.3);
      ctx.moveTo(cx - aw * 0.3, cy + py);
      ctx.lineTo(cx + aw * 0.3, cy + py);
      ctx.stroke();
    }
    // flight path vector ("bird")
    if (d.showFpv) {
      const dx = (d.driftDeg || 0) * ppd, dy = (d.pitch - d.fpa) * ppd;
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-d.roll * DEG);
      ctx.translate(dx, dy);
      ctx.strokeStyle = C.green;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.arc(0, 0, 5, 0, Math.PI * 2);
      ctx.moveTo(-5, 0);
      ctx.lineTo(-14, 0);
      ctx.moveTo(5, 0);
      ctx.lineTo(14, 0);
      ctx.moveTo(0, -5);
      ctx.lineTo(0, -10);
      ctx.stroke();
      ctx.restore();
    }
    // aircraft symbol
    ctx.fillStyle = C.black;
    ctx.strokeStyle = C.yellow;
    ctx.lineWidth = 2;
    for (const s of [-1, 1]) {
      ctx.beginPath();
      ctx.moveTo(cx + s * 16, cy);
      ctx.lineTo(cx + s * 42, cy);
      ctx.moveTo(cx + s * 16, cy);
      ctx.lineTo(cx + s * 16, cy + 8);
      ctx.stroke();
    }
    ctx.fillRect(cx - 3, cy - 3, 6, 6);
    ctx.strokeRect(cx - 3, cy - 3, 6, 6);

    // ILS deviation scales
    if (d.ils) {
      const dots = (n) => n * aw * 0.13;
      ctx.strokeStyle = C.white;
      ctx.fillStyle = C.white;
      for (let k = -2; k <= 2; k++) {
        if (!k) continue;
        ctx.beginPath();
        ctx.arc(cx + dots(k), ay + ah - 9, 2.2, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(ax + aw - 9, cy + (k * ah * 0.13), 2.2, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.fillStyle = C.magenta;
      if (d.ils.valid) diamond(cx + dots(-d.ils.locDots), ay + ah - 9, 6, 4);
      if (d.ils.gsValid) diamond(ax + aw - 9, cy - d.ils.gsDots * ah * 0.13, 4, 6);
      text(d.ils.ident, ax + 3, ay + 8, C.magenta, 10, 'left', true);
      if (d.ils.dme != null) text(`${d.ils.dme.toFixed(1)} NM`, ax + 3, ay + 20, C.magenta, 10, 'left');
    }
    // radio altitude
    if (d.ra < 2500 && d.ra > -10) {
      const ra = d.ra < 10 ? Math.round(d.ra) : d.ra < 50 ? Math.round(d.ra / 5) * 5 : Math.round(d.ra / 10) * 10;
      text(String(Math.max(0, ra)), cx, ay + ah - 22, d.minimums && d.ra < d.minimums ? C.amber : C.green, 14, 'center', true);
    }
  }

  function diamond(x, y, rx, ry) {
    ctx.beginPath();
    ctx.moveTo(x - rx, y);
    ctx.lineTo(x, y - ry);
    ctx.lineTo(x + rx, y);
    ctx.lineTo(x, y + ry);
    ctx.closePath();
    ctx.fill();
  }

  // --------------------------------------------------------------- speed tape
  function speedTape(x, y, w, h, d) {
    const cy = y + h / 2, ppk = h / 80; // 80 kt visible
    const Y = (v) => cy - (v - d.ias) * ppk;
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, w, h);
    ctx.clip();
    ctx.fillStyle = C.tape;
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = C.white;
    ctx.lineWidth = 1;
    for (let v = Math.max(30, Math.floor((d.ias - 45) / 10) * 10); v < d.ias + 45; v += 10) {
      const yy = Y(v);
      ctx.beginPath();
      ctx.moveTo(x + w - 8, yy);
      ctx.lineTo(x + w, yy);
      ctx.stroke();
      if (v % 20 === 0) text(String(v), x + w - 11, yy, C.white, 11, 'right');
    }
    const s = d.speeds;
    if (s && !d.onGround) {
      // VMAX barber pole (red/black) from VMAX upwards
      barber(x + w - 6, y, Y(s.vmax), 6, C.red, C.black);
      // alpha max (red) and alpha prot (amber/black) strips, VLS amber line
      ctx.fillStyle = C.red;
      ctx.fillRect(x + w - 6, Y(s.vAlphaMax), 6, y + h - Y(s.vAlphaMax));
      barber(x + w - 6, Y(s.vAlphaProt), Y(s.vAlphaMax), 6, C.amber, C.black);
      ctx.fillStyle = C.amber;
      ctx.fillRect(x + w - 3, Y(s.vls), 3, Y(s.vAlphaProt) - Y(s.vls));
      // characteristic speeds
      const c = Math.round(d.conf);
      if (c === 0) {
        ctx.strokeStyle = C.green;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x + w - 12, Y(s.greenDot), 3.5, 0, Math.PI * 2);
        ctx.stroke();
      } else if (c === 1) text('S', x + w - 13, Y(s.s), C.green, 11, 'center', true);
      else if (c < 4) text('F', x + w - 13, Y(s.f), C.green, 11, 'center', true);
    }
    if (d.vspeeds && d.onGround) {
      text('1', x + w - 12, Y(d.vspeeds.v1), C.cyan, 11, 'center', true);
      ctx.strokeStyle = C.cyan;
      ctx.beginPath();
      ctx.arc(x + w - 12, Y(d.vspeeds.vr), 3.5, 0, Math.PI * 2);
      ctx.stroke();
    }
    // speed trend
    const trend = (d.accel || 0) * 10;
    if (Math.abs(trend) > 2) {
      ctx.strokeStyle = C.yellow;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x + w - 18, cy);
      ctx.lineTo(x + w - 18, Y(d.ias + trend));
      ctx.stroke();
      const ty = Y(d.ias + trend), dir = trend > 0 ? 1 : -1;
      ctx.beginPath();
      ctx.moveTo(x + w - 22, ty + dir * 5);
      ctx.lineTo(x + w - 18, ty);
      ctx.lineTo(x + w - 14, ty + dir * 5);
      ctx.stroke();
    }
    // target speed
    if (d.speedTarget) {
      const ty = Math.max(y + 6, Math.min(y + h - 6, Y(d.speedTarget)));
      ctx.fillStyle = d.spdManaged ? C.magenta : C.cyan;
      ctx.beginPath();
      ctx.moveTo(x + w - 1, ty);
      ctx.lineTo(x + w + 8, ty - 6);
      ctx.lineTo(x + w + 8, ty + 6);
      ctx.fill();
    }
    ctx.restore();
    // reference line and value
    ctx.fillStyle = C.yellow;
    ctx.fillRect(x, cy - 1, w + 6, 2.5);
    ctx.beginPath();
    ctx.moveTo(x + w + 6, cy);
    ctx.lineTo(x + w - 2, cy - 5);
    ctx.lineTo(x + w - 2, cy + 5);
    ctx.fill();
    ctx.fillStyle = C.black;
    ctx.fillRect(x + 1, cy - 9, w - 16, 18);
    text(String(Math.max(0, Math.round(d.ias))), x + w - 17, cy + 1, C.white, 13, 'right', true);
    if (d.mach > 0.5) text(`.${Math.round(d.mach * 1000).toString().padStart(3, '0').slice(0, 3)}`, x + w / 2, y + h + 9, C.green, 11);
    // selected speed numeric when off scale
    if (d.speedTarget && !d.spdManaged) text(String(Math.round(d.speedTarget)), x + w / 2, y - 7, C.cyan, 11);
  }

  // --------------------------------------------------------------- altitude tape
  function altTape(x, y, w, h, d) {
    const cy = y + h / 2, ppf = h / 1000; // 1000 ft visible
    const Y = (a) => cy - (a - d.alt) * ppf;
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, w, h);
    ctx.clip();
    ctx.fillStyle = C.tape;
    ctx.fillRect(x, y, w, h);
    // ground reference (red ribbon) from radio altitude
    if (d.ra < 600) {
      const gy = Y(d.alt - d.ra);
      ctx.fillStyle = C.red;
      ctx.fillRect(x + w - 6, gy, 6, y + h - gy);
    }
    ctx.strokeStyle = C.white;
    ctx.lineWidth = 1;
    for (let a = Math.floor((d.alt - 600) / 100) * 100; a < d.alt + 600; a += 100) {
      const yy = Y(a);
      ctx.beginPath();
      ctx.moveTo(x, yy);
      ctx.lineTo(x + 7, yy);
      ctx.stroke();
      if (a % 500 === 0) text(String(Math.round(a / 100)).padStart(2, '0'), x + 10, yy, C.white, 11, 'left');
    }
    // selected altitude
    if (d.altSel != null) {
      const sy = Math.max(y + 5, Math.min(y + h - 5, Y(d.altSel)));
      ctx.strokeStyle = C.cyan;
      ctx.lineWidth = 2;
      ctx.strokeRect(x + 1, sy - 6, 12, 12);
    }
    ctx.restore();
    if (d.altSel != null) text(String(d.altSel), x + w / 2, y - 7, C.cyan, 11);
    ctx.strokeStyle = C.yellow;
    ctx.lineWidth = 1.5;
    ctx.fillStyle = C.black;
    ctx.fillRect(x + 2, cy - 10, w - 4, 20);
    ctx.strokeRect(x + 2, cy - 10, w - 4, 20);
    text(String(Math.round(d.alt)), x + w - 5, cy + 1, C.green, 12, 'right', true);
    text(d.baro || 'QNH 1013', x + w / 2, y + h + 9, C.cyan, 9);
  }

  function vsScale(x, y, w, h, d) {
    const cy = y + h / 2;
    ctx.fillStyle = 'rgba(75,85,99,0.85)';
    ctx.beginPath();
    ctx.moveTo(x, y + 10);
    ctx.lineTo(x + w, y + 22);
    ctx.lineTo(x + w, y + h - 22);
    ctx.lineTo(x, y + h - 10);
    ctx.fill();
    const scale = (vs) => {
      const a = Math.min(6000, Math.abs(vs));
      const k = a <= 1000 ? (a / 1000) * 0.5 : a <= 2000 ? 0.5 + ((a - 1000) / 1000) * 0.25 : 0.75 + ((a - 2000) / 4000) * 0.25;
      return -Math.sign(vs) * k * (h / 2 - 14);
    };
    ctx.strokeStyle = C.white;
    for (const v of [-6000, -2000, -1000, 0, 1000, 2000, 6000]) {
      const yy = cy + scale(v);
      ctx.beginPath();
      ctx.moveTo(x + 2, yy);
      ctx.lineTo(x + (v ? 6 : 9), yy);
      ctx.stroke();
    }
    const vy = cy + scale(d.vs);
    const warn = Math.abs(d.vs) > 6000 || (d.ra < 2500 && d.vs < -2000);
    ctx.strokeStyle = warn ? C.amber : C.green;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(x + w + 6, cy);
    ctx.lineTo(x + 2, vy);
    ctx.stroke();
    if (Math.abs(d.vs) >= 200) {
      ctx.fillStyle = C.black;
      ctx.fillRect(x + 2, vy + (d.vs > 0 ? -16 : 2), w - 2, 13);
      text(String(Math.round(Math.abs(d.vs) / 100)).padStart(2, '0'), x + w / 2 + 1, vy + (d.vs > 0 ? -9 : 9), warn ? C.amber : C.green, 10, 'center', true);
    }
  }

  function headingTape(x, y, w, h, d) {
    const cx = x + w / 2, ppd = w / 50;
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, w, h);
    ctx.clip();
    ctx.fillStyle = C.tape;
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = C.white;
    ctx.lineWidth = 1;
    for (let hd = Math.floor((d.hdg - 30) / 5) * 5; hd <= d.hdg + 30; hd += 5) {
      const xx = cx + (hd - d.hdg) * ppd;
      const n = ((hd % 360) + 360) % 360;
      ctx.beginPath();
      ctx.moveTo(xx, y);
      ctx.lineTo(xx, y + (n % 10 === 0 ? 6 : 3));
      ctx.stroke();
      if (n % 10 === 0) text(String(n / 10), xx, y + h * 0.65, C.white, n % 30 === 0 ? 11 : 9);
    }
    const mark = (hd, color, shape) => {
      let dd = ((hd - d.hdg + 540) % 360) - 180;
      dd = Math.max(-24, Math.min(24, dd));
      const xx = cx + dd * ppd;
      ctx.fillStyle = color;
      ctx.strokeStyle = color;
      if (shape === 'tri') {
        ctx.beginPath();
        ctx.moveTo(xx, y + 7);
        ctx.lineTo(xx - 5, y);
        ctx.lineTo(xx + 5, y);
        ctx.fill();
      } else if (shape === 'diamond') {
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(xx, y + 2);
        ctx.lineTo(xx - 3, y + 6);
        ctx.lineTo(xx, y + 10);
        ctx.lineTo(xx + 3, y + 6);
        ctx.closePath();
        ctx.stroke();
      } else {
        ctx.fillRect(xx - 1, y, 2, h * 0.5);
      }
    };
    if (d.ils) mark(d.ils.course, C.magenta, 'bar');
    if (d.hdgSel != null) mark(d.hdgSel, C.cyan, 'tri');
    mark(d.track, C.green, 'diamond');
    ctx.restore();
    ctx.fillStyle = C.yellow;
    ctx.fillRect(cx - 1, y - 4, 2, h * 0.55 + 4);
  }

  function draw(d) {
    if (!W) resize();
    if (!W || !H) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const fmaH = d.fma ? Math.round(H * 0.2) : 0;
    const hdgH = Math.round(H * 0.11);
    const speedW = Math.round(W * 0.15), altW = Math.round(W * 0.14), vsW = Math.round(W * 0.06);
    const top = fmaH + 14, bottom = H - hdgH - 4;
    const ah = bottom - top;
    const ax = speedW + 12, aw = W - ax - altW - vsW - 14;
    if (d.fma) fma(0, 0, W, fmaH, d);
    attitude(ax, top, aw, ah, d);
    speedTape(2, top, speedW, ah, d);
    altTape(ax + aw + 6, top, altW, ah, d);
    vsScale(ax + aw + 6 + altW + 1, top, vsW, ah, d);
    headingTape(ax, bottom + 2, aw, hdgH, d);
  }

  return { draw, resize };
}
