// Procedural Airbus A320 model built from published dimensions.
// Origin = centre of gravity, nose towards -z, right wing +x, up +y (meters).
// Structural coordinates (x aft from the nose, y right, z up from the fuselage
// centreline) match the JSBSim model in wasm/aircraft-src/gen_a320.py.
import * as THREE from 'three';

const CG_X = 16.4, CG_Z = -0.5;
const P = (xs, ys, zs) => new THREE.Vector3(ys, zs - CG_Z, xs - CG_X);
const DEG = Math.PI / 180;

const LENGTH = 37.57;
// fuselage stations: [x, centre z, half height, half width]
const FUSELAGE = [
  [0.0, -0.35, 0.02, 0.02], [0.15, -0.35, 0.42, 0.40], [0.5, -0.33, 0.78, 0.74], [1.0, -0.28, 1.12, 1.06],
  [1.7, -0.2, 1.46, 1.38], [2.6, -0.11, 1.76, 1.68], [3.6, -0.04, 1.96, 1.89], [4.6, 0, 2.06, 1.975],
  [26.8, 0, 2.06, 1.975], [28.3, 0.12, 1.92, 1.88], [30.2, 0.42, 1.6, 1.58], [32.2, 0.72, 1.18, 1.14],
  [34.2, 0.98, 0.78, 0.7], [36.2, 1.18, 0.4, 0.33], [37.2, 1.25, 0.18, 0.15], [LENGTH, 1.27, 0.08, 0.06],
];

function liveryTexture() {
  const W = 4096, H = 512;
  const cv = document.createElement('canvas');
  cv.width = W;
  cv.height = H;
  const g = cv.getContext('2d');
  const X = (x) => (x / LENGTH) * W;
  // v = theta / 2pi, theta = 0 at the top, increasing down the right side
  const Yth = (thDeg) => (thDeg / 360) * H;
  g.fillStyle = '#f4f5f7';
  g.fillRect(0, 0, W, H);
  // belly
  g.fillStyle = '#c9ced6';
  g.fillRect(0, Yth(118), W, Yth(242) - Yth(118));
  // cheatline
  for (const [a, b] of [[100, 108], [252, 260]]) {
    g.fillStyle = '#1d4f91';
    g.fillRect(X(3), Yth(a), X(37) - X(3), Yth(b) - Yth(a));
  }
  // windows (both sides at ~0.55 m above the centreline)
  const winTh = [Math.acos(0.55 / 2.06) / DEG, 360 - Math.acos(0.55 / 2.06) / DEG];
  g.fillStyle = '#1b222c';
  for (let x = 7.0; x < 29.2; x += 0.508) {
    if (x > 15.0 && x < 16.6) continue; // overwing exits
    for (const th of winTh) {
      g.beginPath();
      g.roundRect(X(x), Yth(th - 4.2), X(0.24), Yth(8.4), 6);
      g.fill();
    }
  }
  // doors
  g.strokeStyle = '#9aa3ad';
  g.lineWidth = 3;
  for (const x of [4.9, 15.45, 16.15, 29.6]) {
    for (const th of winTh) {
      const w = x > 15 && x < 17 ? 0.55 : 0.85;
      g.beginPath();
      g.roundRect(X(x), Yth(th - (x > 15 && x < 17 ? 9 : 14)), X(w), Yth(x > 15 && x < 17 ? 22 : 36), 10);
      g.stroke();
    }
  }
  // cockpit windows
  g.fillStyle = '#141a22';
  for (const side of [1, -1]) {
    const th = (d) => (side > 0 ? d : 360 - d);
    const poly = [[2.05, th(28)], [3.0, th(22)], [3.35, th(55)], [2.45, th(70)]];
    g.beginPath();
    poly.forEach(([x, t], i) => (i ? g.lineTo(X(x), Yth(t)) : g.moveTo(X(x), Yth(t))));
    g.fill();
    g.beginPath();
    g.moveTo(X(3.45), Yth(th(48)));
    g.lineTo(X(4.1), Yth(th(52)));
    g.lineTo(X(4.0), Yth(th(66)));
    g.lineTo(X(3.5), Yth(th(68)));
    g.fill();
  }
  g.beginPath();
  g.moveTo(X(1.95), Yth(360 - 26));
  g.lineTo(X(2.95), Yth(360 - 20));
  g.lineTo(X(2.95), Yth(20));
  g.lineTo(X(1.95), Yth(26));
  g.fill();
  // titles above the windows on both sides
  g.fillStyle = '#1d4f91';
  g.font = `bold ${Yth(14)}px Arial, sans-serif`;
  g.textBaseline = 'middle';
  g.save();
  g.translate(X(9.5), Yth(winTh[0] - 18));
  g.scale(1, -1);
  g.textAlign = 'left';
  g.fillText('SKY HOPPER', 0, 0);
  g.restore();
  g.save();
  g.translate(X(9.5), Yth(winTh[1] + 18));
  g.textAlign = 'left';
  g.fillText('SKY HOPPER', 0, 0);
  g.restore();
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

function fuselageGeometry() {
  const N = 40;
  const pos = [], uv = [], idx = [];
  FUSELAGE.forEach(([x, cz, ry, rx], i) => {
    for (let k = 0; k <= N; k++) {
      const th = (k / N) * Math.PI * 2;
      const p = P(x, rx * Math.sin(th), cz + ry * Math.cos(th));
      pos.push(p.x, p.y, p.z);
      uv.push(x / LENGTH, 1 - k / N);
    }
    if (i) {
      const a = (i - 1) * (N + 1), b = i * (N + 1);
      for (let k = 0; k < N; k++) idx.push(a + k, a + k + 1, b + k, a + k + 1, b + k + 1, b + k);
    }
  });
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return geo;
}

// Airfoil outline (unit chord, from TE over the top to LE and back underneath)
const AIRFOIL = (() => {
  const pts = [];
  const n = 14;
  const thick = (x) => 0.6 * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
  const camber = (x) => 0.02 * Math.sin(Math.PI * x);
  for (let i = 0; i <= n; i++) {
    const x = 1 - 0.5 * (1 - Math.cos((Math.PI * i) / n));
    pts.push([x, camber(x) + thick(x) * 2]);
  }
  for (let i = 1; i < n; i++) {
    const x = 0.5 * (1 - Math.cos((Math.PI * i) / n));
    pts.push([x, camber(x) - thick(x) * 2]);
  }
  return pts; // y normalised by thickness ratio later
})();

// Lofts a surface through sections {le: Vector3 (structural), chord, t (thickness m), axis: 'span'|'vertical'}
function loft(sections, vertical = false) {
  const pos = [], idx = [];
  const M = AIRFOIL.length;
  sections.forEach((s, i) => {
    for (const [cx, cy] of AIRFOIL) {
      const xs = s.x + cx * s.chord;
      const off = (cy / 0.12) * s.t * 0.5;
      const p = vertical ? P(xs, s.y + off, s.z) : P(xs, s.y, s.z + off);
      pos.push(p.x, p.y, p.z);
    }
    if (i) {
      const a = (i - 1) * M, b = i * M;
      for (let k = 0; k < M; k++) {
        const k1 = (k + 1) % M;
        idx.push(a + k, b + k, a + k1, a + k1, b + k, b + k1);
      }
    }
  });
  // close the end caps
  for (const [i, flip] of [[0, false], [sections.length - 1, true]]) {
    const base = i * M;
    for (let k = 1; k < M - 1; k++) flip ? idx.push(base, base + k, base + k + 1) : idx.push(base, base + k + 1, base + k);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return geo;
}

const WING_Z0 = -1.45, DIHEDRAL = 5.1 * DEG;
const wingZ = (y) => WING_Z0 + (Math.abs(y) - 1.95) * Math.tan(DIHEDRAL);
const WING = [
  { y: 0.0, x: 12.2, chord: 7.0, t: 0.8 },
  { y: 1.95, x: 12.6, chord: 6.4, t: 0.75 },
  { y: 6.3, x: 14.95, chord: 4.25, t: 0.5 },
  { y: 16.9, x: 20.65, chord: 1.55, t: 0.17 },
];
const wingLE = (y) => {
  for (let i = 1; i < WING.length; i++) {
    const a = WING[i - 1], b = WING[i];
    if (y <= b.y) {
      const f = (y - a.y) / (b.y - a.y);
      return { x: a.x + (b.x - a.x) * f, chord: a.chord + (b.chord - a.chord) * f, t: a.t + (b.t - a.t) * f };
    }
  }
  return WING[WING.length - 1];
};

function wingGeometry(side) {
  const secs = WING.map((s) => ({ x: s.x, y: side * s.y, z: s.y < 1.95 ? WING_Z0 : wingZ(s.y), chord: s.chord, t: s.t }));
  return loft(side > 0 ? secs : secs);
}

function sharkletGeometry(side) {
  const tip = WING[WING.length - 1];
  const z0 = wingZ(tip.y);
  const secs = [];
  const n = 6;
  for (let i = 0; i <= n; i++) {
    const f = i / n;
    const bend = Math.min(1, f * 3); // blended curve from the tip upwards
    secs.push({
      x: tip.x + 0.25 + f * 1.4 + (1 - bend) * 0.0,
      y: side * (tip.y + 0.15 * bend + f * 0.35),
      z: z0 + 0.05 + f * 2.35,
      chord: tip.chord * (1 - 0.65 * f),
      t: 0.12,
    });
  }
  return loft(secs, true);
}

function tailGeometry() {
  return loft(
    [
      { x: 28.6, y: 0, z: 1.6, chord: 6.2, t: 0.55 },
      { x: 30.4, y: 0, z: 3.6, chord: 4.5, t: 0.4 },
      { x: 33.4, y: 0, z: 7.7, chord: 2.0, t: 0.18 },
    ],
    true
  );
}

function stabiliserGeometry(side) {
  const dz = Math.tan(6 * DEG);
  return loft([
    { x: 32.2, y: side * 0.6, z: 0.75, chord: 3.9, t: 0.35 },
    { x: 35.4, y: side * 6.2, z: 0.75 + 5.6 * dz, chord: 1.5, t: 0.12 },
  ]);
}

function nacelle() {
  const g = new THREE.Group();
  const prof = [[0, 0.98], [0.15, 1.12], [0.6, 1.2], [2.0, 1.16], [3.4, 1.0], [4.2, 0.78], [4.3, 0.7]];
  const pts = prof.map(([x, r]) => new THREE.Vector2(r, x));
  const body = new THREE.Mesh(new THREE.LatheGeometry(pts, 40), MATS.engine);
  body.rotation.x = Math.PI / 2; // lathe axis (y) -> z (aft)
  g.add(body);
  const lip = new THREE.Mesh(new THREE.TorusGeometry(1.02, 0.08, 10, 40), MATS.lip);
  g.add(lip);
  const fan = new THREE.Mesh(new THREE.CircleGeometry(0.98, 32), MATS.fan);
  fan.position.z = 0.35;
  g.add(fan);
  const spinner = new THREE.Mesh(new THREE.ConeGeometry(0.3, 0.6, 20), MATS.metal);
  spinner.rotation.x = -Math.PI / 2;
  spinner.position.z = 0.15;
  g.add(spinner);
  const core = new THREE.Mesh(new THREE.ConeGeometry(0.55, 1.4, 24), MATS.metal);
  core.rotation.x = Math.PI / 2;
  core.position.z = 4.9;
  g.add(core);
  return g;
}

const MATS = {};
function materials() {
  if (MATS.body) return;
  MATS.body = new THREE.MeshStandardMaterial({ map: liveryTexture(), roughness: 0.45, metalness: 0.1 });
  MATS.wing = new THREE.MeshStandardMaterial({ color: 0xd9dde3, roughness: 0.5, metalness: 0.2 });
  MATS.wingDark = new THREE.MeshStandardMaterial({ color: 0xb5bcc6, roughness: 0.55, metalness: 0.2 });
  MATS.tail = new THREE.MeshStandardMaterial({ color: 0x1d4f91, roughness: 0.4, metalness: 0.1 });
  MATS.engine = new THREE.MeshStandardMaterial({ color: 0xe8ebef, roughness: 0.35, metalness: 0.3, side: THREE.DoubleSide });
  MATS.lip = new THREE.MeshStandardMaterial({ color: 0xb8c0c8, roughness: 0.25, metalness: 0.8 });
  MATS.fan = new THREE.MeshStandardMaterial({ color: 0x23272d, roughness: 0.6, metalness: 0.6 });
  MATS.metal = new THREE.MeshStandardMaterial({ color: 0x8a9098, roughness: 0.35, metalness: 0.8 });
  MATS.gear = new THREE.MeshStandardMaterial({ color: 0x9aa1a8, roughness: 0.5, metalness: 0.6 });
  MATS.tyre = new THREE.MeshStandardMaterial({ color: 0x1c1d20, roughness: 0.9 });
}

function lightSprite(color, size) {
  const cv = document.createElement('canvas');
  cv.width = cv.height = 64;
  const g = cv.getContext('2d');
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.2, color);
  grad.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  const mat = new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), transparent: true, depthWrite: false, blending: THREE.AdditiveBlending });
  const s = new THREE.Sprite(mat);
  s.scale.setScalar(size);
  return s;
}

export function createA320() {
  materials();
  const root = new THREE.Group();
  const add = (geo, mat, parent = root) => {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = true;
    parent.add(m);
    return m;
  };
  add(fuselageGeometry(), MATS.body);
  add(wingGeometry(1), MATS.wing);
  add(wingGeometry(-1), MATS.wing);
  add(sharkletGeometry(1), MATS.tail);
  add(sharkletGeometry(-1), MATS.tail);
  add(tailGeometry(), MATS.tail);
  add(stabiliserGeometry(1), MATS.wing);
  add(stabiliserGeometry(-1), MATS.wing);
  // wing-body fairing
  const fairing = add(new THREE.SphereGeometry(1, 24, 12), MATS.body);
  fairing.scale.set(2.1, 0.9, 6.5);
  fairing.position.copy(P(16.3, 0, -1.55));
  fairing.material = MATS.wingDark;

  // engines and pylons
  for (const side of [-1, 1]) {
    const eng = nacelle();
    eng.position.copy(P(12.0, side * 5.75, -2.15));
    eng.traverse((o) => (o.castShadow = true));
    root.add(eng);
    const pylon = add(new THREE.BoxGeometry(0.35, 1.0, 4.2), MATS.wing);
    pylon.position.copy(P(14.6, side * 5.75, -1.25));
    pylon.rotation.x = -0.1;
  }

  // flaps (inboard and outboard) hinge near the trailing edge
  const flaps = [];
  for (const side of [-1, 1]) {
    for (const [y0, y1] of [[2.2, 6.2], [6.4, 13.6]]) {
      const a = wingLE(y0), b = wingLE(y1);
      const ca = a.chord * 0.27, cb = b.chord * 0.27;
      const geo = loft([
        { x: 0, y: 0, z: 0, chord: ca, t: a.t * 0.35 },
        { x: (b.x + b.chord - cb) - (a.x + a.chord - ca), y: side * (y1 - y0), z: (y1 - y0) * Math.tan(DIHEDRAL), chord: cb, t: b.t * 0.35 },
      ]);
      geo.translate(0, CG_Z, CG_X); // undo P's offset: hinge-relative coordinates
      const pivot = new THREE.Group();
      pivot.position.copy(P(a.x + a.chord - ca, side * y0, wingZ(y0) - 0.05));
      const m = add(geo, MATS.wingDark, pivot);
      m.castShadow = true;
      root.add(pivot);
      flaps.push(pivot);
    }
  }
  // spoilers (5 panels per wing) on the upper surface
  const spoilers = [];
  for (const side of [-1, 1]) {
    for (let k = 0; k < 5; k++) {
      const y0 = 6.6 + k * 1.45, y1 = y0 + 1.35;
      const a = wingLE((y0 + y1) / 2);
      const pivot = new THREE.Group();
      pivot.position.copy(P(a.x + a.chord * 0.6, side * (y0 + y1) / 2, wingZ((y0 + y1) / 2) + a.t * 0.42));
      const panel = add(new THREE.BoxGeometry(y1 - y0, 0.04, a.chord * 0.13), MATS.wingDark, pivot);
      panel.position.z = a.chord * 0.065;
      pivot.rotation.z = side * DIHEDRAL;
      root.add(pivot);
      spoilers.push(pivot);
    }
  }

  // landing gear
  const gear = [];
  const makeLeg = (xs, ys, wheelR, wheelW, wheelSpacing, legTop, hingeAxis, retractSign) => {
    const hinge = new THREE.Group();
    hinge.position.copy(P(xs, ys, legTop));
    root.add(hinge);
    const legLen = legTop - (-3.9 + wheelR);
    const strut = add(new THREE.CylinderGeometry(0.14, 0.14, legLen, 10), MATS.gear, hinge);
    strut.position.y = -legLen / 2;
    const wheels = new THREE.Group();
    wheels.position.y = -legLen;
    hinge.add(wheels);
    for (const dx of [-wheelSpacing / 2, wheelSpacing / 2]) {
      const w = add(new THREE.CylinderGeometry(wheelR, wheelR, wheelW, 20), MATS.tyre, wheels);
      w.rotation.z = Math.PI / 2;
      w.position.x = dx;
      const hub = add(new THREE.CylinderGeometry(wheelR * 0.5, wheelR * 0.5, wheelW + 0.02, 12), MATS.gear, wheels);
      hub.rotation.z = Math.PI / 2;
      hub.position.x = dx;
    }
    const axle = add(new THREE.CylinderGeometry(0.07, 0.07, wheelSpacing + 0.1, 8), MATS.gear, wheels);
    axle.rotation.z = Math.PI / 2;
    gear.push({ hinge, wheels, legLen, axis: hingeAxis, sign: retractSign });
  };
  makeLeg(5.07, 0, 0.38, 0.28, 0.6, -1.7, 'x', 1);
  makeLeg(17.71, -3.795, 0.57, 0.38, 0.95, -1.55, 'z', 1); // retracts inwards
  makeLeg(17.71, 3.795, 0.57, 0.38, 0.95, -1.55, 'z', -1);

  // lights: nav (red left, green right, white tail), beacons, strobes
  const navL = lightSprite('rgba(255,40,30,0.9)', 1.6);
  navL.position.copy(P(20.9, -17.1, wingZ(16.9) + 0.1));
  const navR = lightSprite('rgba(40,255,90,0.9)', 1.6);
  navR.position.copy(P(20.9, 17.1, wingZ(16.9) + 0.1));
  const tailNav = lightSprite('rgba(255,255,255,0.9)', 1.2);
  tailNav.position.copy(P(37.5, 0, 1.3));
  const beaconTop = lightSprite('rgba(255,30,20,1)', 2.2);
  beaconTop.position.copy(P(18, 0, 2.12));
  const beaconBottom = lightSprite('rgba(255,30,20,1)', 2.2);
  beaconBottom.position.copy(P(16, 0, -2.1));
  const strobes = [lightSprite('rgba(255,255,255,1)', 4), lightSprite('rgba(255,255,255,1)', 4)];
  strobes[0].position.copy(P(21.2, -17.2, wingZ(16.9) + 0.1));
  strobes[1].position.copy(P(21.2, 17.2, wingZ(16.9) + 0.1));
  root.add(navL, navR, tailNav, beaconTop, beaconBottom, ...strobes);

  let t = 0;
  return {
    group: root,
    eyepoint: P(3.0, -0.55, 0.85),
    length: LENGTH,
    update(s, dt) {
      t += dt;
      // gear: 0 = up, 1 = down; wheels follow the strut compression from JSBSim
      const g = s.gearPos ?? 1;
      gear.forEach((leg, i) => {
        leg.hinge.visible = g > 0.02;
        const ang = (1 - g) * Math.PI * 0.5 * leg.sign;
        leg.hinge.rotation.set(leg.axis === 'x' ? ang : 0, 0, leg.axis === 'z' ? ang : 0);
        leg.wheels.position.y = -leg.legLen + (s.compression?.[i] ?? 0);
      });
      const flapRad = (s.flapDeg ?? 0) * DEG;
      flaps.forEach((f) => (f.rotation.x = flapRad * 0.9));
      const spoiler = Math.max(s.spoiler ?? 0, (s.speedbrake ?? 0) * 0.6);
      spoilers.forEach((sp, i) => (sp.rotation.x = -spoiler * 45 * DEG * (i % 5 < 5 ? 1 : 0)));
      const beacon = Math.sin(t * 2 * Math.PI) > 0.7;
      beaconTop.visible = beaconBottom.visible = beacon;
      const strobe = (t % 1.2) < 0.06 && !s.onGround;
      strobes.forEach((st) => (st.visible = strobe));
    },
  };
}
