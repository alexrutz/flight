// Runway surfaces with ICAO markings, PAPI and approach/edge lights.
import * as THREE from 'three';
import { toScene, fromScene, FT, DEG } from './geo.js';
import { runwaysNear, runwayToLatLon } from './airports.js';
import { prepareRunway } from './elevation.js';

const PX_ALONG = 2.7; // texture pixels per meter along the runway
const SEGMENT = 750; // m per texture segment
const SURFACE_LIFT = 0.25; // m above the flattened terrain

// Pulls a surface slightly towards the camera in view space so it wins the
// depth test against coarse terrain without changing its screen position.
export function biasTowardsCamera(material, factor = 0.0015) {
  material.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader.replace(
      '#include <project_vertex>',
      `#include <project_vertex>
      mvPosition.xyz *= ${(1 - factor).toFixed(5)};
      gl_Position = projectionMatrix * mvPosition;`
    );
  };
  return material;
}

function stripes(width) {
  return width >= 58 ? 16 : width >= 43 ? 12 : width >= 28 ? 8 : width >= 22 ? 6 : 4;
}

function drawSegment(rw, s0, s1) {
  const W = rw.widthM;
  const pxW = 128;
  const ax = pxW / W; // px per meter across
  const H = Math.max(8, Math.round((s1 - s0) * PX_ALONG));
  const cv = document.createElement('canvas');
  cv.width = pxW;
  cv.height = H;
  const g = cv.getContext('2d');
  const X = (c) => (c + W / 2) * ax;
  const Y = (s) => (s1 - s) * PX_ALONG;
  // asphalt with texture noise
  g.fillStyle = '#3d4044';
  g.fillRect(0, 0, pxW, H);
  let seed = Math.floor(s0 * 7 + rw.lengthM);
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let i = 0; i < (H * pxW) / 40; i++) {
    g.fillStyle = rnd() > 0.5 ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.08)';
    g.fillRect(rnd() * pxW, rnd() * H, 2, 3);
  }
  const rect = (sA, sB, cA, cB, color = '#e9e9e4') => {
    const y0 = Math.min(Y(sA), Y(sB)), y1 = Math.max(Y(sA), Y(sB));
    if (y1 < 0 || y0 > H) return;
    g.fillStyle = color;
    g.fillRect(X(Math.min(cA, cB)), y0, Math.abs(cB - cA) * ax, y1 - y0);
  };
  // tire marks in both touchdown zones
  for (const end of [rw.le, rw.he]) {
    const dir = end === rw.le ? 1 : -1;
    for (let k = 0; k < 40; k++) {
      const d = 250 + rnd() * 450, w = 1 + rnd() * 2.5;
      for (const side of [-1, 1]) {
        const c = side * (3.8 + rnd() * 1.5);
        rect(end.thresholdS + dir * d, end.thresholdS + dir * (d + 20 + rnd() * 60), c - w / 2, c + w / 2, 'rgba(15,15,15,0.18)');
      }
    }
  }
  // edge lines and centerline
  if (W >= 29) {
    rect(s0, s1, -W / 2 + 0.3, -W / 2 + 1.2);
    rect(s0, s1, W / 2 - 1.2, W / 2 - 0.3);
  }
  const clW = W >= 43 ? 0.9 : 0.6;
  const first = rw.le.thresholdS + 70, last = rw.he.thresholdS - 70;
  for (let s = first; s < last; s += 50) rect(s, Math.min(s + 30, last), -clW / 2, clW / 2);

  for (const end of [rw.le, rw.he]) {
    const dir = end === rw.le ? 1 : -1;
    const at = (d) => end.thresholdS + dir * d;
    // threshold stripes
    const n = stripes(W), sw = 1.8, gap = (W - 6 - n * sw) / (n - 2 + 3);
    for (let k = 0; k < n / 2; k++) {
      const c0 = gap * 1.5 + k * (sw + gap);
      rect(at(6), at(36), c0, c0 + sw);
      rect(at(6), at(36), -c0, -c0 - sw);
    }
    // designator: letter nearer the threshold, number beyond
    const m = end.ident.match(/^(\d+)([LCR]?)$/);
    const number = m ? m[1].padStart(2, '0') : end.ident, letter = m ? m[2] : '';
    const drawText = (text, d0, d1) => {
      const yc = (Y(at(d0)) + Y(at(d1))) / 2;
      if (yc < -60 || yc > H + 60) return;
      g.save();
      g.translate(X(0), yc);
      if (dir < 0) g.rotate(Math.PI);
      g.scale(1, ((d1 - d0) * PX_ALONG) / (9 * ax) / 1.0);
      g.fillStyle = '#e9e9e4';
      g.font = `bold ${9 * ax}px Arial, sans-serif`;
      g.textAlign = 'center';
      g.textBaseline = 'middle';
      g.fillText(text, 0, 0);
      g.restore();
    };
    if (letter) {
      drawText(letter, 42, 51);
      drawText(number, 54, 63);
    } else drawText(number, 42, 51);
    // aiming point and touchdown zone
    const long = rw.lengthM >= 2400;
    const aim = long ? 400 : rw.lengthM >= 1500 ? 300 : 250;
    const aimW = long ? 9 : 6, aimL = long ? 55 : 45;
    rect(at(aim), at(aim + aimL), 9, 9 + aimW);
    rect(at(aim), at(aim + aimL), -9, -9 - aimW);
    const tdz = long ? [[150, 3], [300, 3], [450, 2], [600, 2], [750, 1], [900, 1]] : [[150, 2], [450, 1]];
    for (const [d, count] of tdz) {
      if (Math.abs(d - aim) < 60) continue;
      for (let k = 0; k < count; k++) {
        const c0 = 9 + k * 3.3;
        rect(at(d), at(d + 22.5), c0, c0 + 1.8);
        rect(at(d), at(d + 22.5), -c0, -c0 - 1.8);
      }
    }
    end.aimDist = aim;
  }
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
  return tex;
}

// Glow sprite for light points
let glowTex = null;
function glowTexture() {
  if (glowTex) return glowTex;
  const cv = document.createElement('canvas');
  cv.width = cv.height = 64;
  const g = cv.getContext('2d');
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.25, 'rgba(255,255,255,0.9)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  glowTex = new THREE.CanvasTexture(cv);
  return glowTex;
}

function lightPoints(positions, colors, size) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  const mat = biasTowardsCamera(
    new THREE.PointsMaterial({ size, map: glowTexture(), vertexColors: true, transparent: true, depthWrite: false, sizeAttenuation: false }),
    0.004
  );
  const pts = new THREE.Points(geo, mat);
  pts.renderOrder = 3;
  return pts;
}

class RunwayVisual {
  constructor(rw) {
    this.rw = rw;
    this.group = new THREE.Group();
    this.papis = [];
    this.build();
  }

  point(s, c, lift = SURFACE_LIFT) {
    const [lat, lon] = runwayToLatLon(this.rw, s, c);
    return toScene(lat, lon, this.rw.profile(s) + lift);
  }

  build() {
    const rw = this.rw, W = rw.widthM;
    for (let s0 = 0; s0 < rw.lengthM - 0.5; s0 += SEGMENT) {
      const s1 = Math.min(rw.lengthM, s0 + SEGMENT);
      const rows = Math.max(2, Math.ceil((s1 - s0) / 50) + 1);
      const pos = [], uv = [], idx = [];
      for (let r = 0; r < rows; r++) {
        const s = s0 + ((s1 - s0) * r) / (rows - 1);
        for (const [c, u] of [[-W / 2, 0], [W / 2, 1]]) {
          const p = this.point(s, c);
          pos.push(p.x, p.y, p.z);
          uv.push(u, (s - s0) / (s1 - s0));
        }
        if (r) {
          const a = (r - 1) * 2;
          idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
        }
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
      geo.setIndex(idx);
      geo.computeVertexNormals();
      const mat = biasTowardsCamera(new THREE.MeshLambertMaterial({ map: drawSegment(rw, s0, s1), side: THREE.DoubleSide }));
      const mesh = new THREE.Mesh(geo, mat);
      mesh.receiveShadow = true;
      mesh.renderOrder = 1;
      this.group.add(mesh);
    }
    this.buildLights();
  }

  buildLights() {
    const rw = this.rw, W = rw.widthM;
    const pos = [], col = [];
    const add = (p, r, g, b) => {
      pos.push(p.x, p.y, p.z);
      col.push(r, g, b);
    };
    // edge lights every 60 m
    for (let s = 0; s <= rw.lengthM; s += 60) for (const side of [-1, 1]) add(this.point(s, side * (W / 2 + 1.5), 0.5), 1, 0.95, 0.8);
    for (const end of [rw.le, rw.he]) {
      const dir = end === rw.le ? 1 : -1;
      // threshold (green) across the runway
      for (let c = -W / 2; c <= W / 2 + 0.1; c += 3) add(this.point(end.thresholdS, c, 0.4), 0.3, 1, 0.4);
      // approach lights: 900 m centerline with crossbars
      if (rw.lengthM > 1800) {
        for (let d = 30; d <= 900; d += 30) {
          for (const c of [-1.5, 0, 1.5]) add(this.point(end.s - dir * d, c, 1.0 + d * 0.004), 1, 1, 0.95);
          if (d === 300 || d === 150 || d === 450 || d === 600) for (let c = -15; c <= 15; c += 2.5) add(this.point(end.s - dir * d, c, 1.0 + d * 0.004), 1, 1, 0.95);
        }
      }
      // PAPI on the left of the landing direction
      const papiS = end.thresholdS + dir * (end.aimDist || 300);
      const unit = [];
      for (let k = 0; k < 4; k++) {
        const c = -dir * (W / 2 + 15 + k * 9);
        unit.push({ p: this.point(papiS, c, 1.0), angle: [3.5, 3.17, 2.83, 2.5][k] });
      }
      this.papis.push(unit);
    }
    this.lights = lightPoints(pos, col, 3.5);
    this.group.add(this.lights);
    const ppos = [], pcol = [];
    for (const u of this.papis) for (const l of u) {
      ppos.push(l.p.x, l.p.y, l.p.z);
      pcol.push(1, 1, 1);
    }
    this.papiPoints = lightPoints(ppos, pcol, 7);
    this.group.add(this.papiPoints);
  }

  update(cameraPos, lightScale) {
    // PAPI: white above the unit's angle, red below
    const colors = this.papiPoints.geometry.attributes.color;
    let i = 0;
    const cam = fromScene(cameraPos);
    for (const unit of this.papis) {
      for (const l of unit) {
        const lp = fromScene(l.p);
        const horiz = Math.hypot(l.p.x - cameraPos.x, l.p.z - cameraPos.z);
        const ang = Math.atan2(cam.h - lp.h, horiz) / DEG;
        if (ang > l.angle) colors.setXYZ(i, 1, 1, 1);
        else colors.setXYZ(i, 1, 0.1, 0.08);
        i++;
      }
    }
    colors.needsUpdate = true;
    this.papiPoints.material.size = 7 * lightScale;
    this.lights.material.size = 3.5 * lightScale;
  }

  dispose() {
    this.group.traverse((o) => {
      o.geometry?.dispose();
      if (o.material) {
        o.material.map?.dispose();
        o.material.dispose();
      }
    });
  }
}

export class Runways {
  constructor(scene) {
    this.scene = scene;
    this.group = new THREE.Group();
    scene.add(this.group);
    this.visuals = new Map();
    this.pending = new Set();
    this.lastScan = 0;
    this.lightScale = 1;
  }

  update(cameraPos, now) {
    if (now - this.lastScan > 2000) {
      this.lastScan = now;
      const cam = fromScene(cameraPos);
      const want = runwaysNear(cam.lat, cam.lon, 45000);
      for (const rw of want) {
        if (this.visuals.has(rw) || this.pending.has(rw)) continue;
        this.pending.add(rw);
        prepareRunway(rw).then(() => {
          this.pending.delete(rw);
          const v = new RunwayVisual(rw);
          this.visuals.set(rw, v);
          this.group.add(v.group);
        });
      }
      for (const [rw, v] of this.visuals) {
        if (!want.includes(rw)) {
          this.group.remove(v.group);
          v.dispose();
          this.visuals.delete(rw);
        }
      }
    }
    for (const v of this.visuals.values()) {
      if (v.group.position.distanceTo(cameraPos) < 20000 || true) v.update(cameraPos, this.lightScale);
    }
  }
}

export { FT };
