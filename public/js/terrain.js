// Streaming level-of-detail terrain: a quadtree of web-mercator tiles, each a
// 32x32 grid textured with Sentinel-2 cloudless imagery and displaced by the
// Terrarium elevation data. Geometry is built in true ECEF geometry, so the
// earth curves away correctly towards the horizon.
import * as THREE from 'three';
import { toScene, toEcef, tileLat, tileLon, tileOf, fromScene, frame } from './geo.js';
import { loadElevationTile, makeSampler, prepareRunway, DETAIL_ZOOM } from './elevation.js';
import { runwaysNear } from './airports.js';

const GRID = 32;
const ROOT_ZOOM = 7;
const MAX_ZOOM = 15;
const SAT_MAX_ZOOM = 15;
const SPLIT = 2.4; // split when the camera is closer than SPLIT x tile size
const MAX_BUILDS_PER_FRAME = 3;
const MAX_LOADS = 10;

const texLoader = new THREE.ImageBitmapLoader();
texLoader.setOptions({ imageOrientation: 'flipY' });

export class Terrain {
  constructor(scene, renderer) {
    this.scene = scene;
    this.renderer = renderer;
    this.group = new THREE.Group();
    scene.add(this.group);
    this.nodes = new Map();
    this.loadQueue = [];
    this.loading = 0;
    this.buildQueue = [];
    this.frame = 0;
    this.maxAniso = renderer.capabilities.getMaxAnisotropy();
    this.receiveShadowZoom = 14;
    this.stats = { visible: 0, total: 0 };
  }

  node(z, x, y) {
    const key = `${z}/${x}/${y}`;
    let n = this.nodes.get(key);
    if (!n) {
      n = new TileNode(this, z, x, y);
      this.nodes.set(key, n);
    }
    return n;
  }

  // Selects and shows the tiles for the current camera position.
  update(cameraPos) {
    this.frame++;
    const cam = fromScene(cameraPos);
    const altitude = Math.max(10, cam.h);
    // Visible radius: horizon distance plus margin, capped.
    const radius = Math.min(450000, Math.sqrt(2 * 6371000 * altitude) + 80000);
    const roots = this.rootsAround(cam.lat, cam.lon, radius);
    const selected = [];
    for (const r of roots) this.select(r, cameraPos, selected);
    for (const n of selected) n.lastUsed = this.frame;
    // visibility
    let visible = 0;
    for (const n of this.nodes.values()) {
      const show = n.selectedFrame === this.frame && !!n.mesh;
      if (n.mesh) n.mesh.visible = show;
      if (show) visible++;
    }
    this.stats.visible = visible;
    this.stats.total = this.nodes.size;
    this.pumpLoads(cameraPos);
    this.pumpBuilds();
    if (this.frame % 120 === 0) this.evict();
  }

  rootsAround(lat, lon, radius) {
    const dLat = radius / 111000, dLon = radius / (111000 * Math.max(0.15, Math.cos((lat * Math.PI) / 180)));
    const latN = Math.min(84, lat + dLat), latS = Math.max(-84, lat - dLat);
    const [x0, y0] = tileOf(latN, lon - dLon, ROOT_ZOOM);
    const [x1, y1] = tileOf(latS, lon + dLon, ROOT_ZOOM);
    const n = 2 ** ROOT_ZOOM, out = [];
    for (let x = Math.floor(x0); x <= Math.floor(x1); x++)
      for (let y = Math.max(0, Math.floor(y0)); y <= Math.min(n - 1, Math.floor(y1)); y++)
        out.push(this.node(ROOT_ZOOM, ((x % n) + n) % n, y));
    return out;
  }

  select(node, cameraPos, out) {
    node.touch(cameraPos);
    const dist = Math.max(0, node.center.distanceTo(cameraPos) - node.radius * 0.5);
    const wantSplit = node.z < MAX_ZOOM && dist < node.size * SPLIT;
    if (wantSplit) {
      const kids = node.children();
      for (const k of kids) k.touch(cameraPos);
      if (kids.every((k) => k.ready)) {
        for (const k of kids) this.select(k, cameraPos, out);
        return;
      }
      for (const k of kids) k.request();
    }
    node.request();
    if (node.ready) {
      node.selectedFrame = this.frame;
      out.push(node);
    }
  }

  enqueueLoad(node) {
    if (!this.loadQueue.includes(node)) this.loadQueue.push(node);
  }

  pumpLoads(cameraPos) {
    if (!this.loadQueue.length) return;
    // nearest first, coarse levels first when far
    this.loadQueue.sort((a, b) => a.priority - b.priority);
    while (this.loading < MAX_LOADS && this.loadQueue.length) {
      const n = this.loadQueue.shift();
      if (n.state !== 'queued') continue;
      if (this.frame - n.lastTouched > 30) {
        n.state = 'idle'; // no longer wanted
        continue;
      }
      this.loading++;
      n.load().finally(() => this.loading--);
    }
  }

  pumpBuilds() {
    this.buildQueue.sort((a, b) => a.priority - b.priority);
    for (let i = 0; i < MAX_BUILDS_PER_FRAME && this.buildQueue.length; i++) this.buildQueue.shift().build();
  }

  evict() {
    for (const [key, n] of this.nodes) {
      if (n.z === ROOT_ZOOM) continue;
      if (this.frame - n.lastTouched > 600 && n.state !== 'loading') {
        n.dispose();
        this.nodes.delete(key);
        const parent = this.nodes.get(`${n.z - 1}/${n.x >> 1}/${n.y >> 1}`);
        if (parent?.kids?.includes(n)) parent.kids = null;
      }
    }
  }

  // Waits until the tiles under a location are loaded at full detail.
  async preload(lat, lon, cameraPos, maxFrames = 600) {
    for (let i = 0; i < maxFrames; i++) {
      this.update(cameraPos);
      if (this.loadQueue.length === 0 && this.loading === 0 && this.buildQueue.length === 0) return;
      await new Promise((r) => setTimeout(r, 16));
    }
  }

  dispose() {
    for (const n of this.nodes.values()) n.dispose();
    this.nodes.clear();
    this.scene.remove(this.group);
  }
}

class TileNode {
  constructor(terrain, z, x, y) {
    this.terrain = terrain;
    this.z = z;
    this.x = x;
    this.y = y;
    this.state = 'idle'; // idle | queued | loading | loaded | built | failed
    this.ready = false;
    this.mesh = null;
    this.kids = null;
    this.lastUsed = 0;
    this.lastTouched = 0;
    this.selectedFrame = -1;
    this.priority = 0;
    this.latN = tileLat(y, z);
    this.latS = tileLat(y + 1, z);
    this.lonW = tileLon(x, z);
    this.lonE = tileLon(x + 1, z);
    const latC = (this.latN + this.latS) / 2, lonC = (this.lonW + this.lonE) / 2;
    this.latC = latC;
    this.lonC = lonC;
    this.center = toScene(latC, lonC, 0);
    const corner = toScene(this.latN, this.lonW, 0);
    this.radius = corner.distanceTo(this.center) * 1.05;
    this.size = this.radius * 1.414;
  }

  children() {
    if (!this.kids) {
      const t = this.terrain, z = this.z + 1, x = this.x * 2, y = this.y * 2;
      this.kids = [t.node(z, x, y), t.node(z, x + 1, y), t.node(z, x, y + 1), t.node(z, x + 1, y + 1)];
    }
    return this.kids;
  }

  touch(cameraPos) {
    this.lastTouched = this.terrain.frame;
    this.priority = this.center.distanceTo(cameraPos) / this.size + this.z * 0.01;
  }

  request() {
    if (this.state === 'idle') {
      this.state = 'queued';
      this.terrain.enqueueLoad(this);
    }
  }

  async load() {
    this.state = 'loading';
    try {
      const ez = Math.min(this.z, DETAIL_ZOOM);
      const shift = this.z - ez;
      const elev = loadElevationTile(ez, this.x >> shift, this.y >> shift);
      const sz = Math.min(this.z, SAT_MAX_ZOOM);
      const sshift = this.z - sz;
      const tex = new Promise((resolve, reject) =>
        texLoader.load(`tiles/sat/${sz}/${this.x >> sshift}/${this.y >> sshift}.jpg`, resolve, undefined, reject)
      );
      // Runways inside detailed tiles must have their surface fitted first.
      const extra = [];
      if (this.z >= DETAIL_ZOOM - 1) {
        for (const rw of runwaysNear(this.latC, this.lonC, this.radius)) if (!rw.profile) extra.push(prepareRunway(rw));
      }
      const [, bitmap] = await Promise.all([elev, tex.catch(() => null), ...extra]);
      this.bitmap = bitmap;
      this.state = 'loaded';
      this.terrain.buildQueue.push(this);
    } catch (e) {
      this.state = 'failed';
    }
  }

  build() {
    if (this.state !== 'loaded') return;
    const terrain = this.terrain;
    const ez = Math.min(this.z, DETAIL_ZOOM);
    const sample = makeSampler(ez, this.latC, this.lonC, this.radius);
    const N = GRID, V = N + 1;
    const vertCount = V * V + 4 * V;
    const pos = new Float32Array(vertCount * 3);
    const uv = new Float32Array(vertCount * 2);
    const ecef = [0, 0, 0];
    const c = this.center;
    const p = new THREE.Vector3();
    // texture sub-rectangle when the imagery zoom is lower than the tile zoom
    const sz = Math.min(this.z, SAT_MAX_ZOOM), sshift = this.z - sz, scale = 1 / 2 ** sshift;
    const u0 = (this.x - ((this.x >> sshift) << sshift)) * scale, v0 = (this.y - ((this.y >> sshift) << sshift)) * scale;
    const heights = new Float32Array(V * V);
    for (let j = 0; j < V; j++) {
      const lat = tileLat(this.y + j / N, this.z);
      for (let i = 0; i < V; i++) {
        const lon = tileLon(this.x + i / N, this.z);
        const h = sample(lat, lon);
        heights[j * V + i] = h;
        toScene(lat, lon, h, p);
        const k = j * V + i;
        pos[k * 3] = p.x - c.x;
        pos[k * 3 + 1] = p.y - c.y;
        pos[k * 3 + 2] = p.z - c.z;
        uv[k * 2] = u0 + (i / N) * scale;
        uv[k * 2 + 1] = 1 - (v0 + (j / N) * scale);
      }
    }
    // Skirts: copies of the edge vertices pushed down to hide cracks between levels.
    const skirt = Math.max(20, this.size * 0.015);
    const edges = [];
    for (let i = 0; i < V; i++) edges.push(i); // north
    for (let i = 0; i < V; i++) edges.push(N * V + i); // south
    for (let j = 0; j < V; j++) edges.push(j * V); // west
    for (let j = 0; j < V; j++) edges.push(j * V + N); // east
    const up = toScene(this.latC, this.lonC, 1).sub(toScene(this.latC, this.lonC, 0)).normalize();
    edges.forEach((src, e) => {
      const k = V * V + e;
      pos[k * 3] = pos[src * 3] - up.x * skirt;
      pos[k * 3 + 1] = pos[src * 3 + 1] - up.y * skirt;
      pos[k * 3 + 2] = pos[src * 3 + 2] - up.z * skirt;
      uv[k * 2] = uv[src * 2];
      uv[k * 2 + 1] = uv[src * 2 + 1];
    });
    const idx = [];
    for (let j = 0; j < N; j++)
      for (let i = 0; i < N; i++) {
        const a = j * V + i, b = a + 1, d = a + V, e = d + 1;
        idx.push(a, d, b, b, d, e);
      }
    const strip = (start, reverse) => {
      for (let i = 0; i < N; i++) {
        const a = edges[start + i], b = edges[start + i + 1];
        const a2 = V * V + start + i, b2 = a2 + 1;
        if (reverse) idx.push(a, b, a2, b, b2, a2);
        else idx.push(a, a2, b, b, a2, b2);
      }
    };
    strip(0, false);
    strip(V, true);
    strip(2 * V, true);
    strip(3 * V, false);

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    geo.setIndex(idx);
    geo.computeVertexNormals();
    geo.computeBoundingSphere();

    let map = null;
    if (this.bitmap) {
      map = new THREE.Texture(this.bitmap);
      map.colorSpace = THREE.SRGBColorSpace;
      map.anisotropy = Math.min(8, terrain.maxAniso);
      map.generateMipmaps = true;
      map.minFilter = THREE.LinearMipmapLinearFilter;
      map.wrapS = map.wrapT = THREE.ClampToEdgeWrapping;
      map.needsUpdate = true;
      this.texture = map;
    }
    const mat = new THREE.MeshLambertMaterial({ map, color: map ? 0xffffff : 0x6b7d5a, side: THREE.DoubleSide });
    this.mesh = new THREE.Mesh(geo, mat);
    this.mesh.position.copy(c);
    this.mesh.matrixAutoUpdate = false;
    this.mesh.updateMatrix();
    this.mesh.receiveShadow = this.z >= terrain.receiveShadowZoom;
    this.mesh.visible = false;
    terrain.group.add(this.mesh);
    this.heights = heights;
    this.state = 'built';
    this.ready = true;
  }

  dispose() {
    if (this.mesh) {
      this.terrain.group.remove(this.mesh);
      this.mesh.geometry.dispose();
      this.mesh.material.dispose();
      this.texture?.dispose();
      this.bitmap?.close?.();
      this.mesh = null;
    }
    this.ready = false;
    this.state = 'idle';
    if (this.kids) this.kids = null;
  }
}

export { DETAIL_ZOOM, toEcef, frame };
