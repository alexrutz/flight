// Atmosphere: physically based sky dome, sun, ambient light and distance haze.
import * as THREE from 'three';
import { Sky } from 'three/addons/objects/Sky.js';

export const TIMES = {
  dawn: { elevation: 4, azimuth: 95 },
  morning: { elevation: 22, azimuth: 120 },
  noon: { elevation: 55, azimuth: 180 },
  afternoon: { elevation: 30, azimuth: 235 },
  dusk: { elevation: 3, azimuth: 265 },
};

export const VISIBILITY = { cavok: 80000, '10km': 10000, '5km': 5000, '1500m': 1500 };

export class Atmosphere {
  constructor(scene, renderer) {
    this.scene = scene;
    this.renderer = renderer;
    this.sky = new Sky();
    this.sky.scale.setScalar(900000);
    this.sky.frustumCulled = false;
    const u = this.sky.material.uniforms;
    u.turbidity.value = 4;
    u.rayleigh.value = 1.2;
    u.mieCoefficient.value = 0.004;
    u.mieDirectionalG.value = 0.8;
    scene.add(this.sky);

    this.sun = new THREE.DirectionalLight(0xffffff, 3.0);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = sc.bottom = -45;
    sc.right = sc.top = 45;
    sc.near = 1;
    sc.far = 600;
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.05;
    scene.add(this.sun, this.sun.target);
    this.hemi = new THREE.HemisphereLight(0xbfd8ff, 0x6a6050, 1.3);
    scene.add(this.hemi);
    this.sunDir = new THREE.Vector3();
    this.fogColor = new THREE.Color();
    scene.fog = new THREE.FogExp2(0xc8d8e8, 1e-5);
    this.set('noon', 'cavok');
  }

  set(time, visibility) {
    const t = TIMES[time] || TIMES.noon;
    this.time = time;
    const phi = THREE.MathUtils.degToRad(90 - t.elevation), theta = THREE.MathUtils.degToRad(t.azimuth);
    // azimuth measured from north (-z) clockwise towards east (+x)
    this.sunDir.setFromSphericalCoords(1, phi, theta);
    this.sunDir.set(Math.sin(theta) * Math.sin(phi), Math.cos(phi), -Math.cos(theta) * Math.sin(phi));
    this.sky.material.uniforms.sunPosition.value.copy(this.sunDir);
    const low = THREE.MathUtils.clamp((20 - t.elevation) / 20, 0, 1);
    // light colour warms and dims near the horizon
    this.sun.color.setRGB(1, 1 - 0.25 * low, 1 - 0.5 * low);
    this.sun.intensity = 0.6 + 2.6 * Math.min(1, t.elevation / 30);
    this.hemi.intensity = 0.5 + 0.9 * Math.min(1, t.elevation / 30);
    this.hemi.color.setRGB(0.75 - 0.2 * low, 0.85 - 0.3 * low, 1 - 0.4 * low);
    this.fogColor.setRGB(0.74 + 0.15 * low, 0.8 - 0.05 * low, 0.88 - 0.25 * low).multiplyScalar(0.55 + 0.45 * Math.min(1, t.elevation / 20));
    this.scene.fog.color.copy(this.fogColor);
    const vis = VISIBILITY[visibility] || VISIBILITY.cavok;
    this.visibility = vis;
    // FogExp2: ~95% extinction at the visibility distance
    this.scene.fog.density = 1.73 / vis;
    this.lightScale = 1 + 2 * low + (vis < 6000 ? 1.5 : 0);
    this.renderer.toneMappingExposure = 0.75 + 0.15 * low;
  }

  // Keeps the sun's shadow camera centred on the aircraft.
  update(focus) {
    this.sun.position.copy(focus).addScaledVector(this.sunDir, 300);
    this.sun.target.position.copy(focus);
    this.sky.position.copy(focus);
  }
}
