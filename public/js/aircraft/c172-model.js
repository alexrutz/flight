// Cessna 172 model (low-poly). Origin = CG, nose towards -z, dimensions from the JSBSim c172p.
import * as THREE from 'three';

// Cessna 172 style model. Origin = center of gravity, nose towards -z,
// dimensions taken from the JSBSim c172p geometry so the wheels touch the
// ground exactly where JSBSim's gear contacts are.
export function createC172() {
  const g = new THREE.Group();
  const mat = (color, extra = {}) => new THREE.MeshLambertMaterial({ color, flatShading: true, ...extra });
  const white = mat(0xf4f4f2), blue = mat(0x1f4e8c), red = mat(0xc8322b), dark = mat(0x2a2d33);
  const glass = mat(0x6fa8d6, { emissive: 0x0e2a44 });
  const add = (geo, m, x, y, z) => {
    const mesh = new THREE.Mesh(geo, m);
    mesh.position.set(x, y, z);
    g.add(mesh);
    return mesh;
  };
  const box = (w, h, d, m, x, y, z) => add(new THREE.BoxGeometry(w, h, d), m, x, y, z);
  const rod = (a, b, r, m) => {
    const va = new THREE.Vector3(...a), vb = new THREE.Vector3(...b);
    const geo = new THREE.CylinderGeometry(r, r, va.distanceTo(vb), 6);
    geo.rotateX(Math.PI / 2);
    const mesh = add(geo, m, ...va.clone().add(vb).multiplyScalar(0.5).toArray());
    mesh.lookAt(vb.applyMatrix4(g.matrixWorld));
    return mesh;
  };

  box(1.1, 1.35, 2.6, white, 0, -0.07, 0); // cabin
  box(1.12, 0.42, 1.7, glass, 0, 0.3, -0.15); // side windows
  box(1.0, 0.8, 0.85, white, 0, -0.3, -1.72); // engine cowling (top below the pilot's eye)
  box(1.13, 0.12, 2.62, blue, 0, -0.45, 0); // stripe
  const wind = box(1.0, 0.6, 0.06, glass, 0, 0.32, -1.25); // windshield
  wind.rotation.x = -0.9;
  const cone = new THREE.CylinderGeometry(0.18, 0.7, 2.7, 4, 1);
  cone.rotateY(Math.PI / 4);
  cone.rotateX(Math.PI / 2);
  cone.scale(1, 1.25, 1);
  add(cone, white, 0, 0.0, 2.64); // tail cone
  const wingParts = [
    box(10.9, 0.14, 1.5, white, 0, 0.64, 0.05), // wing
    box(0.6, 0.15, 1.52, red, 5.15, 0.64, 0.05), // wing tips
    box(0.6, 0.15, 1.52, red, -5.15, 0.64, 0.05),
  ];
  box(3.4, 0.08, 0.85, white, 0, 0.05, 3.85); // horizontal stabilizer
  const fin = box(0.08, 1.3, 1.0, white, 0, 0.75, 3.85);
  fin.rotation.x = 0.4;
  const finStripe = box(0.1, 0.25, 0.9, red, 0, 1.1, 4.0);
  finStripe.rotation.x = 0.4;
  wingParts.push(
    rod([0.5, -0.55, -0.1], [2.5, 0.58, 0.0], 0.035, white), // wing struts
    rod([-0.5, -0.55, -0.1], [-2.5, 0.58, 0.0], 0.035, white)
  );

  // Landing gear (wheel radius 0.2 m)
  const wheelGeo = new THREE.CylinderGeometry(0.2, 0.2, 0.14, 12);
  wheelGeo.rotateZ(Math.PI / 2);
  for (const side of [-1, 1]) {
    add(wheelGeo, dark, side * 1.09, -1.12, 0.44);
    rod([side * 0.4, -0.72, 0.35], [side * 1.05, -1.12, 0.44], 0.04, white);
  }
  add(wheelGeo, dark, 0, -1.22, -1.21);
  rod([0, -0.62, -1.45], [0, -1.22, -1.21], 0.04, dark);

  // Propeller: blades at low rpm, a translucent disc when spinning fast.
  const prop = new THREE.Group();
  prop.position.set(0, -0.3, -2.25);
  g.add(prop);
  const spinner = new THREE.Mesh(new THREE.ConeGeometry(0.2, 0.4, 10), white);
  spinner.rotation.x = -Math.PI / 2;
  prop.add(spinner);
  const blades = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.9, 0.04), dark);
  prop.add(blades);
  const disc = new THREE.Mesh(
    new THREE.CircleGeometry(0.95, 32),
    new THREE.MeshBasicMaterial({ color: 0x222222, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false })
  );
  prop.add(disc);

  g.traverse((o) => o.isMesh && (o.castShadow = true));
  return {
    group: g,
    eyepoint: new THREE.Vector3(-0.3, 0.29, -0.1),
    length: 8.3,
    // Pilot's-eye view: only the high wing stays visible overhead.
    setCockpit(on) {
      for (const child of g.children) child.visible = !on || wingParts.includes(child);
    },
    update(s, dt) {
      const rpm = s.rpm ?? 0;
      blades.rotation.z += (rpm / 60) * Math.PI * 2 * dt;
      const fast = rpm > 500;
      blades.visible = !fast;
      disc.visible = fast;
    },
  };
}

