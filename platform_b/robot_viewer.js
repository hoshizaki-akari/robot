import * as THREE from "three";
import { STLLoader } from "/assets/vendor/STLLoader.js";
import { OrbitControls } from "/assets/vendor/OrbitControls.js";


const container = document.getElementById("robotViewer");
const status = document.getElementById("robotViewerStatus");
const jointText = document.getElementById("jointAngles");

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202832);

const camera = new THREE.PerspectiveCamera(36, 1, 0.01, 20);
// 从机械臂侧面观察，避免六个关节在画面中重叠成“竖直一根”。
camera.position.set(1.35, 0.85, 0.9);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.15, 0, 0.34);
controls.enableDamping = true;
controls.minDistance = 0.55;
controls.maxDistance = 3.5;

scene.add(new THREE.HemisphereLight(0xe8f2ff, 0x29313a, 2.2));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.8);
keyLight.position.set(1.5, -1.2, 2.0);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0x9bc7ff, 1.3);
fillLight.position.set(-1.2, 0.8, 1.0);
scene.add(fillLight);

const grid = new THREE.GridHelper(2.8, 24, 0x54708c, 0x354657);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.002;
scene.add(grid);

const robotRoot = new THREE.Group();
robotRoot.rotation.z = Math.PI;
robotRoot.position.set(0.25, 0, 0);
scene.add(robotRoot);

const loader = new STLLoader();
const robotMaterial = new THREE.MeshStandardMaterial({
  color: 0xdde5ea,
  roughness: 0.52,
  metalness: 0.12,
});
const jointMaterial = new THREE.MeshStandardMaterial({
  color: 0xaebbc4,
  roughness: 0.45,
  metalness: 0.18,
});
const adapterMaterial = new THREE.MeshStandardMaterial({
  color: 0x718096,
  roughness: 0.38,
  metalness: 0.5,
});
const sensorMaterial = new THREE.MeshStandardMaterial({
  color: 0x2f855a,
  roughness: 0.42,
  metalness: 0.25,
});
const gripperMaterial = new THREE.MeshStandardMaterial({
  color: 0x334155,
  roughness: 0.48,
  metalness: 0.3,
});
const fingerMaterial = new THREE.MeshStandardMaterial({
  color: 0xe2e8f0,
  roughness: 0.56,
  metalness: 0.15,
});

function addMesh(parent, name, material = robotMaterial) {
  return new Promise((resolve, reject) => {
    loader.load(
      `/assets/model/fairino5_v6/${name}.STL`,
      geometry => {
        geometry.computeVertexNormals();
        const mesh = new THREE.Mesh(geometry, material);
        parent.add(mesh);
        resolve(mesh);
      },
      undefined,
      reject,
    );
  });
}

function addJoint(parent, xyz, rpy) {
  const fixed = new THREE.Group();
  fixed.position.set(...xyz);
  fixed.rotation.set(...rpy, "XYZ");
  parent.add(fixed);
  const moving = new THREE.Group();
  fixed.add(moving);
  return moving;
}

const meshLoads = [addMesh(robotRoot, "base_link", jointMaterial)];
const joints = [];

let link = addJoint(robotRoot, [0, 0, 0], [0, 0, 0]);
joints.push(link);
meshLoads.push(addMesh(link, "shoulder_link", jointMaterial));

link = addJoint(link, [0, 0, 0.152], [Math.PI / 2, 0, 0]);
joints.push(link);
meshLoads.push(addMesh(link, "upperarm_link"));

link = addJoint(link, [-0.425, 0, 0], [0, 0, 0]);
joints.push(link);
meshLoads.push(addMesh(link, "forearm_link"));

link = addJoint(link, [-0.39501, 0, 0], [0, 0, 0]);
joints.push(link);
meshLoads.push(addMesh(link, "wrist1_link", jointMaterial));

link = addJoint(link, [0, 0, 0.1021], [Math.PI / 2, 0, 0]);
joints.push(link);
meshLoads.push(addMesh(link, "wrist2_link", jointMaterial));

link = addJoint(link, [0, 0, 0.102], [-Math.PI / 2, 0, 0]);
joints.push(link);
meshLoads.push(addMesh(link, "wrist3_link", jointMaterial));

const toolRoot = new THREE.Group();
link.add(toolRoot);
let leftFinger = null;
let rightFinger = null;
let tractionPoint = null;
let gripperConfig = null;
let latestOpeningRaw = 0;

const forceArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, 1),
  new THREE.Vector3(),
  0.1,
  0xef4444,
  0.035,
  0.018,
);
const movementArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, 1),
  new THREE.Vector3(),
  0.1,
  0x22c55e,
  0.035,
  0.018,
);
forceArrow.visible = false;
movementArrow.visible = false;
robotRoot.add(forceArrow, movementArrow);

function mm(value) {
  return Number(value) / 1000;
}

function addCylinder(parent, diameterMm, thicknessMm, material) {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(mm(diameterMm) / 2, mm(diameterMm) / 2, mm(thicknessMm), 48),
    material,
  );
  mesh.rotation.x = Math.PI / 2;
  parent.add(mesh);
  return mesh;
}

function buildEndEffector(config) {
  const model = config.model;
  toolRoot.position.z = mm(model.flange_face_offset_mm);
  let cursor = 0;

  for (const [part, material] of [
    [model.rear_adapter, adapterMaterial],
    [model.sensor, sensorMaterial],
    [model.front_adapter, adapterMaterial],
  ]) {
    const mesh = addCylinder(toolRoot, part.diameter_mm, part.thickness_mm, material);
    mesh.position.z = cursor + mm(part.thickness_mm) / 2;
    cursor += mm(part.thickness_mm);
  }

  gripperConfig = model.gripper;
  const bodyWidth = mm(gripperConfig.body_width_mm);
  const bodyLength = mm(gripperConfig.body_length_mm);
  const fingerLength = mm(gripperConfig.finger_length_mm);
  const fingerWidth = mm(gripperConfig.finger_width_mm);

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(bodyWidth, bodyWidth, bodyLength),
    gripperMaterial,
  );
  body.position.z = cursor + bodyLength / 2;
  toolRoot.add(body);

  const fingerGeometry = new THREE.BoxGeometry(fingerWidth, bodyWidth * 0.72, fingerLength);
  leftFinger = new THREE.Mesh(fingerGeometry, fingerMaterial);
  rightFinger = new THREE.Mesh(fingerGeometry, fingerMaterial);
  leftFinger.position.z = rightFinger.position.z = cursor + bodyLength + fingerLength / 2;
  toolRoot.add(leftFinger, rightFinger);

  tractionPoint = new THREE.Object3D();
  tractionPoint.position.z = cursor + bodyLength + fingerLength;
  toolRoot.add(tractionPoint);
  updateAG95(latestOpeningRaw);
  if (window.latestForceVector) {
    updateForceVector(window.latestForceVector, window.latestMovementVector || [0, 0, 0]);
  }
}

function updateAG95(positionRaw) {
  latestOpeningRaw = Number(positionRaw || 0);
  if (!leftFinger || !rightFinger || !gripperConfig) return;
  const rawMin = Number(gripperConfig.position_raw_min);
  const rawMax = Number(gripperConfig.position_raw_max);
  const ratio = THREE.MathUtils.clamp((latestOpeningRaw - rawMin) / (rawMax - rawMin), 0, 1);
  const opening = mm(gripperConfig.stroke_mm) * ratio;
  const halfOffset = (mm(gripperConfig.finger_width_mm) + opening) / 2;
  leftFinger.position.x = -halfOffset;
  rightFinger.position.x = halfOffset;
}

function placeArrow(arrow, vector, length) {
  if (!tractionPoint) return;
  const direction = new THREE.Vector3(...vector);
  if (direction.lengthSq() < 1e-10) {
    arrow.visible = false;
    return;
  }
  const origin = new THREE.Vector3();
  tractionPoint.getWorldPosition(origin);
  robotRoot.worldToLocal(origin);
  arrow.position.copy(origin);
  arrow.setDirection(direction.normalize());
  arrow.setLength(length, Math.min(length * 0.28, 0.045), Math.min(length * 0.15, 0.025));
  arrow.visible = true;
}

function updateForceVector(forceVector, movementVector = [0, 0, 0]) {
  window.latestForceVector = forceVector;
  window.latestMovementVector = movementVector;
  const forceLength = new THREE.Vector3(...forceVector).length();
  placeArrow(forceArrow, forceVector, Math.min(0.42, 0.05 + forceLength * 0.004));
  placeArrow(movementArrow, movementVector, 0.14);
}

function updateJoints(degrees) {
  if (!Array.isArray(degrees) || degrees.length !== 6) return;
  degrees.forEach((value, index) => {
    joints[index].rotation.z = THREE.MathUtils.degToRad(Number(value));
  });
  if (jointText) {
    jointText.textContent = degrees
      .map((value, index) => `J${index + 1} ${Number(value).toFixed(1)}°`)
      .join("   ");
  }
}

window.updateFR5Joints = updateJoints;
window.updateAG95 = updateAG95;
window.updateForceVector = updateForceVector;
if (window.latestFR5Joints) updateJoints(window.latestFR5Joints);

const endEffectorLoad = fetch("/assets/config.json", { cache: "no-store" })
  .then(response => {
    if (!response.ok) throw new Error(`配置读取失败：${response.status}`);
    return response.json();
  })
  .then(buildEndEffector);

Promise.all([...meshLoads, endEffectorLoad])
  .then(() => {
    if (status) {
      status.textContent = "三维模型已连接真实关节角和末端工具";
      status.classList.add("ready");
    }
  })
  .catch(error => {
    console.error("FR5 三维模型加载失败", error);
    if (status) {
      status.textContent = "三维模型加载失败";
      status.classList.add("error");
    }
  });

function resize() {
  const width = Math.max(container.clientWidth, 10);
  const height = Math.max(container.clientHeight, 10);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

const observer = new ResizeObserver(resize);
observer.observe(container);
resize();

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();
