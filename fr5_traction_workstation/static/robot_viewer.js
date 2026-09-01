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
// The viewer shows the FR5 itself. The real sensor, flange and belt are not
// part of the robot body, so they are deliberately not drawn as a fake tool.
const tractionPoint = new THREE.Object3D();
toolRoot.add(tractionPoint);

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

function placeArrow(arrow, vector, length) {
  if (!tractionPoint) return;
  // The display root is rotated 180 degrees for the chosen camera view.  The
  // incoming force/motion vectors are in base_link, so convert them into the
  // model's local frame before applying the arrow direction.
  const direction = new THREE.Vector3(...vector).applyQuaternion(
    robotRoot.quaternion.clone().invert(),
  );
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
window.updateForceVector = updateForceVector;
if (window.latestFR5Joints) updateJoints(window.latestFR5Joints);

Promise.all(meshLoads)
  .then(() => {
    if (status) {
      status.textContent = "FR5模型已连接真实关节角";
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
