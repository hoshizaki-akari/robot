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

// The flange face is approximately 99 mm along wrist3 +Z. The arrow is kept
// in the robot base frame, starts at that face and has a fixed visual length;
// only its direction changes with the direction locked by the traction logic.
const flangeAnchor = new THREE.Object3D();
flangeAnchor.position.set(0, 0, 0.099);
link.add(flangeAnchor);
const tractionDirection = new THREE.Vector3(1, 0, 0);
const tractionArrowColor = 0x0b3b8f;
const tractionArrow = new THREE.ArrowHelper(
  tractionDirection,
  new THREE.Vector3(),
  0.24,
  tractionArrowColor,
  0.064,
  0.046,
);
// WebGL implementations commonly ignore lineWidth on line primitives.  Hide
// ArrowHelper's thin shaft and replace it with a real cylinder so the traction
// direction remains clearly visible on the workstation display.
tractionArrow.line.visible = false;
const tractionShaft = new THREE.Mesh(
  new THREE.CylinderGeometry(0.0095, 0.0095, 0.176, 18),
  new THREE.MeshBasicMaterial({ color: tractionArrowColor, depthTest: false }),
);
tractionShaft.position.y = 0.088;
tractionShaft.renderOrder = 20;
tractionArrow.add(tractionShaft);
tractionArrow.line.material.depthTest = false;
tractionArrow.cone.material.depthTest = false;
tractionArrow.renderOrder = 20;
robotRoot.add(tractionArrow);
const flangeWorld = new THREE.Vector3();
const flangeInBase = new THREE.Vector3();

function validDirection(candidate) {
  if (!Array.isArray(candidate) || candidate.length !== 3) return null;
  const vector = new THREE.Vector3(...candidate.map(Number));
  if (![vector.x, vector.y, vector.z].every(Number.isFinite) || vector.lengthSq() < 0.25) {
    return null;
  }
  return vector.normalize();
}

window.updateTractionDirection = (lockedDirection, fallbackDirection) => {
  const nextDirection = validDirection(lockedDirection) || validDirection(fallbackDirection);
  if (nextDirection) {
    // The FR5 model root is rotated 180 degrees around Z for STL alignment.
    // Compensate that visual-only rotation: X/Y reverse while Z is unchanged.
    tractionDirection.set(-nextDirection.x, -nextDirection.y, nextDirection.z).normalize();
  }
};

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
  scene.updateMatrixWorld(true);
  flangeAnchor.getWorldPosition(flangeWorld);
  flangeInBase.copy(flangeWorld);
  robotRoot.worldToLocal(flangeInBase);
  tractionArrow.position.copy(flangeInBase);
  tractionArrow.setDirection(tractionDirection);
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();
