import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import URDFLoader from "urdf-loader";

const dom = {
  sourcePath: document.getElementById("source-path"),
  urdfPath: document.getElementById("urdf-path"),
  frameCount: document.getElementById("frame-count"),
  frameLabel: document.getElementById("frame-label"),
  frameSlider: document.getElementById("frame-slider"),
  playToggle: document.getElementById("play-toggle"),
  prevFrame: document.getElementById("prev-frame"),
  nextFrame: document.getElementById("next-frame"),
  resetCamera: document.getElementById("reset-camera"),
  fpsInput: document.getElementById("fps-input"),
  gripperScale: document.getElementById("gripper-scale"),
  gripperOffset: document.getElementById("gripper-offset"),
  gripperHint: document.getElementById("gripper-hint"),
  chassisDt: document.getElementById("chassis-dt"),
  chassisLinearScale: document.getElementById("chassis-linear-scale"),
  chassisAngularScale: document.getElementById("chassis-angular-scale"),
  chassisHint: document.getElementById("chassis-hint"),
  frameInspector: document.getElementById("frame-inspector"),
  viewport: document.getElementById("viewport"),
};

const state = {
  payload: null,
  robot: null,
  robotRoot: null,
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  currentFrame: 0,
  isPlaying: false,
  lastTickMs: 0,
  chassisPoses: [],
  chassisPath: null,
};

function clamp(value, lower, upper) {
  return Math.max(lower, Math.min(upper, value));
}

function getGripperOpening(rawValue) {
  const config = state.payload.gripperConfig;
  const scale = Number(dom.gripperScale.value);
  const offset = Number(dom.gripperOffset.value);
  return clamp((rawValue * scale) + offset, 0, config.maxOpening);
}

function applyJointValue(jointName, value) {
  const joint = state.robot?.joints?.[jointName];
  if (!joint) {
    return;
  }
  joint.setJointValue(value);
}

function computeChassisPoses() {
  const config = state.payload.chassisConfig;
  if (!config?.enabled) {
    state.chassisPoses = [];
    return;
  }

  const dt = Number(dom.chassisDt.value) || config.defaultDt;
  const linearScale = Number(dom.chassisLinearScale.value) || config.defaultLinearScale;
  const angularScale = Number(dom.chassisAngularScale.value) || config.defaultAngularScale;

  let x = 0;
  let z = 0;
  let yaw = 0;

  state.chassisPoses = state.payload.frames.map((frame) => {
    const cmd = frame.chassis ?? [0, 0, 0];
    const pose = {
      x,
      z,
      yaw,
      raw: cmd,
    };

    const [vx = 0, vy = 0, wz = 0] = cmd;
    const worldX = (vx * Math.cos(yaw) - vy * Math.sin(yaw)) * dt * linearScale;
    const worldZ = (vx * Math.sin(yaw) + vy * Math.cos(yaw)) * dt * linearScale;
    x += worldX;
    z += worldZ;
    yaw += wz * dt * angularScale;
    return pose;
  });
}

function updateChassisPath() {
  if (state.chassisPath) {
    state.scene.remove(state.chassisPath);
    state.chassisPath.geometry.dispose();
    state.chassisPath.material.dispose();
    state.chassisPath = null;
  }

  if (!state.chassisPoses.length) {
    return;
  }

  const points = state.chassisPoses.map((pose) => new THREE.Vector3(pose.x, 0.005, pose.z));
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = new THREE.LineBasicMaterial({ color: 0x7bd4ff });
  state.chassisPath = new THREE.Line(geometry, material);
  state.scene.add(state.chassisPath);
}

function applyFrame(frameIndex) {
  if (!state.payload || !state.robot) {
    return;
  }

  const frame = state.payload.frames[frameIndex];
  const mapping = state.payload.mapping;

  if (frame.left_arm) {
    mapping.left_arm.forEach((jointName, idx) => applyJointValue(jointName, frame.left_arm[idx]));
  }
  if (frame.right_arm) {
    mapping.right_arm.forEach((jointName, idx) => applyJointValue(jointName, frame.right_arm[idx]));
  }
  if (frame.torso) {
    mapping.torso.forEach((jointName, idx) => applyJointValue(jointName, frame.torso[idx]));
  }

  const leftOpening = frame.left_gripper ? getGripperOpening(frame.left_gripper[0]) : 0;
  mapping.left_gripper.joints.forEach(({ name, direction }) => applyJointValue(name, leftOpening * direction));

  const rightOpening = frame.right_gripper ? getGripperOpening(frame.right_gripper[0]) : 0;
  mapping.right_gripper.joints.forEach(({ name, direction }) => applyJointValue(name, rightOpening * direction));

  const chassisPose = state.chassisPoses[frameIndex] ?? { x: 0, z: 0, yaw: 0, raw: frame.chassis ?? [0, 0, 0] };
  if (state.robotRoot) {
    state.robotRoot.position.set(chassisPose.x, 0, chassisPose.z);
    state.robotRoot.rotation.y = chassisPose.yaw;
  }

  state.currentFrame = frameIndex;
  dom.frameSlider.value = String(frameIndex);
  dom.frameLabel.textContent = `${frameIndex + 1} / ${state.payload.frameCount}`;

  const inspector = {
    rawFrame: frame,
    applied: {
      left_arm: frame.left_arm
        ? Object.fromEntries(mapping.left_arm.map((name, idx) => [name, frame.left_arm[idx]]))
        : null,
      right_arm: frame.right_arm
        ? Object.fromEntries(mapping.right_arm.map((name, idx) => [name, frame.right_arm[idx]]))
        : null,
      torso: frame.torso
        ? Object.fromEntries(mapping.torso.map((name, idx) => [name, frame.torso[idx]]))
        : null,
      left_gripper: {
        raw: frame.left_gripper ? frame.left_gripper[0] : null,
        opening: leftOpening,
        joints: Object.fromEntries(mapping.left_gripper.joints.map(({ name, direction }) => [name, leftOpening * direction])),
      },
      right_gripper: {
        raw: frame.right_gripper ? frame.right_gripper[0] : null,
        opening: rightOpening,
        joints: Object.fromEntries(mapping.right_gripper.joints.map(({ name, direction }) => [name, rightOpening * direction])),
      },
      chassis: {
        raw: frame.chassis ?? null,
        pose: chassisPose,
      },
    },
  };

  dom.frameInspector.textContent = JSON.stringify(inspector, null, 2);
}

function resizeRenderer() {
  const width = dom.viewport.clientWidth;
  const height = dom.viewport.clientHeight;
  state.camera.aspect = width / height;
  state.camera.updateProjectionMatrix();
  state.renderer.setSize(width, height, false);
}

function animate(nowMs) {
  requestAnimationFrame(animate);

  if (state.isPlaying && state.payload) {
    const fps = Math.max(1, Number(dom.fpsInput.value) || 15);
    const frameInterval = 1000 / fps;
    if (nowMs - state.lastTickMs >= frameInterval) {
      const nextFrame = (state.currentFrame + 1) % state.payload.frameCount;
      applyFrame(nextFrame);
      state.lastTickMs = nowMs;
    }
  }

  state.controls.update();
  state.renderer.render(state.scene, state.camera);
}

async function loadPayload() {
  const response = await fetch("/api/data");
  if (!response.ok) {
    throw new Error(`Failed to fetch payload: ${response.status}`);
  }
  state.payload = await response.json();
  dom.sourcePath.textContent = state.payload.source.actionsPath;
  dom.urdfPath.textContent = state.payload.source.urdfPath;
  dom.frameCount.textContent = String(state.payload.frameCount);
  dom.frameSlider.max = String(state.payload.frameCount - 1);
  dom.gripperScale.value = String(state.payload.gripperConfig.defaultScale);
  dom.gripperOffset.value = String(state.payload.gripperConfig.defaultOffset);
  dom.gripperHint.textContent = state.payload.gripperConfig.modeHint;
  dom.chassisDt.value = String(state.payload.chassisConfig.defaultDt);
  dom.chassisLinearScale.value = String(state.payload.chassisConfig.defaultLinearScale);
  dom.chassisAngularScale.value = String(state.payload.chassisConfig.defaultAngularScale);
  dom.chassisHint.textContent = state.payload.chassisConfig.modeHint;
}

function initScene() {
  state.scene = new THREE.Scene();
  state.scene.background = new THREE.Color(0x0b0d12);

  state.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  state.camera.position.set(1.8, 1.4, 1.9);

  state.renderer = new THREE.WebGLRenderer({ antialias: true });
  state.renderer.setPixelRatio(window.devicePixelRatio);
  dom.viewport.appendChild(state.renderer.domElement);

  state.controls = new OrbitControls(state.camera, state.renderer.domElement);
  state.controls.target.set(0, 0.75, 0);

  const hemi = new THREE.HemisphereLight(0xffffff, 0x223344, 2.0);
  state.scene.add(hemi);

  const dir = new THREE.DirectionalLight(0xffffff, 1.7);
  dir.position.set(2, 3, 2);
  state.scene.add(dir);

  const grid = new THREE.GridHelper(4, 24, 0x5a708f, 0x283242);
  state.scene.add(grid);

  const axes = new THREE.AxesHelper(0.4);
  state.scene.add(axes);

  resizeRenderer();
  window.addEventListener("resize", resizeRenderer);
}

async function loadRobot() {
  return new Promise((resolve, reject) => {
    const loader = new URDFLoader();
    loader.loadMeshCb = (path, manager, done) => {
      if (path.toLowerCase().endsWith(".obj")) {
        const objLoader = new OBJLoader(manager);
        objLoader.load(path, (obj) => done(obj), undefined, (error) => done(null, error));
        return;
      }
      done(null, new Error(`Unsupported mesh type for ${path}`));
    };
    loader.load(
      state.payload.robot.urdfPath,
      (robot) => {
        robot.rotation.x = -Math.PI / 2;
        const robotRoot = new THREE.Group();
        robotRoot.add(robot);
        state.robotRoot = robotRoot;
        state.robot = robot;
        state.scene.add(robotRoot);
        resolve(robot);
      },
      undefined,
      reject,
    );
  });
}

function bindEvents() {
  dom.playToggle.addEventListener("click", () => {
    state.isPlaying = !state.isPlaying;
    dom.playToggle.textContent = state.isPlaying ? "暂停" : "播放";
    state.lastTickMs = performance.now();
  });

  dom.prevFrame.addEventListener("click", () => {
    const prev = Math.max(0, state.currentFrame - 1);
    applyFrame(prev);
  });

  dom.nextFrame.addEventListener("click", () => {
    const next = Math.min(state.payload.frameCount - 1, state.currentFrame + 1);
    applyFrame(next);
  });

  dom.resetCamera.addEventListener("click", () => {
    state.camera.position.set(1.8, 1.4, 1.9);
    state.controls.target.set(0, 0.75, 0);
    state.controls.update();
  });

  dom.frameSlider.addEventListener("input", (event) => {
    applyFrame(Number(event.target.value));
  });

  dom.gripperScale.addEventListener("change", () => applyFrame(state.currentFrame));
  dom.gripperOffset.addEventListener("change", () => applyFrame(state.currentFrame));

  const refreshChassis = () => {
    computeChassisPoses();
    updateChassisPath();
    applyFrame(state.currentFrame);
  };
  dom.chassisDt.addEventListener("change", refreshChassis);
  dom.chassisLinearScale.addEventListener("change", refreshChassis);
  dom.chassisAngularScale.addEventListener("change", refreshChassis);
}

async function main() {
  try {
    initScene();
    bindEvents();
    await loadPayload();
    await loadRobot();
    computeChassisPoses();
    updateChassisPath();
    applyFrame(0);
    animate(0);
  } catch (error) {
    console.error(error);
    dom.frameInspector.textContent = String(error);
  }
}

main();
