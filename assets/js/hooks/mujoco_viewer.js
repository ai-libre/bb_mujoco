/**
 * MuJoCo Viewer Hook
 *
 * Connects browser MuJoCo WASM to Elixir backend via Phoenix Channel.
 * Acts as a "physics server" that Elixir commands remotely.
 *
 * Architecture:
 * - MuJoCo WASM runs physics in browser
 * - Phoenix Channel receives commands from Elixir (BB.Mujoco.Bridge)
 * - This hook processes commands and returns physics state
 * - Three.js renders the scene
 *
 * Commands (Elixir → Browser):
 * - set_joints: Set target joint positions
 * - get_joints: Get current joint positions
 * - get_velocities: Get current joint velocities
 * - step: Advance physics by dt seconds
 * - reset: Reset simulation to initial state
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Socket } from 'phoenix';

// ============================================================================
// MuJoCo Utilities (coordinate swizzle: MuJoCo Z-up → Three.js Y-up)
// ============================================================================

function getPosition(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      buffer[(index * 3) + 0],
      buffer[(index * 3) + 2],
      -buffer[(index * 3) + 1]
    );
  }
  return target.set(
    buffer[(index * 3) + 0],
    buffer[(index * 3) + 1],
    buffer[(index * 3) + 2]
  );
}

function getQuaternion(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      -buffer[(index * 4) + 1],
      -buffer[(index * 4) + 3],
      buffer[(index * 4) + 2],
      -buffer[(index * 4) + 0]
    );
  }
  return target.set(
    buffer[(index * 4) + 0],
    buffer[(index * 4) + 1],
    buffer[(index * 4) + 2],
    buffer[(index * 4) + 3]
  );
}

// ============================================================================
// MuJoCo Viewer Hook
// ============================================================================

const MujocoViewer = {
  // MuJoCo state
  mujoco: null,
  model: null,
  data: null,

  // Three.js state
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  bodies: {},
  meshes: {},
  mujocoRoot: null,

  // Channel state
  physicsSocket: null,
  physicsChannel: null,

  // Config
  robot: null,

  // Note: Physics stepping is now controlled by Elixir Simulation GenServer
  // Browser only responds to Channel commands (step, step_with_targets, etc.)

  // ============================================================================
  // Lifecycle
  // ============================================================================

  async mounted() {
    this.robot = this.el.dataset.robot;
    console.log('[MujocoViewer] Mounting for robot:', this.robot);

    // Register camera handler (visualization-only LiveView events)
    this.setupCameraHandler();

    try {
      // Initialize Three.js
      this.initThreeJS();

      // Initialize MuJoCo WASM
      await this.initMuJoCo();

      // Build scene from model
      await this.buildSceneFromMuJoCo();

      // Connect to physics channel
      this.connectPhysicsChannel();

      // Start render loop
      this.startRenderLoop();

      // Notify LiveView with joint info
      this.pushEvent('mujoco_status', {
        status: 'ready',
        nbody: this.model.nbody,
        njnt: this.model.njnt,
        nq: this.model.nq,
        nv: this.model.nv,
        nu: this.model.nu,
        joint_names: this.getJointNames(),
        joint_ranges: this.getJointRanges()
      });

      // Physics stepping is now controlled by Elixir Simulation GenServer
      // via Channel commands (step_with_targets at 50Hz)

      console.log('[MujocoViewer] Initialization complete');
    } catch (error) {
      console.error('[MujocoViewer] Initialization failed:', error);
      this.pushEvent('mujoco_status', {
        status: 'error',
        message: error.message
      });
    }
  },

  destroyed() {
    console.log('[MujocoViewer] Destroying hook');
    this.cleanup();
  },

  // ============================================================================
  // MuJoCo Initialization
  // ============================================================================

  async initMuJoCo() {
    console.log('[MujocoViewer] Loading MuJoCo WASM...');

    // Dynamic import of MuJoCo WASM from vendor
    const { default: load_mujoco } = await import('/vendor/mujoco_wasm.js');
    this.mujoco = await load_mujoco();

    console.log('[MujocoViewer] MuJoCo WASM loaded');

    // Set up virtual filesystem
    this.mujoco.FS.mkdir('/working');
    this.mujoco.FS.mount(this.mujoco.MEMFS, { root: '.' }, '/working');
    this.mujoco.FS.mkdir('/working/assets');

    // Download MJCF from server
    await this.downloadMJCF();

    // Load model
    console.log('[MujocoViewer] Loading MuJoCo model...');
    this.model = this.mujoco.MjModel.loadFromXML('/working/scene.xml');
    if (!this.model) {
      throw new Error('Failed to load MuJoCo model');
    }

    this.data = new this.mujoco.MjData(this.model);
    if (!this.data) {
      throw new Error('Failed to create MuJoCo data');
    }

    console.log('[MujocoViewer] Model loaded:', {
      nbody: this.model.nbody,
      njnt: this.model.njnt,
      ngeom: this.model.ngeom,
      nq: this.model.nq,
      nv: this.model.nv,
      nu: this.model.nu
    });

    // Expose for debugging
    window.mjModel = this.model;
    window.mjData = this.data;
  },

  async downloadMJCF() {
    console.log('[MujocoViewer] Downloading MJCF...');

    // Get MJCF from API
    const response = await fetch(`/api/mujoco/mjcf/${this.robot}`);
    if (!response.ok) {
      throw new Error(`Failed to fetch MJCF: ${response.statusText}`);
    }
    const mjcf = await response.text();
    this.mujoco.FS.writeFile('/working/scene.xml', mjcf);

    // Get assets list and download
    try {
      const assetsResponse = await fetch(`/api/mujoco/mjcf/${this.robot}/assets`);
      if (assetsResponse.ok) {
        const assetList = await assetsResponse.json();
        console.log('[MujocoViewer] Assets to download:', assetList);
        for (const asset of assetList.assets || []) {
          const assetResponse = await fetch(`/api/mujoco/mjcf/${this.robot}/assets/${asset}`);
          if (assetResponse.ok) {
            const data = new Uint8Array(await assetResponse.arrayBuffer());
            const path = `/working/assets/${asset}`;
            console.log(`[MujocoViewer] Writing asset: ${path} (${data.length} bytes)`);
            this.mujoco.FS.writeFile(path, data);
          }
        }
        // Debug: list files in /working/assets
        console.log('[MujocoViewer] Files in /working/assets:', this.mujoco.FS.readdir('/working/assets'));
      }
    } catch (e) {
      console.warn('[MujocoViewer] No assets to download:', e.message);
    }
  },

  // ============================================================================
  // Phoenix Channel Connection
  // ============================================================================

  connectPhysicsChannel() {
    console.log('[MujocoViewer] Connecting to physics channel...');

    const socketPath = this.el.dataset.socketPath || '/mujoco';
    this.physicsSocket = new Socket(socketPath, {});
    this.physicsSocket.connect();

    this.physicsChannel = this.physicsSocket.channel(`mujoco:physics:${this.robot}`, {});

    // Handle physics commands from Elixir
    this.physicsChannel.on("physics:command", (payload) => {
      this.handlePhysicsCommand(payload);
    });

    this.physicsChannel.join()
      .receive("ok", () => {
        console.log('[MujocoViewer] Joined physics channel');
        this.physicsChannel.push("physics:ready", {
          nq: this.model.nq,
          nv: this.model.nv,
          nu: this.model.nu
        });
      })
      .receive("error", (resp) => {
        console.error('[MujocoViewer] Failed to join channel:', resp);
      });
  },

  handlePhysicsCommand(payload) {
    const { cmd, id, params } = payload;
    let response = { id, status: "ok" };

    try {
      switch (cmd) {
        case "set_joints":
          this.setJoints(params.joints);
          break;

        case "get_joints":
          response.joints = this.getJoints();
          break;

        case "get_velocities":
          response.velocities = this.getVelocities();
          break;

        case "step":
          const stepResult = this.step(params.dt);
          response = { ...response, ...stepResult };
          break;

        case "step_with_targets":
          // Debug: log every 50th step (1 second)
          if (id % 50 === 0) {
            console.log(`[MujocoViewer] step_with_targets: targets=${JSON.stringify(params.targets)}, ctrl before=${JSON.stringify(Array.from(this.data.ctrl).slice(0, this.model.nu))}`);
          }
          this.setJoints(params.targets);
          const result = this.step(params.dt);
          if (id % 50 === 0) {
            console.log(`[MujocoViewer] after step: ctrl=${JSON.stringify(Array.from(this.data.ctrl).slice(0, this.model.nu))}, qpos=${JSON.stringify(result.joints)}`);
          }
          response = { ...response, ...result };
          break;

        case "reset":
          this.reset();
          break;

        case "get_state":
          response.joints = this.getJoints();
          response.velocities = this.getVelocities();
          response.time = this.data.time;
          break;

        default:
          response.status = "error";
          response.message = `Unknown command: ${cmd}`;
      }
    } catch (error) {
      response.status = "error";
      response.message = error.message;
      console.error('[MujocoViewer] Command error:', error);
    }

    this.physicsChannel.push("physics:response", response);
  },

  // ============================================================================
  // Physics Operations
  // ============================================================================

  setJoints(joints) {
    const nu = this.model.nu;
    for (let i = 0; i < Math.min(joints.length, nu); i++) {
      this.data.ctrl[i] = joints[i];
    }
  },

  getJoints() {
    const nu = this.model.nu;
    const joints = [];

    for (let i = 0; i < nu; i++) {
      const jointId = this.model.actuator_trnid[i * 2];
      const qposAdr = this.model.jnt_qposadr[jointId];
      joints.push(this.data.qpos[qposAdr]);
    }

    return joints;
  },

  getVelocities() {
    const nu = this.model.nu;
    const velocities = [];

    for (let i = 0; i < nu; i++) {
      const jointId = this.model.actuator_trnid[i * 2];
      const qvelAdr = this.model.jnt_dofadr[jointId];
      velocities.push(this.data.qvel[qvelAdr]);
    }

    return velocities;
  },

  step(dt) {
    const numSteps = Math.max(1, Math.round(dt / this.model.opt.timestep));

    for (let i = 0; i < numSteps; i++) {
      this.mujoco.mj_step(this.model, this.data);
    }

    this.updateBodyTransforms();

    return {
      joints: this.getJoints(),
      velocities: this.getVelocities(),
      time: this.data.time
    };
  },

  reset() {
    this.mujoco.mj_resetData(this.model, this.data);
    this.mujoco.mj_forward(this.model, this.data);
    this.updateBodyTransforms();
  },

  // ============================================================================
  // Three.js Visualization
  // ============================================================================

  initThreeJS() {
    const container = this.el;
    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1e293b);
    this.scene.fog = new THREE.Fog(0x1e293b, 15, 25);

    // Camera
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.001, 100);
    this.camera.position.set(0.8, 0.5, 0.8);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(width, height);
    this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    // Controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.3, 0);
    this.controls.enableDamping = true;
    this.controls.update();

    // Lighting
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.4));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(2, 4, 2);
    dirLight.castShadow = true;
    this.scene.add(dirLight);

    // Resize handler
    this.resizeHandler = () => this.onResize();
    window.addEventListener('resize', this.resizeHandler);

    console.log('[MujocoViewer] Three.js initialized');
  },

  async buildSceneFromMuJoCo() {
    console.log('[MujocoViewer] Building Three.js scene from MuJoCo model...');

    const model = this.model;
    const textDecoder = new TextDecoder('utf-8');

    // Create root group
    this.mujocoRoot = new THREE.Group();
    this.mujocoRoot.name = 'MuJoCo Root';
    this.scene.add(this.mujocoRoot);

    // Loop through geoms and create Three.js meshes
    for (let g = 0; g < model.ngeom; g++) {
      // Skip collision geoms (group 3)
      if (model.geom_group[g] >= 3) continue;

      const bodyId = model.geom_bodyid[g];
      const type = model.geom_type[g];
      const size = [
        model.geom_size[(g * 3) + 0],
        model.geom_size[(g * 3) + 1],
        model.geom_size[(g * 3) + 2]
      ];

      // Create body group if needed
      if (!(bodyId in this.bodies)) {
        const bodyGroup = new THREE.Group();
        bodyGroup.bodyID = bodyId;

        const nameStart = model.name_bodyadr[bodyId];
        let nameEnd = nameStart;
        const namesArray = new Uint8Array(model.names);
        while (nameEnd < namesArray.length && namesArray[nameEnd] !== 0) {
          nameEnd++;
        }
        bodyGroup.name = textDecoder.decode(namesArray.subarray(nameStart, nameEnd));

        this.bodies[bodyId] = bodyGroup;
      }

      // Create geometry
      let geometry = this.createGeometry(type, size, g);
      if (!geometry) continue;

      // Create material
      const material = this.createMaterial(g);

      // Create mesh
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;

      // Set local position/rotation
      getPosition(model.geom_pos, g, mesh.position);
      if (type !== 0) {
        getQuaternion(model.geom_quat, g, mesh.quaternion);
      }

      if (type === 4) { // Ellipsoid
        mesh.scale.set(size[0], size[2], size[1]);
      }

      this.bodies[bodyId].add(mesh);
    }

    // Add bodies to scene
    for (let b = 0; b < model.nbody; b++) {
      if (b === 0 || !this.bodies[0]) {
        this.mujocoRoot.add(this.bodies[b]);
      } else if (this.bodies[b]) {
        this.bodies[0].add(this.bodies[b]);
      }
    }

    // Initial forward kinematics
    this.mujoco.mj_forward(this.model, this.data);
    this.updateBodyTransforms();

    console.log(`[MujocoViewer] Built scene with ${Object.keys(this.bodies).length} bodies`);
  },

  createGeometry(type, size, geomIndex) {
    const PLANE = 0, SPHERE = 2, CAPSULE = 3, ELLIPSOID = 4, CYLINDER = 5, BOX = 6, MESH = 7;

    switch (type) {
      case PLANE:
        const planeGeom = new THREE.PlaneGeometry(20, 20);
        planeGeom.rotateX(-Math.PI / 2);
        return planeGeom;

      case SPHERE:
        return new THREE.SphereGeometry(size[0], 32, 32);

      case CAPSULE:
        return new THREE.CapsuleGeometry(size[0], size[1] * 2, 16, 32);

      case ELLIPSOID:
        return new THREE.SphereGeometry(1, 32, 32);

      case CYLINDER:
        const cylGeom = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 32);
        cylGeom.rotateX(Math.PI / 2);
        return cylGeom;

      case BOX:
        return new THREE.BoxGeometry(size[0] * 2, size[2] * 2, size[1] * 2);

      case MESH:
        return this.createMeshGeometry(geomIndex);

      default:
        return null;
    }
  },

  createMeshGeometry(geomIndex) {
    const model = this.model;
    const meshId = model.geom_dataid[geomIndex];

    if (meshId < 0) return null;

    if (this.meshes[meshId]) {
      return this.meshes[meshId].clone();
    }

    const geometry = new THREE.BufferGeometry();

    const vertStart = model.mesh_vertadr[meshId] * 3;
    const vertCount = model.mesh_vertnum[meshId];
    const vertexBuffer = model.mesh_vert.subarray(vertStart, vertStart + vertCount * 3);

    // Swizzle vertices
    const swizzledVerts = new Float32Array(vertexBuffer.length);
    for (let v = 0; v < vertexBuffer.length; v += 3) {
      swizzledVerts[v + 0] = vertexBuffer[v + 0];
      swizzledVerts[v + 1] = vertexBuffer[v + 2];
      swizzledVerts[v + 2] = -vertexBuffer[v + 1];
    }

    const faceStart = model.mesh_faceadr[meshId] * 3;
    const faceCount = model.mesh_facenum[meshId];
    const faceBuffer = model.mesh_face.subarray(faceStart, faceStart + faceCount * 3);

    geometry.setAttribute('position', new THREE.BufferAttribute(swizzledVerts, 3));
    geometry.setIndex(Array.from(faceBuffer));
    geometry.computeVertexNormals();

    this.meshes[meshId] = geometry;
    return geometry.clone();
  },

  createMaterial(geomIndex) {
    const model = this.model;

    const r = model.geom_rgba[(geomIndex * 4) + 0];
    const g = model.geom_rgba[(geomIndex * 4) + 1];
    const b = model.geom_rgba[(geomIndex * 4) + 2];
    const a = model.geom_rgba[(geomIndex * 4) + 3];

    let color = new THREE.Color(r, g, b);
    let opacity = a;

    const matId = model.geom_matid[geomIndex];
    if (matId >= 0) {
      color = new THREE.Color(
        model.mat_rgba[(matId * 4) + 0],
        model.mat_rgba[(matId * 4) + 1],
        model.mat_rgba[(matId * 4) + 2]
      );
      opacity = model.mat_rgba[(matId * 4) + 3];
    }

    return new THREE.MeshPhysicalMaterial({
      color: color,
      transparent: opacity < 1.0,
      opacity: opacity,
      roughness: 0.5,
      metalness: 0.1
    });
  },

  updateBodyTransforms() {
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.data.xpos, b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix(true, true);
      }
    }
  },

  startRenderLoop() {
    const animate = () => {
      this.animationFrameId = requestAnimationFrame(animate);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    animate();
  },

  onResize() {
    const width = this.el.clientWidth;
    const height = this.el.clientHeight;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  },

  cleanup() {
    console.log('[MujocoViewer] Cleaning up...');

    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }

    if (this.physicsChannel) {
      this.physicsChannel.leave();
      this.physicsChannel = null;
    }
    if (this.physicsSocket) {
      this.physicsSocket.disconnect();
      this.physicsSocket = null;
    }

    if (this.controls) {
      this.controls.dispose();
      this.controls = null;
    }

    // Remove MuJoCo scene objects from Three.js
    if (this.mujocoRoot && this.scene) {
      this.scene.remove(this.mujocoRoot);
    }

    if (this.scene) {
      this.scene.traverse((object) => {
        if (object.geometry) object.geometry.dispose();
        if (object.material) {
          if (Array.isArray(object.material)) {
            object.material.forEach(m => m.dispose());
          } else {
            object.material.dispose();
          }
        }
      });
    }

    // CRITICAL: Free MuJoCo WASM memory
    if (this.data) {
      console.log('[MujocoViewer] Freeing MuJoCo data...');
      this.data.delete();
      this.data = null;
    }
    if (this.model) {
      console.log('[MujocoViewer] Freeing MuJoCo model...');
      // Note: model.delete() may not exist in all MuJoCo WASM versions
      // but we null the reference to allow garbage collection
      this.model = null;
    }

    // Clear body/mesh dictionaries
    this.bodies = {};
    this.meshes = {};
    this.mujocoRoot = null;

    if (this.renderer) {
      this.renderer.dispose();
      this.renderer = null;
    }

    if (this.resizeHandler) {
      window.removeEventListener('resize', this.resizeHandler);
      this.resizeHandler = null;
    }

    if (window.mjData === this.data) delete window.mjData;
    if (window.mjModel === this.model) delete window.mjModel;

    console.log('[MujocoViewer] Cleanup complete');
  },

  // Reload model without destroying Three.js renderer
  async reloadModel(newRobot) {
    console.log('[MujocoViewer] Reloading model:', newRobot);

    // Leave old physics channel
    if (this.physicsChannel) {
      this.physicsChannel.leave();
      this.physicsChannel = null;
    }

    // Remove old MuJoCo scene from Three.js
    if (this.mujocoRoot && this.scene) {
      // Dispose geometries and materials before removing
      this.mujocoRoot.traverse((object) => {
        if (object.geometry) object.geometry.dispose();
        if (object.material) {
          if (Array.isArray(object.material)) {
            object.material.forEach(m => m.dispose());
          } else {
            object.material.dispose();
          }
        }
      });
      this.scene.remove(this.mujocoRoot);
      this.mujocoRoot = null;
    }

    // Free MuJoCo WASM memory
    if (this.data) {
      this.data.delete();
      this.data = null;
    }
    this.model = null;

    // Clear dictionaries
    this.bodies = {};
    this.meshes = {};

    // Clean up virtual filesystem - remove old model files
    try {
      // Remove old scene.xml
      try {
        this.mujoco.FS.unlink('/working/scene.xml');
      } catch (e) { /* file may not exist */ }

      // Remove old assets
      try {
        const assets = this.mujoco.FS.readdir('/working/assets');
        for (const asset of assets) {
          if (asset !== '.' && asset !== '..') {
            this.mujoco.FS.unlink(`/working/assets/${asset}`);
          }
        }
      } catch (e) { /* directory may not exist */ }
    } catch (e) {
      console.warn('[MujocoViewer] FS cleanup error:', e);
    }

    // Set new robot
    this.robot = newRobot;

    // Download and load new model
    await this.downloadMJCF();

    this.model = this.mujoco.MjModel.loadFromXML('/working/scene.xml');
    if (!this.model) {
      throw new Error('Failed to load MuJoCo model');
    }

    this.data = new this.mujoco.MjData(this.model);
    if (!this.data) {
      throw new Error('Failed to create MuJoCo data');
    }

    // Rebuild scene
    await this.buildSceneFromMuJoCo();

    // Reconnect physics channel with new robot
    this.physicsChannel = this.physicsSocket.channel(`mujoco:physics:${this.robot}`, {});
    this.physicsChannel.on("physics:command", (payload) => {
      this.handlePhysicsCommand(payload);
    });

    this.physicsChannel.join()
      .receive("ok", () => {
        console.log('[MujocoViewer] Rejoined physics channel for:', this.robot);
        this.physicsChannel.push("physics:ready", {
          nq: this.model.nq,
          nv: this.model.nv,
          nu: this.model.nu
        });
      });

    // Update debug globals
    window.mjModel = this.model;
    window.mjData = this.data;

    // Notify LiveView
    this.pushEvent('mujoco_status', {
      status: 'ready',
      nbody: this.model.nbody,
      njnt: this.model.njnt,
      nq: this.model.nq,
      nv: this.model.nv,
      nu: this.model.nu,
      joint_names: this.getJointNames(),
      joint_ranges: this.getJointRanges()
    });

    console.log('[MujocoViewer] Model reload complete');
  },

  // ============================================================================
  // LiveView Event Handlers (visualization only)
  // ============================================================================

  setupCameraHandler() {
    // Camera preset - visualization only, safe for LiveView push_event
    this.handleEvent('mujoco_camera', ({ position }) => {
      console.log('[MujocoViewer] Camera:', position);
      if (position && position.length === 3) {
        this.camera.position.set(position[0], position[1], position[2]);
        this.camera.lookAt(this.controls.target);
        this.controls.update();
      }
    });

    // Robot switch handler - reload model when robot changes
    this.handleEvent('mujoco_load_robot', async ({ robot }) => {
      console.log('[MujocoViewer] Load robot event:', robot);
      if (robot && robot !== this.robot) {
        try {
          await this.reloadModel(robot);
        } catch (error) {
          console.error('[MujocoViewer] Failed to reload model:', error);
          this.pushEvent('mujoco_status', {
            status: 'error',
            message: `Failed to load robot ${robot}: ${error.message}`
          });
        }
      }
    });

    console.log('[MujocoViewer] Camera and robot handlers registered');
  },

  // Note: Physics stepping (pause, step, reset, set_ctrl) is now controlled
  // by Elixir Simulation GenServer via Channel commands, not LiveView events.

  // ============================================================================
  // Joint Info Helpers
  // ============================================================================

  getJointNames() {
    const names = [];
    const textDecoder = new TextDecoder('utf-8');
    const namesArray = new Uint8Array(this.model.names);

    for (let i = 0; i < this.model.nu; i++) {
      const jointId = this.model.actuator_trnid[i * 2];
      if (jointId >= 0 && jointId < this.model.njnt) {
        const nameStart = this.model.name_jntadr[jointId];
        let nameEnd = nameStart;
        while (nameEnd < namesArray.length && namesArray[nameEnd] !== 0) {
          nameEnd++;
        }
        names.push(textDecoder.decode(namesArray.subarray(nameStart, nameEnd)));
      } else {
        names.push(`actuator_${i}`);
      }
    }

    return names;
  },

  getJointRanges() {
    const ranges = [];

    for (let i = 0; i < this.model.nu; i++) {
      const jointId = this.model.actuator_trnid[i * 2];
      if (jointId >= 0 && jointId < this.model.njnt) {
        // Check if joint has limits
        const limited = this.model.jnt_limited[jointId];
        if (limited) {
          const rangeIdx = jointId * 2;
          ranges.push([
            this.model.jnt_range[rangeIdx],
            this.model.jnt_range[rangeIdx + 1]
          ]);
        } else {
          // No limits - use default range
          ranges.push([-3.14159, 3.14159]);
        }
      } else {
        ranges.push([-1, 1]);
      }
    }

    return ranges;
  }
};

export default MujocoViewer;
