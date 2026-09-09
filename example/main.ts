/**
 * mujoco-web-loader demo: pick a URDF (plus its STL meshes) with the file
 * input and watch it simulate under gravity with injected PD position
 * actuators holding pose.
 *
 * Deliberately main-thread and ~200 lines so the whole pipeline is readable
 * top to bottom. In a real app, run MuJoCo in a Web Worker (this package's
 * `initMujocoModule` + `mujocoWorker` plumbing work identically there) and
 * import from '@robot-sim/mujoco-web-loader' instead of '../src'.
 */

import * as THREE from 'three'
import {
  MuJoCoEngine,
  initMujocoModule,
  stripUnsupportedURDFElements,
  preprocessURDF,
  applyURDFPass2,
  computeSTLBounds,
  DEFAULT_ENVIRONMENT,
  type MeshBoundsInfo,
  type RobotDescriptor,
} from '../src/index'
// Vite resolves the WASM asset URL; the loader takes it as plain config.
import mujocoWasmUrl from '@mujoco/mujoco/mujoco.wasm?url'

const statusEl = document.getElementById('status') as HTMLSpanElement
const filesEl = document.getElementById('files') as HTMLInputElement
const canvas = document.getElementById('viewport') as HTMLCanvasElement

function status(text: string): void {
  statusEl.textContent = text
}

// --- Three.js scene (Y-up); MuJoCo world (Z-up) parents under a rotated root
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true })
const scene = new THREE.Scene()
scene.background = new THREE.Color(0x14161a)
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100)
camera.position.set(0.8, 0.6, 1.2)
camera.lookAt(0, 0.2, 0)
scene.add(new THREE.HemisphereLight(0xffffff, 0x333944, 1.2))
const sun = new THREE.DirectionalLight(0xffffff, 1.5)
sun.position.set(2, 4, 3)
scene.add(sun)
scene.add(new THREE.GridHelper(4, 40, 0x39404d, 0x252a33))

const mujocoRoot = new THREE.Group()
mujocoRoot.rotation.x = -Math.PI / 2 // MuJoCo Z-up → Three Y-up
scene.add(mujocoRoot)

function resize(): void {
  const w = canvas.clientWidth
  const h = canvas.clientHeight
  renderer.setSize(w, h, false)
  camera.aspect = w / h
  camera.updateProjectionMatrix()
}
window.addEventListener('resize', resize)
resize()

// --- Engine (initialized once; models load/replace on top of it)
status('loading MuJoCo WASM…')
const mj = await initMujocoModule({
  wasmUrl: mujocoWasmUrl,
  printErr: (t) => console.error(`[MuJoCo] ${t}`),
})
const engine = new MuJoCoEngine(mj)
status(`MuJoCo ready (version ${engine.getVersion()}) — select a .urdf and its .stl meshes`)

// --- Per-model render state
const bodyGroups = new Map<number, THREE.Group>()

function geometryFor(v: RobotDescriptor['visuals'][number]): THREE.BufferGeometry | null {
  // MuJoCo sizes are half-extents (box) / radius+half-length (cylinder,
  // capsule); Three geometries take full extents and are Y-aligned where
  // MuJoCo's are Z-aligned — the mesh-level rotation below accounts for it.
  switch (v.type) {
    case 'box':
    case 'mesh': // preprocessURDF substitutes measured bounding boxes for meshes
      return new THREE.BoxGeometry(v.size[0] * 2, v.size[1] * 2, v.size[2] * 2)
    case 'sphere':
      return new THREE.SphereGeometry(v.size[0], 24, 16)
    case 'cylinder':
      return new THREE.CylinderGeometry(v.size[0], v.size[0], v.size[1] * 2, 24)
    case 'capsule':
      return new THREE.CapsuleGeometry(v.size[0], v.size[1] * 2, 8, 16)
    case 'ellipsoid':
      return new THREE.SphereGeometry(1, 24, 16).scale(v.size[0], v.size[1], v.size[2])
    case 'plane':
      return null // the grid helper stands in for the ground plane
  }
}

function buildScene(descriptor: RobotDescriptor): void {
  for (const group of bodyGroups.values()) mujocoRoot.remove(group)
  bodyGroups.clear()
  for (const body of descriptor.bodies) {
    const group = new THREE.Group()
    bodyGroups.set(body.id, group)
    mujocoRoot.add(group)
  }
  for (const v of descriptor.visuals) {
    const geometry = geometryFor(v)
    if (!geometry) continue
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(v.rgba[0], v.rgba[1], v.rgba[2]),
      transparent: v.rgba[3] < 1,
      opacity: v.rgba[3],
      roughness: 0.7,
    })
    const mesh = new THREE.Mesh(geometry, material)
    if (v.type === 'cylinder' || v.type === 'capsule') {
      geometry.rotateX(Math.PI / 2) // MuJoCo Z-axis solids → Three Y-axis geometries
    }
    mesh.position.set(v.localPos[0], v.localPos[1], v.localPos[2])
    // MuJoCo quat [w,x,y,z] → Three [x,y,z,w]
    mesh.quaternion.set(v.localQuat[1], v.localQuat[2], v.localQuat[3], v.localQuat[0])
    bodyGroups.get(v.bodyId)?.add(mesh)
  }
}

// --- Load pipeline: exactly the README quickstart
async function loadRobot(urdfXml: string, stls: ReadonlyMap<string, Uint8Array>): Promise<void> {
  status('writing meshes + measuring bounds…')
  engine.ensureVFSDir('/working')
  engine.ensureVFSDir('/working/assets')
  const meshBounds = new Map<string, MeshBoundsInfo>()
  for (const [name, bytes] of stls) {
    engine.writeToVFS(`/working/assets/${name}`, bytes)
    const info = computeSTLBounds(bytes)
    if (info) meshBounds.set(name, info)
  }

  status('preprocessing + compiling…')
  const stripped = stripUnsupportedURDFElements(urdfXml)
  engine.loadModel(preprocessURDF(stripped, DEFAULT_ENVIRONMENT, meshBounds), '/working/model.xml')

  status('two-pass actuator/sensor injection…')
  const descriptor = applyURDFPass2(engine, mj, urdfXml, {}, undefined, '/working/model_pass2.xml')

  buildScene(descriptor)
  status(
    `${descriptor.name || 'robot'}: ${descriptor.bodyCount} bodies, ` +
      `${descriptor.jointCount} joints, ${descriptor.actuatorCount} actuators — simulating`,
  )
}

filesEl.addEventListener('change', () => {
  void (async () => {
    const files = [...(filesEl.files ?? [])]
    const urdf = files.find((f) => f.name.toLowerCase().endsWith('.urdf'))
    if (!urdf) {
      status('no .urdf in the selection — pick the robot file plus its meshes')
      return
    }
    const stls = new Map<string, Uint8Array>()
    for (const f of files) {
      if (f.name.toLowerCase().endsWith('.stl')) {
        stls.set(f.name, new Uint8Array(await f.arrayBuffer()))
      }
    }
    try {
      await loadRobot(await urdf.text(), stls)
    } catch (err) {
      console.error(err)
      status(`load failed: ${err instanceof Error ? err.message : String(err)}`)
    }
  })()
})

// --- Sim + render loop (~60 Hz, N physics substeps per frame)
function tick(): void {
  requestAnimationFrame(tick)
  if (engine.isLoaded() && bodyGroups.size > 0) {
    const dt = engine.getTimestep()
    engine.stepN(Math.max(1, Math.round(1 / 60 / dt)))
    for (const body of engine.getBodyStates()) {
      const group = bodyGroups.get(body.id)
      if (!group) continue
      group.position.set(body.position[0], body.position[1], body.position[2])
      group.quaternion.set(body.quaternion[1], body.quaternion[2], body.quaternion[3], body.quaternion[0])
    }
  }
  renderer.render(scene, camera)
}
tick()
