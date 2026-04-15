import * as THREE from 'three';
import { Game, GameSnapshot } from '../game/Game';
import { Hold } from '../wall/Hold';

const HOLD_COLORS: Record<string, number> = {
  jug: 0x4caf50,
  crimp: 0xf44336,
  sloper: 0x2196f3,
  pinch: 0xffeb3b,
  pocket: 0x9c27b0,
};

/**
 * Three.js renderer. Builds a scene with:
 *   - wall plane tilted by wall.angle
 *   - hold spheres (colour by type, ring highlight for hovered/active)
 *   - climber: capsule per segment
 *   - UI overlay drawn via separate React layer (see App.tsx)
 */
export class Renderer {
  scene = new THREE.Scene();
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  rayCaster = new THREE.Raycaster();
  mouse = new THREE.Vector2();

  private capsules = new Map<string, THREE.Mesh>();
  private holdMeshes = new Map<string, THREE.Mesh>();
  private holdHighlight: THREE.Mesh;
  private reachLines = new Map<string, THREE.Line>();
  private activeTipMarker!: THREE.Mesh;
  private onHoldPick: ((hold: Hold | null) => void) | null = null;
  private containerEl: HTMLElement;

  constructor(container: HTMLElement) {
    this.containerEl = container;
    const w = container.clientWidth;
    const h = container.clientHeight;

    this.scene.background = new THREE.Color(0x1a1a22);
    this.scene.fog = new THREE.Fog(0x1a1a22, 10, 25);

    this.camera = new THREE.PerspectiveCamera(40, w / h, 0.05, 60);
    this.camera.position.set(0, 3, 6);
    this.camera.lookAt(0, 3, 0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    // Lights
    const ambient = new THREE.AmbientLight(0x8899aa, 0.4);
    this.scene.add(ambient);
    const key = new THREE.DirectionalLight(0xffffff, 1.0);
    key.position.set(3, 8, 5);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x88aaff, 0.3);
    rim.position.set(-4, 2, -3);
    this.scene.add(rim);

    // Ground
    const groundMat = new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.9 });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(20, 20), groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    // Hold highlight ring (for selected hold)
    const ringGeom = new THREE.RingGeometry(0.08, 0.11, 32);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0xffe066, side: THREE.DoubleSide, transparent: true, opacity: 0.9 });
    this.holdHighlight = new THREE.Mesh(ringGeom, ringMat);
    this.holdHighlight.visible = false;
    this.scene.add(this.holdHighlight);

    // Reach guide line — a glowing line from each reaching limb to its
    // target hold. Updated each frame in applySnapshot. Pre-allocate four
    // (one per limb).
    for (const limb of ['L_hand', 'R_hand', 'L_foot', 'R_foot']) {
      const geom = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(), new THREE.Vector3(),
      ]);
      const mat = new THREE.LineBasicMaterial({
        color: limb.includes('hand') ? 0x42a5f5 : 0xffa94d,
        transparent: true, opacity: 0.7,
      });
      const line = new THREE.Line(geom, mat);
      line.visible = false;
      this.scene.add(line);
      this.reachLines.set(limb, line);
    }

    // Active-limb tip marker — a small sphere on the climber's currently
    // selected limb so the player can see what they're moving.
    const tipGeom = new THREE.SphereGeometry(0.04, 12, 10);
    const tipMat = new THREE.MeshBasicMaterial({ color: 0xffe066, transparent: true, opacity: 0.8 });
    this.activeTipMarker = new THREE.Mesh(tipGeom, tipMat);
    this.activeTipMarker.visible = false;
    this.scene.add(this.activeTipMarker);

    // Wall + pointer events
    this.renderer.domElement.addEventListener('pointerdown', (e) => this.handlePointerDown(e));

    window.addEventListener('resize', () => this.handleResize());
  }

  setHoldPickHandler(fn: (h: Hold | null) => void) { this.onHoldPick = fn; }

  buildWall(game: Game) {
    const wall = game.wall;

    // Wall surface — tilted plane
    const wallGeom = new THREE.PlaneGeometry(wall.width, wall.height);
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x2e3440, roughness: 0.85 });
    const wallMesh = new THREE.Mesh(wallGeom, wallMat);
    wallMesh.receiveShadow = true;
    // Tilt by wall.angle about x-axis so overhang leans toward +z (climber side).
    // Default plane normal is +Z; rotating +t around X gives normal (0, -sin t, cos t),
    // which points outward and downward as required for an overhang.
    const t = wall.angle * Math.PI / 180;
    wallMesh.rotation.x = t;
    wallMesh.position.set(0, wall.height / 2 * Math.cos(t), wall.height / 2 * Math.sin(t));
    this.scene.add(wallMesh);

    // Holds
    for (const h of wall.holds) {
      const size = h.type === 'jug' ? 0.07 : h.type === 'crimp' ? 0.03 : 0.06;
      const geom = new THREE.SphereGeometry(size, 12, 10);
      const mat = new THREE.MeshStandardMaterial({ color: HOLD_COLORS[h.type] ?? 0x888888, roughness: 0.6 });
      const mesh = new THREE.Mesh(geom, mat);
      mesh.position.set(h.position.x, h.position.y, h.position.z);
      mesh.castShadow = true;
      mesh.userData.holdId = h.id;
      this.scene.add(mesh);
      this.holdMeshes.set(h.id, mesh);
    }
  }

  buildClimber(game: Game) {
    for (const [id, body] of game.climber.bodies) {
      if (id === '__world__') continue;
      const seg = game.climber.bodies.get(id)!;
      const segDef = import_segmentDef(id);
      const geom = new THREE.CapsuleGeometry(segDef.radius, Math.max(0.01, segDef.length - 2 * segDef.radius), 4, 10);
      const mat = new THREE.MeshStandardMaterial({ color: colorForSegment(id), roughness: 0.5 });
      const mesh = new THREE.Mesh(geom, mat);
      mesh.castShadow = true;
      this.scene.add(mesh);
      this.capsules.set(id, mesh);
    }
  }

  applySnapshot(snap: GameSnapshot) {
    // Update capsule transforms
    for (const b of snap.bodies) {
      const mesh = this.capsules.get(b.id);
      if (!mesh) continue;
      mesh.position.set(b.pos[0], b.pos[1], b.pos[2]);
      mesh.quaternion.set(b.quat[0], b.quat[1], b.quat[2], b.quat[3]);
    }

    // Hand / forearm color shift with pump intensity
    const pump = snap.forearmPump;
    for (const id of ['L_forearm', 'R_forearm', 'L_hand', 'R_hand']) {
      const mesh = this.capsules.get(id);
      if (!mesh) continue;
      const mat = mesh.material as THREE.MeshStandardMaterial;
      const base = new THREE.Color(0xf0c090);
      const pumped = new THREE.Color(0xd04060);
      const mix = base.clone().lerp(pumped, pump);
      mat.color.copy(mix);
    }

    // Update hold highlight
    if (snap.selectedHold) {
      const mesh = this.holdMeshes.get(snap.selectedHold);
      if (mesh) {
        this.holdHighlight.visible = true;
        this.holdHighlight.position.copy(mesh.position);
        this.holdHighlight.lookAt(this.camera.position);
      }
    } else {
      this.holdHighlight.visible = false;
    }

    // Reach guide lines: for each limb that's actively reaching, draw a line
    // from its current tip to the target hold.
    for (const [limb, line] of this.reachLines) {
      const targetId = snap.reachTargets[limb];
      if (!targetId) { line.visible = false; continue; }
      const hold = snap.holds.find(h => h.id === targetId);
      const tip = snap.limbTips[limb];
      if (!hold || !tip) { line.visible = false; continue; }
      line.visible = true;
      const positions = (line.geometry.attributes.position as THREE.BufferAttribute);
      positions.setXYZ(0, tip[0], tip[1], tip[2]);
      positions.setXYZ(1, hold.position.x, hold.position.y, hold.position.z);
      positions.needsUpdate = true;
      line.geometry.computeBoundingSphere();
    }

    // Mark the currently-selected limb's tip with a yellow sphere so the
    // player can see which limb is "active".
    const activeTip = snap.limbTips[snap.activeLimb];
    if (activeTip) {
      this.activeTipMarker.visible = true;
      this.activeTipMarker.position.set(activeTip[0], activeTip[1], activeTip[2]);
    }

    // Camera follow climber CoM with smoothing
    const target = new THREE.Vector3(0, snap.com[1] + 0.3, snap.com[2] + 4.5);
    this.camera.position.lerp(target, 0.05);
    this.camera.lookAt(snap.com[0], snap.com[1], snap.com[2]);
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    this.renderer.dispose();
    if (this.renderer.domElement.parentNode === this.containerEl) {
      this.containerEl.removeChild(this.renderer.domElement);
    }
  }

  private handleResize() {
    const w = this.containerEl.clientWidth;
    const h = this.containerEl.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  private handlePointerDown(e: PointerEvent) {
    if (!this.onHoldPick) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this.rayCaster.setFromCamera(this.mouse, this.camera);
    const holdObjects = [...this.holdMeshes.values()];
    const hits = this.rayCaster.intersectObjects(holdObjects, false);
    if (hits.length > 0) {
      const id = hits[0].object.userData.holdId as string;
      this.onHoldPick(this.findHoldById(id));
    } else {
      this.onHoldPick(null);
    }
  }

  private _lastGameRef: Game | null = null;
  setGame(g: Game) { this._lastGameRef = g; }
  private findHoldById(id: string): Hold | null {
    if (!this._lastGameRef) return null;
    return this._lastGameRef.wall.holds.find(h => h.id === id) ?? null;
  }
}

import { segmentById } from '../skeleton/segments';
function import_segmentDef(id: string) { return segmentById(id); }

function colorForSegment(id: string): number {
  if (id === 'pelvis' || id === 'lumbar' || id === 'thorax') return 0x3a6ea5;
  if (id === 'head') return 0xf0c090;
  if (id.startsWith('L_upper_arm') || id.startsWith('R_upper_arm')) return 0x4a8bc2;
  if (id.startsWith('L_forearm') || id.startsWith('R_forearm')) return 0xf0c090;
  if (id.startsWith('L_hand') || id.startsWith('R_hand')) return 0xf0c090;
  if (id.includes('thigh')) return 0x2e4a6b;
  if (id.includes('shin')) return 0x3a6ea5;
  if (id.includes('foot')) return 0x222222;
  return 0x888888;
}
