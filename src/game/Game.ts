import { Vec3 } from '../math/Vec3';
import { Vec2 } from '../math/Vec2';
import { Quat } from '../math/Quat';
import { World } from '../physics/World';
import { Climber, LimbGrip } from '../climber/Climber';
import { Wall, buildDefaultWall } from '../wall/Wall';
import { Hold } from '../wall/Hold';
import { computeCoM, supportPolygon, stabilityMargin } from '../balance/CenterOfMass';
import { computeMoveDifficulty, classifyDifficulty } from '../balance/Difficulty';
import { FATIGUE_PARAMS } from '../fatigue/ThreeCompartment';

export interface GameSnapshot {
  t: number;
  climberPos: [number, number, number];
  com: [number, number, number];
  support: Array<[number, number]>;
  stabilityMargin: number;
  forearmPump: number;       // 0..1
  gripAvailability: number;  // 0..1
  lactate: number;
  pcr: number;
  aerobic: number;
  fatigueByRegion: Record<string, { mA: number; mF: number }>;
  moveDifficulty: number;
  moveClass: string;
  gripFailCount: number;
  fell: boolean;
  onHolds: Record<string, string | null>;
  selectedHold: string | null;
  activeLimb: LimbGrip['limb'];
  holds: Hold[];
  wallAngle: number;
  topReached: boolean;
  bodies: Array<{ id: string; pos: [number, number, number]; quat: [number, number, number, number]; length: number; radius: number }>;
  /** Per-limb reach state for HUD / renderer. */
  reachTargets: Record<string, string | null>;
  /** World-space tip position of each limb (for renderer to draw reach lines). */
  limbTips: Record<string, [number, number, number]>;
  /** Player's current body-lean bias in [-1, 1] for x and y. */
  bodyLean: [number, number];
  /** Player's leg-drive command in [0, 1]. */
  legDrive: number;
}

/**
 * Top-level game state owner. One `tick(dtSeconds)` per rendered frame does
 * fixed-substep physics and bookkeeping, and produces a snapshot for the UI.
 */
export class Game {
  world = new World();
  wall: Wall;
  climber: Climber;
  t = 0;

  // Input state (mutated externally by React UI)
  activeLimb: LimbGrip['limb'] = 'R_hand';
  selectedHold: Hold | null = null;
  pendingAction: 'grip' | 'release' | 'reach' | null = null;
  /** Per-limb active reach target. Null = no reach in progress. */
  reachTargets: Map<LimbGrip['limb'], Hold> = new Map();
  /** Player WASD body lean bias in [-1, 1]^2 (x = lateral, y = forward). */
  bodyLean: Vec2 = new Vec2(0, 0);
  /** Player leg-drive command in [0, 1]. Held SPACE → 1, otherwise decays. */
  legDriveCmd = 0;
  /** Distance threshold (m) at which an active reach auto-attaches. The
   *  generous threshold reflects that a real climber "grabs" a hold when
   *  their hand is near it, not just when fingertip-precise. */
  autoAttachDistance = 0.30;

  // Difficulty/Stability snapshots
  private lastMoveDifficulty = 0;
  private lastMoveClass = 'Easy';
  private fell = false;
  private topReached = false;

  // Physics substepping
  private fixedDt = 1 / 120;
  private accumulator = 0;

  constructor(wallAngle = 15) {
    this.wall = buildDefaultWall(wallAngle);
    // Climber is placed on the +z side of the wall surface, facing -z.
    this.climber = new Climber(this.world, { position: new Vec3(0, 1.0, 0.4) });

    // Snap the starting holds to the T-pose limb positions so the attach
    // constraints start with zero error (otherwise the body yanks violently).
    this.alignStartHoldsToClimber();

    const startL = this.wall.holds.find(h => h.id === 'start_L')!;
    const startR = this.wall.holds.find(h => h.id === 'start_R')!;
    const footL = this.wall.holds.find(h => h.id === 'foot_L0')!;
    const footR = this.wall.holds.find(h => h.id === 'foot_R0')!;
    this.climber.attachGrip('L_hand', startL);
    this.climber.attachGrip('R_hand', startR);
    this.climber.attachGrip('L_foot', footL);
    this.climber.attachGrip('R_foot', footR);

    // Ambient muscle tone
    this.climber.setRegionActivation('trunk', 0.2);
    this.climber.setRegionActivation('hip', 0.2);
    this.climber.setRegionActivation('shoulder', 0.3);
    this.climber.setRegionActivation('grip', 0.5);
  }

  /** Call once per animation frame with real delta in seconds. */
  tick(frameDt: number): void {
    this.accumulator += Math.min(frameDt, 0.05); // clamp to avoid tunnelling after a tab-switch
    while (this.accumulator >= this.fixedDt) {
      this.stepFixed(this.fixedDt);
      this.accumulator -= this.fixedDt;
      this.t += this.fixedDt;
    }
  }

  private stepFixed(dt: number): void {
    // --- Handle pending one-shot input ---
    if (this.pendingAction === 'release') {
      this.climber.releaseGrip(this.activeLimb);
      this.climber.cancelReach(this.activeLimb);
      this.reachTargets.delete(this.activeLimb);
      this.pendingAction = null;
    } else if (this.pendingAction === 'grip' && this.selectedHold) {
      this.climber.attachGrip(this.activeLimb, this.selectedHold);
      this.climber.cancelReach(this.activeLimb);
      this.reachTargets.delete(this.activeLimb);
      this.pendingAction = null;
      if (this.selectedHold.id === 'top') this.topReached = true;
    } else if (this.pendingAction === 'reach' && this.selectedHold) {
      // Begin reaching toward selected hold with active limb.
      this.climber.releaseGrip(this.activeLimb);
      this.reachTargets.set(this.activeLimb, this.selectedHold);
      this.pendingAction = null;
    }

    // --- Update body-lean pose targets (spine + hips bias toward CoM target) ---
    this.applyBodyLean();
    // --- Apply leg drive command directly to the climber.
    this.climber.legDrive = this.legDriveCmd;

    // --- Run IK for any active reach + auto-attach when close ---
    for (const [limb, hold] of this.reachTargets) {
      this.climber.reachToward(limb, hold.position);
      const tip = this.climber.limbTipWorld(limb);
      if (tip.distanceTo(hold.position) < this.autoAttachDistance) {
        this.climber.attachGrip(limb, hold);
        this.climber.cancelReach(limb);
        this.reachTargets.delete(limb);
        if (hold.id === 'top') this.topReached = true;
      }
    }

    this.climber.prePhysicsStep(dt);
    this.world.step(dt);
    this.climber.postPhysicsStep(dt);

    this.updateDifficulty();
    this.detectFall();
  }

  /**
   * Translate the player's WASD lean bias into a target world-frame
   * orientation for the pelvis (driven by the postural controller) plus a
   * little knee flex if the player leans into the wall. The spine joints
   * stay at rest so the trunk rides with the pelvis.
   */
  private applyBodyLean(): void {
    const lateralRad = this.bodyLean.x * 0.40;   // up to ~23° pelvis tilt
    const pitchRad   = -this.bodyLean.y * 0.30;  // forward = lean toward wall

    const roll = Quat.fromAxisAngle(new Vec3(0, 0, 1), lateralRad);
    const tilt = Quat.fromAxisAngle(new Vec3(1, 0, 0), pitchRad);
    this.climber.pelvisOrientationTarget.copy(Quat.mul(roll, tilt, new Quat()).normalize());

    // Knee bend bias from forward lean (squat into the wall).
    const kneeFlex = Math.max(0, this.bodyLean.y) * 0.7; // [0, 0.7] rad
    for (const knee of ['L_knee_j', 'R_knee_j']) {
      const rest = this.climber.jointRestRelative.get(knee);
      if (!rest) continue;
      const flex = Quat.fromAxisAngle(new Vec3(1, 0, 0), kneeFlex);
      this.climber.setJointTarget(knee, Quat.mul(rest, flex, new Quat()).normalize());
    }
  }

  private updateDifficulty(): void {
    // Collect torque demand (use MTG activation * T_max as proxy torque)
    const jointTorques = new Map<string, number>();
    const jointMax = new Map<string, number>();
    for (const [id, mtg] of this.climber.mtgs) {
      jointTorques.set(id, mtg.activation * mtg.params.tMax);
      jointMax.set(id, mtg.params.tMax);
    }

    const com = computeCoM([...this.climber.bodies.values()]);
    const supports = this.climber.activeContactPoints();
    const poly = supportPolygon(supports);
    const margin = stabilityMargin(new Vec2(com.x, com.y), poly);

    // Friction margin — simple proxy: 1 - (implied force / maxForce)
    const fric: number[] = [];
    for (const g of this.climber.grips) {
      if (g.constraint && g.constraint.maxForce > 0) {
        const f = g.constraint.impliedForce(this.fixedDt);
        fric.push(Math.max(0, 1 - f / g.constraint.maxForce));
      }
    }

    // Reach ratio — distance from shoulder to furthest hand contact / arm length
    const shoulderY = com.y + 0.3;
    let reachRatio = 0.6;
    for (const g of this.climber.grips) {
      if (g.hold && (g.limb === 'L_hand' || g.limb === 'R_hand')) {
        const dx = g.hold.position.x - com.x;
        const dy = g.hold.position.y - shoulderY;
        const d = Math.hypot(dx, dy);
        reachRatio = Math.max(reachRatio, d / 0.85);
      }
    }
    reachRatio = Math.min(1, reachRatio);

    this.lastMoveDifficulty = computeMoveDifficulty({
      jointTorques, jointMaxTorques: jointMax,
      stabilityMargin: margin, frictionMargins: fric, reachRatio,
    });
    this.lastMoveClass = classifyDifficulty(this.lastMoveDifficulty);
  }

  private detectFall(): void {
    if (this.fell) return;
    // Lost all contacts or dropped too far
    const anyContact = this.climber.grips.some(g => g.constraint);
    const com = computeCoM([...this.climber.bodies.values()]);
    if (!anyContact && com.y < 0.2) this.fell = true;
    // Pelvis fell way below start
    const pelvis = this.climber.bodies.get('pelvis')!;
    if (pelvis.position.y < -1.5) this.fell = true;
  }

  // ----- Input API -----

  selectLimb(limb: LimbGrip['limb']): void { this.activeLimb = limb; }
  selectHold(hold: Hold | null): void { this.selectedHold = hold; }
  requestGrip(): void { this.pendingAction = 'grip'; }
  requestRelease(): void { this.pendingAction = 'release'; }
  /**
   * Begin reaching the active limb toward the selected hold. The reach
   * persists until the limb auto-attaches (within `autoAttachDistance`) or the
   * player presses release / triggers another reach.
   */
  requestReach(): void { this.pendingAction = 'reach'; }
  cancelReach(limb?: LimbGrip['limb']): void {
    const l = limb ?? this.activeLimb;
    this.climber.cancelReach(l);
    this.reachTargets.delete(l);
  }
  /** Bias the climber's CoM target. Each component clamped to [-1, 1]. */
  setLean(x: number, y: number): void {
    this.bodyLean.set(
      Math.max(-1, Math.min(1, x)),
      Math.max(-1, Math.min(1, y)),
    );
  }
  /** Set the climber's commanded leg drive [0, 1]. */
  setLegDrive(d: number): void {
    this.legDriveCmd = Math.max(0, Math.min(1, d));
  }
  reset(): void {
    // Simplest: rebuild the whole game.
    this.world = new World();
    this.wall = buildDefaultWall(this.wall.angle);
    this.climber = new Climber(this.world, { position: new Vec3(0, 1.0, 0.4) });
    this.fell = false;
    this.topReached = false;
    this.t = 0;
    this.accumulator = 0;
    this.pendingAction = null;
    this.selectedHold = null;
    this.reachTargets.clear();
    this.bodyLean.set(0, 0);

    this.alignStartHoldsToClimber();

    const startL = this.wall.holds.find(h => h.id === 'start_L')!;
    const startR = this.wall.holds.find(h => h.id === 'start_R')!;
    const footL = this.wall.holds.find(h => h.id === 'foot_L0')!;
    const footR = this.wall.holds.find(h => h.id === 'foot_R0')!;
    this.climber.attachGrip('L_hand', startL);
    this.climber.attachGrip('R_hand', startR);
    this.climber.attachGrip('L_foot', footL);
    this.climber.attachGrip('R_foot', footR);
  }

  /**
   * Position the starting holds at the climber's T-pose hand/foot tips so the
   * attach constraints start with zero positional error. This avoids the
   * instantaneous yank that would otherwise happen when gripping holds placed
   * on a far-away wall surface.
   */
  private alignStartHoldsToClimber(): void {
    const limbMap: Array<[string, 'L_hand' | 'R_hand' | 'L_foot' | 'R_foot', [number, number, number]]> = [
      ['start_L', 'L_hand', [0, 0.095, 0]],
      ['start_R', 'R_hand', [0, 0.095, 0]],
      ['foot_L0', 'L_foot', [0, 0.13, 0]],
      ['foot_R0', 'R_foot', [0, 0.13, 0]],
    ];
    for (const [holdId, bodyId, tipLocal] of limbMap) {
      const hold = this.wall.holds.find(h => h.id === holdId);
      const body = this.climber.bodies.get(bodyId);
      if (!hold || !body) continue;
      const world = body.localToWorld(new Vec3(...tipLocal), new Vec3());
      hold.position.copy(world);
    }
  }

  // ----- Snapshot for UI / rendering -----

  snapshot(): GameSnapshot {
    const com = computeCoM([...this.climber.bodies.values()]);
    const supports = this.climber.activeContactPoints();
    const poly = supportPolygon(supports);
    const margin = stabilityMargin(new Vec2(com.x, com.y), poly);

    const onHolds: Record<string, string | null> = {};
    for (const g of this.climber.grips) onHolds[g.limb] = g.hold?.id ?? null;

    const fatigueByRegion: Record<string, { mA: number; mF: number }> = {};
    for (const [region, state] of this.climber.fatigue) {
      fatigueByRegion[region] = { mA: state.M_A, mF: state.M_F };
    }

    const gripFat = this.climber.fatigue.get('grip')!;
    const gripAvail = Math.max(0, Math.min(1, gripFat.M_R + gripFat.M_A));

    const bodies: GameSnapshot['bodies'] = [];
    for (const [id, b] of this.climber.bodies) {
      if (id === '__world__') continue;
      const seg = import_segment(id);
      bodies.push({
        id,
        pos: [b.position.x, b.position.y, b.position.z],
        quat: [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w],
        length: seg.length,
        radius: seg.radius,
      });
    }

    const reachTargets: Record<string, string | null> = {
      L_hand: this.reachTargets.get('L_hand')?.id ?? null,
      R_hand: this.reachTargets.get('R_hand')?.id ?? null,
      L_foot: this.reachTargets.get('L_foot')?.id ?? null,
      R_foot: this.reachTargets.get('R_foot')?.id ?? null,
    };

    const limbTips: Record<string, [number, number, number]> = {};
    for (const limb of ['L_hand', 'R_hand', 'L_foot', 'R_foot'] as LimbGrip['limb'][]) {
      const tip = this.climber.limbTipWorld(limb);
      limbTips[limb] = [tip.x, tip.y, tip.z];
    }

    return {
      t: this.t,
      climberPos: [com.x, com.y, com.z],
      com: [com.x, com.y, com.z],
      support: poly.map(p => [p.x, p.y] as [number, number]),
      stabilityMargin: margin,
      forearmPump: this.climber.lastForearmPumpL,
      gripAvailability: gripAvail,
      lactate: this.climber.energy.lactate,
      pcr: this.climber.energy.phosphocreatine,
      aerobic: this.climber.energy.aerobicCapacity,
      fatigueByRegion,
      moveDifficulty: this.lastMoveDifficulty,
      moveClass: this.lastMoveClass,
      gripFailCount: this.climber.gripFailCount,
      fell: this.fell,
      onHolds,
      selectedHold: this.selectedHold?.id ?? null,
      activeLimb: this.activeLimb,
      holds: this.wall.holds,
      wallAngle: this.wall.angle,
      topReached: this.topReached,
      bodies,
      reachTargets,
      limbTips,
      bodyLean: [this.bodyLean.x, this.bodyLean.y],
      legDrive: this.legDriveCmd,
    };
  }
}

// Lazy-imported helper (keeps Game decoupled from skeleton/segments path)
import { segmentById } from '../skeleton/segments';
function import_segment(id: string) { return segmentById(id); }
