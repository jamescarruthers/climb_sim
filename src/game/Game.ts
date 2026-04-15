import { Vec3 } from '../math/Vec3';
import { Vec2 } from '../math/Vec2';
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
  pendingAction: 'grip' | 'release' | null = null;

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
    // Handle pending input
    if (this.pendingAction === 'release') {
      this.climber.releaseGrip(this.activeLimb);
      this.pendingAction = null;
    } else if (this.pendingAction === 'grip' && this.selectedHold) {
      this.climber.attachGrip(this.activeLimb, this.selectedHold);
      this.pendingAction = null;
      // Check top
      if (this.selectedHold.id === 'top') this.topReached = true;
    }

    this.climber.prePhysicsStep(dt);
    this.world.step(dt);
    this.climber.postPhysicsStep(dt);

    this.updateDifficulty();
    this.detectFall();
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
    };
  }
}

// Lazy-imported helper (keeps Game decoupled from skeleton/segments path)
import { segmentById } from '../skeleton/segments';
function import_segment(id: string) { return segmentById(id); }
