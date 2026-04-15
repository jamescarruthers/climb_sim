import { Vec3 } from '../math/Vec3';
import { Quat, quatFromTo } from '../math/Quat';
import { RigidBody } from '../physics/RigidBody';
import { World } from '../physics/World';
import { PointConstraint } from '../physics/PointConstraint';
import { applyAngularLimits } from '../physics/AngularConstraint';
import { SEGMENTS, DEFAULT_MASS, SegmentDef } from '../skeleton/segments';
import { JOINTS, JointDef, MuscleRegion } from '../skeleton/joints';
import { diagInertia } from '../skeleton/inertia';
import { MTG } from '../muscle/MTG';
import { mtgParamsForRegion } from '../muscle/params';
import { FatigueState, freshState, stepFatigue, FATIGUE_PARAMS,
         gripBloodFlowFactor, armPositionRecoveryFactor } from '../fatigue/ThreeCompartment';
import { EnergyState, freshEnergy, stepEnergy, lactateStrengthScale } from '../energy/EnergySystem';
import { Hold } from '../wall/Hold';
import { clamp, rad } from '../math/util';
import { computeCoM } from '../balance/CenterOfMass';

export interface ClimberOptions {
  mass?: number;
  position?: Vec3;
}

/**
 * Per-limb grip state. A limb is either free (no constraint) or attached to a
 * hold via a point constraint whose maxForce depends on grip strength.
 */
export interface LimbGrip {
  limb: 'L_hand' | 'R_hand' | 'L_foot' | 'R_foot';
  hold: Hold | null;
  constraint: PointConstraint | null;
  intent: number;     // commanded grip pressure [0,1]
}

/**
 * The player avatar — 20-segment articulated ragdoll with MTG actuators,
 * 3CCr fatigue per muscle region, a global energy system, and four grip slots.
 *
 * Construction:
 *   1. Build rigid bodies for each segment, placing them in a T-pose facing
 *      the wall (+Z is away from wall).
 *   2. Add point constraints for each joint so positions track.
 *   3. Store rest relative orientations for each joint → used by angular
 *      limiter.
 */
export class Climber {
  world: World;
  mass: number;
  bodies = new Map<string, RigidBody>();
  jointConstraints = new Map<string, PointConstraint>();
  jointRestRelative = new Map<string, Quat>();

  mtgs = new Map<string, MTG>();       // keyed by joint.id
  fatigue = new Map<MuscleRegion, FatigueState>();
  energy: EnergyState = freshEnergy();

  grips: LimbGrip[] = [
    { limb: 'L_hand', hold: null, constraint: null, intent: 0 },
    { limb: 'R_hand', hold: null, constraint: null, intent: 0 },
    { limb: 'L_foot', hold: null, constraint: null, intent: 0 },
    { limb: 'R_foot', hold: null, constraint: null, intent: 0 },
  ];

  // Controller targets: desired joint angle offset, in local swing-twist coords.
  // (Currently: pose target simply determines the "rest orientation" the limits
  //  pull toward. Full IK-driven reaching is set externally by the game logic.)
  poseTargets = new Map<string, Quat>();

  // Diagnostics
  lastForearmPumpL = 0;
  lastForearmPumpR = 0;
  gripFailCount = 0;

  // Warmup: during the first `warmupSteps` physics substeps after
  // construction, grip constraints cannot break. This lets the body settle
  // out of its idealised T-pose spawn into a relaxed hang without a transient
  // blowing out the grips.
  private warmupSteps = 240;     // ~2s at 120 Hz: fully damped settle
  private stepsSinceStart = 0;

  constructor(world: World, options: ClimberOptions = {}) {
    this.world = world;
    this.mass = options.mass ?? DEFAULT_MASS;

    this.buildBodies(options.position ?? new Vec3(0, 1, -0.4));
    this.buildJoints();
    this.buildMTGs();
    this.initFatigue();
  }

  // ----- Construction helpers -----

  private buildBodies(rootPos: Vec3): void {
    // Normalise mass fractions so we exactly equal total climber mass.
    const totalFraction = SEGMENTS.reduce((s, seg) => s + seg.massFraction, 0);
    const scale = this.mass / totalFraction;

    // Place each segment in a T-pose with orientation such that the segment's
    // local +Y axis aligns with its rest direction in world space. Use the
    // joint's exact localAnchorChild offset to place the body so that the
    // child anchor world position matches the parent anchor at startup.
    const placeSegment = (seg: SegmentDef, parentBody?: RigidBody, joint?: JointDef) => {
      const segMass = seg.massFraction * scale;
      const inertia = diagInertia(segMass, seg.length, seg.radius);
      const body = new RigidBody(seg.id, segMass, inertia);
      body.tag = seg.id;

      const dir = this.segmentRestDirection(seg.id);
      body.orientation.copy(quatFromTo(new Vec3(0, 1, 0), dir));

      if (!parentBody || !joint) {
        body.position.copy(rootPos);
      } else {
        const parentAnchor = parentBody.localToWorld(new Vec3(...joint.localAnchorParent), new Vec3());
        const childAnchorLocal = new Vec3(...joint.localAnchorChild);
        const childAnchorInWorldIfAtOrigin = body.orientation.rotate(childAnchorLocal, new Vec3());
        // body.position + childAnchorInWorldIfAtOrigin == parentAnchor
        body.position.copy(parentAnchor).sub(childAnchorInWorldIfAtOrigin);
      }
      this.bodies.set(seg.id, body);
      this.world.addBody(body);
    };

    const root = SEGMENTS.find(s => s.parent === null)!;
    placeSegment(root);

    const queue: SegmentDef[] = [root];
    while (queue.length) {
      const cur = queue.shift()!;
      const curBody = this.bodies.get(cur.id)!;
      for (const seg of SEGMENTS.filter(s => s.parent === cur.id)) {
        const j = JOINTS.find(j => j.parent === cur.id && j.child === seg.id);
        placeSegment(seg, curBody, j);
        queue.push(seg);
      }
    }
  }

  /** T-pose rest direction of a segment's long axis, in world space. */
  private segmentRestDirection(id: string): Vec3 {
    // Arms extend laterally (X±), legs extend downward (-Y), spine up (+Y), head up.
    if (id.startsWith('L_upper_arm') || id.startsWith('L_forearm') || id.startsWith('L_hand'))
      return new Vec3(-1, 0, 0);
    if (id.startsWith('R_upper_arm') || id.startsWith('R_forearm') || id.startsWith('R_hand'))
      return new Vec3(1, 0, 0);
    if (id.includes('thigh') || id.includes('shin')) return new Vec3(0, -1, 0);
    if (id.includes('foot')) return new Vec3(0, 0, 1);
    // Spine & head: up
    return new Vec3(0, 1, 0);
  }

  private buildJoints(): void {
    for (const j of JOINTS) {
      const a = this.bodies.get(j.parent)!;
      const b = this.bodies.get(j.child)!;
      const la = new Vec3(...j.localAnchorParent);
      const lb = new Vec3(...j.localAnchorChild);
      const c = new PointConstraint(a, b, la, lb, 0.0, Infinity);
      this.world.addPointConstraint(c);
      this.jointConstraints.set(j.id, c);

      // Rest relative orientation = parent^-1 * child  (using current T-pose orientations)
      const parentInv = new Quat(-a.orientation.x, -a.orientation.y, -a.orientation.z, a.orientation.w);
      const rel = Quat.mul(parentInv, b.orientation, new Quat());
      this.jointRestRelative.set(j.id, rel);
      this.poseTargets.set(j.id, rel.clone());
    }
  }

  private buildMTGs(): void {
    for (const j of JOINTS) {
      const thetaMin = -Math.max(j.swingX, j.swingZ);
      const thetaMax = Math.max(j.swingX, j.swingZ);
      const params = mtgParamsForRegion(j.muscleRegion, thetaMin, thetaMax, 0);
      this.mtgs.set(j.id, new MTG(params));
    }
  }

  private initFatigue(): void {
    for (const region of Object.keys(FATIGUE_PARAMS) as MuscleRegion[]) {
      this.fatigue.set(region, freshState());
    }
  }

  // ----- Runtime step -----

  /**
   * Pre-physics: apply angular joint limits, SPD pose torques, compute MTG
   * outputs modulated by fatigue/energy. Also update fatigue/energy states.
   */
  prePhysicsStep(dt: number): void {
    // --- Update energy first (uses current activation average)
    let totalActivation = 0;
    let n = 0;
    for (const mtg of this.mtgs.values()) { totalActivation += mtg.activation; n++; }
    const avgAct = n > 0 ? totalActivation / n : 0;
    this.energy = stepEnergy(this.energy, avgAct, dt);
    const lactScale = lactateStrengthScale(this.energy.lactate);

    // --- Per-region fatigue accumulation
    // Aggregate target load per muscle region by averaging across joints in region
    const regionLoad = new Map<MuscleRegion, { sum: number; n: number }>();
    for (const j of JOINTS) {
      const mtg = this.mtgs.get(j.id)!;
      const slot = regionLoad.get(j.muscleRegion) ?? { sum: 0, n: 0 };
      slot.sum += mtg.targetU;
      slot.n += 1;
      regionLoad.set(j.muscleRegion, slot);
    }

    // Arm position recovery factor (for grip region)
    const L_hand = this.bodies.get('L_hand')!;
    const R_hand = this.bodies.get('R_hand')!;
    const com = computeCoM([...this.bodies.values()]);
    const heartY = com.y + 0.2; // approximate heart height above CoM
    const armYRel = Math.max(L_hand.position.y, R_hand.position.y) - heartY;
    const armRecovery = armPositionRecoveryFactor(armYRel);

    for (const [region, { sum, n }] of regionLoad) {
      const targetU = n > 0 ? sum / n : 0;
      const base = FATIGUE_PARAMS[region];
      let params = base;

      if (region === 'grip') {
        const gripActivation = Math.max(
          this.mtgs.get('L_wrist_j')!.activation,
          this.mtgs.get('R_wrist_j')!.activation,
        );
        const blood = gripBloodFlowFactor(gripActivation);
        params = { ...base, R: base.R * blood * armRecovery };
      }

      const prev = this.fatigue.get(region)!;
      this.fatigue.set(region, stepFatigue(prev, targetU, dt, params));
    }

    this.lastForearmPumpL = this.fatigue.get('grip')!.M_F;
    this.lastForearmPumpR = this.fatigue.get('grip')!.M_F;

    // --- Angular limits + pose tracking torque (SPD-based, stable for all I)
    for (const j of JOINTS) {
      const a = this.bodies.get(j.parent)!;
      const b = this.bodies.get(j.child)!;
      const rest = this.poseTargets.get(j.id)!;
      applyAngularLimits(a, b, rest, {
        swingX: j.swingX, swingZ: j.swingZ,
        twistMin: j.twistMin, twistMax: j.twistMax,
      }, this.gainForRegion(j.muscleRegion), this.dampForRegion(j.muscleRegion), dt);
    }

    // --- MTG torques: modulate each joint's output by available fatigue strength
    //     Currently we don't map MTG torques back into specific per-DOF torques;
    //     the SPD-style torques are already inside applyAngularLimits. MTG
    //     output here is used to *cap* those by "availableStrength".
    for (const j of JOINTS) {
      const mtg = this.mtgs.get(j.id)!;
      const fat = this.fatigue.get(j.muscleRegion)!;
      const availableStrength = clamp(fat.M_R + fat.M_A, 0, 1) * lactScale;
      mtg.update(0, 0, dt, availableStrength);
    }

    // --- Update grip constraint maxForce from grip-region availability.
    // Apply extra damping during warmup to help the ragdoll settle.
    const inWarmup = this.stepsSinceStart < this.warmupSteps;
    if (inWarmup) {
      // Strong velocity damping kills oscillations during settle.
      for (const b of this.bodies.values()) {
        b.linearVelocity.scale(0.85);
        b.angularVelocity.scale(0.85);
      }
    }
    for (const grip of this.grips) {
      if (!grip.constraint) continue;
      if (grip.limb === 'L_hand' || grip.limb === 'R_hand') {
        const fat = this.fatigue.get('grip')!;
        const availableStrength = clamp(fat.M_R + fat.M_A, 0, 1) * lactScale;
        // 2500 N peak — generous to absorb dynamic oscillation in the
        // ragdoll. Will scale down with fatigue/grip availability.
        const baseMaxForce = (grip.hold?.friction ?? 0.6) * 2500;
        grip.constraint.maxForce = baseMaxForce * availableStrength * grip.intent;
      } else {
        grip.constraint.maxForce = 4000 * (grip.hold?.friction ?? 0.6);
      }
    }
  }

  /**
   * Post-physics: check grip forces; break constraints that exceeded maxForce.
   * During the warmup window grips never break — we let the ragdoll settle
   * before applying realistic strength limits.
   */
  postPhysicsStep(dt: number): void {
    this.stepsSinceStart++;
    if (this.stepsSinceStart < this.warmupSteps) return;

    for (const grip of this.grips) {
      if (!grip.constraint) continue;
      const force = grip.constraint.impliedForce(dt);
      if (force > grip.constraint.maxForce && grip.constraint.maxForce > 0) {
        if ((globalThis as any).__DEBUG_GRIP) {
          console.log(`[grip break] ${grip.limb}: force=${force.toFixed(0)} max=${grip.constraint.maxForce.toFixed(0)}`);
        }
        this.releaseGrip(grip.limb);
        this.gripFailCount++;
      }
    }
  }

  // ----- Grip management -----

  attachGrip(limb: LimbGrip['limb'], hold: Hold): void {
    // Release any existing grip on this limb
    this.releaseGrip(limb);

    const body = this.bodies.get(this.limbBodyId(limb))!;
    // Anchor at the distal end of hand/foot
    const localAnchor = new Vec3(0, limb.includes('hand') ? 0.09 : 0.13, 0);
    const anchorWorld = body.localToWorld(localAnchor, new Vec3());
    // Hold is a fixed anchor in the world — model as a point on a "ground" body.
    // We'll use a "world body" trick: create a zero-mass RigidBody positioned at
    // hold.position and constrain to it. To keep it simple, we piggy-back on a
    // single cached "world anchor" body.
    const worldBody = this.getOrCreateWorldBody();
    const localHold = worldBody.worldToLocal(hold.position, new Vec3());
    const c = new PointConstraint(worldBody, body, localHold, localAnchor, 1e-5, 500);
    this.world.addPointConstraint(c);

    const g = this.grips.find(g => g.limb === limb)!;
    g.hold = hold;
    g.constraint = c;
    g.intent = 1;
  }

  releaseGrip(limb: LimbGrip['limb']): void {
    const g = this.grips.find(g => g.limb === limb)!;
    if (g.constraint) {
      this.world.removePointConstraint(g.constraint);
      g.constraint = null;
    }
    g.hold = null;
    g.intent = 0;
  }

  /** Get the active contact points in world space (for CoM/polygon). */
  activeContactPoints(): Vec3[] {
    const pts: Vec3[] = [];
    for (const g of this.grips) {
      if (g.hold) pts.push(g.hold.position.clone());
    }
    return pts;
  }

  private _worldBody: RigidBody | null = null;
  private getOrCreateWorldBody(): RigidBody {
    if (!this._worldBody) {
      this._worldBody = new RigidBody('__world__', 0, [0, 0, 0]);
      this.world.addBody(this._worldBody);
    }
    return this._worldBody;
  }

  private limbBodyId(limb: LimbGrip['limb']): string {
    switch (limb) {
      case 'L_hand': return 'L_hand';
      case 'R_hand': return 'R_hand';
      case 'L_foot': return 'L_foot';
      case 'R_foot': return 'R_foot';
    }
  }

  // Per-region SPD gains. These are normalised by child inertia inside
  // applyAngularLimits, so what matters here is the natural frequency
  // ω_n = sqrt(kp). Higher kp = stiffer joint.
  private gainForRegion(region: MuscleRegion): number {
    switch (region) {
      case 'grip': return 150;
      case 'trunk': return 300;
      case 'shoulder': return 200;
      case 'elbow': return 180;
      case 'wrist': return 100;
      case 'hip': return 250;
      case 'knee': return 200;
      case 'ankle': return 100;
      case 'neck': return 60;
    }
  }

  /** Critical damping for a given kp. */
  private dampForRegion(region: MuscleRegion): number {
    return 2 * Math.sqrt(this.gainForRegion(region));
  }

  // ----- Public pose control -----

  /** Set a relative-orientation target for a named joint (parent-space quaternion). */
  setJointTarget(jointId: string, relTarget: Quat): void {
    this.poseTargets.set(jointId, relTarget.clone());
  }

  /** Set activation level on a muscle region (affects fatigue + MTG output). */
  setRegionActivation(region: MuscleRegion, u: number): void {
    for (const j of JOINTS) {
      if (j.muscleRegion === region) this.mtgs.get(j.id)!.setInput(u);
    }
  }
}
