import { Vec3 } from '../math/Vec3';
import { Quat, quatFromTo, slerp } from '../math/Quat';
import { RigidBody } from '../physics/RigidBody';
import { World } from '../physics/World';
import { PointConstraint } from '../physics/PointConstraint';
import { applyAngularLimits, applyPoseTracking } from '../physics/AngularConstraint';
import { solveCCD, IKJointSpec, clampRelativeToLimits } from '../control/IK';
import { stablePDTorque } from '../control/SPD';
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
  intent: number;          // commanded grip pressure [0,1]
  /**
   * Substeps remaining where this grip is immune to break checks. Set when
   * the grip is freshly attached so the inevitable momentum-spike transient
   * doesn't pop the grip off immediately.
   */
  breakImmunity: number;
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
    { limb: 'L_hand', hold: null, constraint: null, intent: 0, breakImmunity: 0 },
    { limb: 'R_hand', hold: null, constraint: null, intent: 0, breakImmunity: 0 },
    { limb: 'L_foot', hold: null, constraint: null, intent: 0, breakImmunity: 0 },
    { limb: 'R_foot', hold: null, constraint: null, intent: 0, breakImmunity: 0 },
  ];

  // Controller targets: desired joint angle offset, in local swing-twist coords.
  // (Currently: pose target simply determines the "rest orientation" the limits
  //  pull toward. Full IK-driven reaching is set externally by the game logic.)
  poseTargets = new Map<string, Quat>();

  // Diagnostics
  lastForearmPumpL = 0;
  lastForearmPumpR = 0;
  gripFailCount = 0;

  /**
   * Player-controlled leg drive in [0, 1]. When non-zero, leg tracking
   * gains are scaled up (powerful active extension) and a feedforward
   * upward force is applied to the pelvis, simulating a climber actively
   * pushing through their feet to stand up / launch the body.
   */
  legDrive = 0;
  /** Smoothed instantaneous drive force in newtons (ramps in/out). */
  driveForce = 0;

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

    // --- Postural stabilizer: drive pelvis toward a commanded world-frame
    //     orientation. Without this the pelvis is the unconstrained root and
    //     drifts/tilts freely under gravity, dragging the whole body with it.
    this.applyPostural(dt);

    // --- Angular limits (relative to T-pose / restRelative) + pose tracking
    //     toward the user-controllable poseTargets.
    const drive = clamp(this.legDrive, 0, 1);
    for (const j of JOINTS) {
      const a = this.bodies.get(j.parent)!;
      const b = this.bodies.get(j.child)!;
      const rest = this.jointRestRelative.get(j.id)!;
      const target = this.poseTargets.get(j.id)!;
      const fat = this.fatigue.get(j.muscleRegion)!;
      const availability = clamp(fat.M_R + fat.M_A, 0, 1) * lactScale;
      applyAngularLimits(a, b, rest, {
        swingX: j.swingX, swingZ: j.swingZ,
        twistMin: j.twistMin, twistMax: j.twistMax,
      }, this.gainForRegion(j.muscleRegion), 1.0 /* critical */, dt);

      let kpTrack = this.trackingGainForRegion(j.muscleRegion);
      let trackTarget = target;
      if (drive > 0 && (j.muscleRegion === 'hip' || j.muscleRegion === 'knee' || j.muscleRegion === 'ankle')) {
        kpTrack *= 1 + 2 * drive;
        if (drive > 0.3) trackTarget = rest;
      }

      applyPoseTracking(a, b, trackTarget, kpTrack,
        0.7 /* slightly under-critical so reaches feel responsive */,
        dt, availability);
    }

    // --- Leg drive feedforward: a real climber's leg push generates a net
    //     upward force on the body that the joint-by-joint pose tracking
    //     can't reliably reproduce because the multi-joint kinematic chain
    //     oscillates at high gains. Add the missing impulse directly to the
    //     pelvis so SPACE actually lifts the climber. The force is gated by
    //     having both feet attached (no air-drive cheating) and scaled by
    //     leg-region availability so a pumped climber can't push as hard.
    {
      const lFoot = this.grips.find(g => g.limb === 'L_foot')!;
      const rFoot = this.grips.find(g => g.limb === 'R_foot')!;
      const feetAttached = (lFoot.constraint ? 1 : 0) + (rFoot.constraint ? 1 : 0);
      // Ramp the drive force toward its target with a short time constant
      // so the body doesn't bounce when the player taps SPACE on/off.
      const hipFat = this.fatigue.get('hip')!;
      const kneeFat = this.fatigue.get('knee')!;
      const legAvail = 0.5 * (
        clamp(hipFat.M_R + hipFat.M_A, 0, 1) +
        clamp(kneeFat.M_R + kneeFat.M_A, 0, 1)
      ) * lactScale;
      // 1100 N at full drive: enough to lift the climber ~20cm but not so
      // much that body bobs violently when SPACE is held.
      const targetForce = feetAttached === 0 ? 0
        : 1100 * drive * legAvail * (feetAttached / 2);
      const tau = 0.08; // 80 ms ramp constant
      this.driveForce += (targetForce - this.driveForce) * (1 - Math.exp(-dt / tau));
      if (this.driveForce > 0.5) {
        const pelvis = this.bodies.get('pelvis')!;
        pelvis.applyForce(new Vec3(0, this.driveForce, 0));
      }
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
        // 3500 N peak — generous to absorb the dynamic oscillation
        // generated by the postural + pose-tracking controllers when
        // standing on feet, scaled down by fatigue + intent.
        const baseMaxForce = (grip.hold?.friction ?? 0.6) * 3500;
        grip.constraint.maxForce = baseMaxForce * availableStrength * grip.intent;
      } else {
        grip.constraint.maxForce = 5500 * (grip.hold?.friction ?? 0.6);
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
    const inGlobalWarmup = this.stepsSinceStart < this.warmupSteps;

    for (const grip of this.grips) {
      if (!grip.constraint) continue;
      if (grip.breakImmunity > 0) { grip.breakImmunity--; continue; }
      if (inGlobalWarmup) continue;
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
    g.breakImmunity = 60;   // 0.5s grace period for the body to settle in.
  }

  releaseGrip(limb: LimbGrip['limb']): void {
    const g = this.grips.find(g => g.limb === limb)!;
    if (g.constraint) {
      this.world.removePointConstraint(g.constraint);
      g.constraint = null;
    }
    g.hold = null;
    g.intent = 0;
    g.breakImmunity = 0;
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

  /**
   * Per-region SPD natural-frequency-squared (kp). Damping is computed
   * per-axis inside the constraint from the actual segment inertia, so kp
   * here directly controls how snappy the joint responds — kp ≈ ω_n² × I.
   *
   * Limits only fire when the joint exceeds its envelope; they need to be
   * stiff so a hyperextended joint snaps back.
   */
  private gainForRegion(region: MuscleRegion): number {
    switch (region) {
      case 'grip': return 1200;
      case 'trunk': return 2000;
      case 'shoulder': return 1500;
      case 'elbow': return 1500;
      case 'wrist': return 600;
      case 'hip': return 2000;
      case 'knee': return 1800;
      case 'ankle': return 800;
      case 'neck': return 400;
    }
  }

  /**
   * Pose-tracking gains. Per-axis critical damping is computed inside the
   * constraint from segment inertia, so kp directly sets responsiveness
   * (kp = ω_n² · I).
   *
   *   * Trunk is the stiffest — when an arm muscle fires, the equal-and-
   *     opposite reaction torque hits the thorax, and a soft trunk lets
   *     the whole upper body rotate (which dragged the IK target around
   *     in earlier tuning).
   *   * Legs are moderate. Higher gains here make the legs whip the body
   *     during a reach (the same reaction-torque problem), which made
   *     reaches fail. Active leg push for SPACE is a separate code path
   *     (legDrive multiplier).
   *   * Arms moderate so reaches feel firm but don't whip.
   */
  private trackingGainForRegion(region: MuscleRegion): number {
    switch (region) {
      case 'grip': return 300;
      case 'trunk': return 2500;
      case 'shoulder': return 500;
      case 'elbow': return 500;
      case 'wrist': return 200;
      case 'hip': return 1500;
      case 'knee': return 1200;
      case 'ankle': return 500;
      case 'neck': return 150;
    }
  }

  // ----- Public pose control -----

  /** Set a relative-orientation target for a named joint (parent-space quaternion). */
  setJointTarget(jointId: string, relTarget: Quat): void {
    this.poseTargets.set(jointId, relTarget.clone());
  }

  /** Reset the pose target on a joint to its T-pose rest orientation. */
  resetJointTarget(jointId: string): void {
    const rest = this.jointRestRelative.get(jointId);
    if (rest) this.poseTargets.set(jointId, rest.clone());
  }

  /** Set activation level on a muscle region (affects fatigue + MTG output). */
  setRegionActivation(region: MuscleRegion, u: number): void {
    for (const j of JOINTS) {
      if (j.muscleRegion === region) this.mtgs.get(j.id)!.setInput(u);
    }
  }

  // ----- Inverse kinematics (reach) -----

  /**
   * Joint chains used by IK for each limb. We use 2-joint chains (shoulder+
   * elbow / hip+knee) instead of 3-joint chains so CCD doesn't dump all the
   * rotation onto the distal joint (which would otherwise twist 100°+ for
   * any non-trivial reach). The wrist/ankle is left at its rest orientation
   * and follows the forearm/shin via the joint point constraint.
   */
  private static readonly REACH_CHAINS: Record<LimbGrip['limb'], string[]> = {
    L_hand: ['L_shoulder_j', 'L_elbow_j'],
    R_hand: ['R_shoulder_j', 'R_elbow_j'],
    L_foot: ['L_hip_j', 'L_knee_j'],
    R_foot: ['R_hip_j', 'R_knee_j'],
  };

  /**
   * End-effector position in the LAST IK-chain child's local frame:
   *   - For arms: chain ends at the forearm. The hand tip in the forearm's
   *     local frame is at +Y by (forearm half-length) + (hand length).
   *   - For legs: chain ends at the shin. The foot tip is below the shin's
   *     distal end and slightly forward (+Z), but we pretend the foot is
   *     aligned with the shin and use the shin distal as the end-effector.
   */
  private ikEndEffectorLocal(limb: LimbGrip['limb']): Vec3 {
    if (limb === 'L_hand' || limb === 'R_hand') {
      // forearm half = 0.135, hand length = 0.19  → tip at 0.135 + 0.19 = 0.325
      return new Vec3(0, 0.325, 0);
    } else {
      // shin half = 0.20 → ankle at +0.20; pretend foot extends another 0.13
      return new Vec3(0, 0.33, 0);
    }
  }

  /** Distal tip of each limb body in its OWN local frame (for distance checks). */
  private limbTipLocal(limb: LimbGrip['limb']): Vec3 {
    return new Vec3(0, limb.includes('hand') ? 0.095 : 0.13, 0);
  }

  /** World position of the limb tip (hand or foot distal end). */
  limbTipWorld(limb: LimbGrip['limb']): Vec3 {
    const body = this.bodies.get(this.limbBodyId(limb))!;
    return body.localToWorld(this.limbTipLocal(limb), new Vec3());
  }

  // CCD substep accumulator — IK runs at a slower rate than physics so the
  // pose tracker has time to actually drive the limb where IK said to go.
  private ikSubstepCounter = 0;
  private static readonly IK_DECIMATE = 3;        // re-solve every 3 substeps (~40 Hz)
  private static readonly IK_TARGET_BLEND = 0.6;  // slerp amount toward CCD output

  /**
   * One IK pass biasing the limb toward `target`. Run every physics substep
   * while a reach is active. The CCD solver itself only runs every
   * IK_DECIMATE substeps; the pose targets are slerped toward the CCD result
   * by IK_TARGET_BLEND each time, producing a smooth glide rather than a
   * yank.
   */
  reachToward(limb: LimbGrip['limb'], target: Vec3): void {
    this.ikSubstepCounter++;
    if (this.ikSubstepCounter % Climber.IK_DECIMATE !== 0) return;

    const chainIds = Climber.REACH_CHAINS[limb];
    const lastJoint = JOINTS.find(j => j.id === chainIds[chainIds.length - 1])!;
    const chain: IKJointSpec[] = chainIds.map(jId => {
      const jd = JOINTS.find(j => j.id === jId)!;
      return {
        jointId: jId,
        parentBody: this.bodies.get(jd.parent)!,
        childBody: this.bodies.get(jd.child)!,
        parentLocalAnchor: new Vec3(...jd.localAnchorParent),
      };
    });
    const targets = solveCCD(chain, this.ikEndEffectorLocal(limb), target, 8);
    for (const jId of chainIds) {
      const jd = JOINTS.find(j => j.id === jId)!;
      const raw = targets.get(jId);
      if (!raw) continue;
      const restRel = this.jointRestRelative.get(jId)!;
      const restInv = new Quat(-restRel.x, -restRel.y, -restRel.z, restRel.w);
      const err = Quat.mul(restInv, raw, new Quat());
      const clamped = clampRelativeToLimits(err, jd.swingX, jd.swingZ, jd.twistMin, jd.twistMax);
      const final = Quat.mul(restRel, clamped, new Quat()).normalize();
      // Slerp toward the new IK pose target instead of snapping.
      const current = this.poseTargets.get(jId) ?? restRel;
      this.poseTargets.set(jId, slerp(current, final, Climber.IK_TARGET_BLEND));
    }
    // Distal joint stays in rest pose so the hand/foot follows the forearm/shin.
    this.resetJointTarget(lastJoint.id === 'L_elbow_j' ? 'L_wrist_j'
                       : lastJoint.id === 'R_elbow_j' ? 'R_wrist_j'
                       : lastJoint.id === 'L_knee_j' ? 'L_ankle_j'
                       : 'R_ankle_j');
  }

  /** Reset all joints in a limb's reach chain to their rest targets. */
  cancelReach(limb: LimbGrip['limb']): void {
    for (const jId of Climber.REACH_CHAINS[limb]) this.resetJointTarget(jId);
    // Also reset the distal joint we held at rest.
    const distal = limb === 'L_hand' ? 'L_wrist_j'
                 : limb === 'R_hand' ? 'R_wrist_j'
                 : limb === 'L_foot' ? 'L_ankle_j'
                 : 'R_ankle_j';
    this.resetJointTarget(distal);
  }

  // ----- Postural controller -----

  /**
   * Player- or game-controlled target world orientation for the pelvis. By
   * default the pelvis tries to remain upright (identity quaternion); the
   * body-lean controller in Game.ts mutates this to lean the climber.
   */
  pelvisOrientationTarget = Quat.identity();

  /**
   * Apply a world-frame torque to the pelvis driving its orientation toward
   * `pelvisOrientationTarget`. Without this the root segment is unconstrained
   * and tilts freely under load. SPD form keeps it stable.
   */
  private applyPostural(dt: number): void {
    const pelvis = this.bodies.get('pelvis')!;
    // err = target * current^-1
    const ci = new Quat(-pelvis.orientation.x, -pelvis.orientation.y, -pelvis.orientation.z, pelvis.orientation.w);
    const err = Quat.mul(this.pelvisOrientationTarget, ci, new Quat());
    if (err.w < 0) { err.x = -err.x; err.y = -err.y; err.z = -err.z; err.w = -err.w; }
    const errLog = err.toLog(new Vec3());

    // Effective inertia in WORLD frame = pelvis.orientation * I_local * pelvis.orientation^T
    // For a rough diagonal approximation, we just use the average of the
    // body-local diag, which is acceptable for stabilisation purposes.
    const Iavg = (pelvis.inertiaLocal[0] + pelvis.inertiaLocal[1] + pelvis.inertiaLocal[2]) / 3;
    // Tuned: 200 keeps the pelvis upright without the strong reaction torques
    // that 1000+ generates (the latter destabilises arm reaches by whipping
    // the body when the IK pose-targets shift the shoulder).
    const kp = 200;
    const kd = 2 * Math.sqrt(kp * Iavg);

    // Per-axis SPD: drive each angular component from 0 toward errLog (the
    // axis-angle vector needed to align with target).
    const w = pelvis.angularVelocity;
    const tau = new Vec3(
      stablePDTorque(0, w.x, errLog.x, 0, kp, kd, dt, Iavg),
      stablePDTorque(0, w.y, errLog.y, 0, kp, kd, dt, Iavg),
      stablePDTorque(0, w.z, errLog.z, 0, kp, kd, dt, Iavg),
    );
    const m = tau.length();
    if (m > 250) tau.scale(250 / m);
    pelvis.applyTorque(tau);
  }
}
