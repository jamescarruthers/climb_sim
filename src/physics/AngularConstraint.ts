import { Vec3 } from '../math/Vec3';
import { Quat } from '../math/Quat';
import { RigidBody } from './RigidBody';
import { stablePDTorque } from '../control/SPD';

/**
 * Joint angular control split into two torque sources:
 *
 *   1. applyAngularLimits — hard limit cone + twist range. Only fires when
 *      the joint is outside its envelope (relative to the FIXED rest pose).
 *
 *   2. applyPoseTracking — soft pose tracking. Drives the joint toward an
 *      independently-set pose target via SPD, regardless of where the limit
 *      envelope is. The IK solver and the body-lean controller write into
 *      poseTargets without affecting the limit envelope.
 *
 * Damping is computed per-axis from the actual segment inertia
 * (kd_axis = 2·dampingRatio·sqrt(kp · I_axis)) so a hand and a thorax —
 * which differ by 100×+ in inertia — both end up critically damped at the
 * same kp setting. Without this, a single fixed kd over-damps small
 * segments by the same factor and the controller produces almost no torque
 * at all. (Symptom: arms hang limp during a reach because the shoulder /
 * elbow tracking is effectively dead at the gains needed for the trunk.)
 */

export interface AngularLimits {
  swingX: number;
  swingZ: number;
  twistMin: number;
  twistMax: number;
}

// Per-spec peak isometric torque for the strongest single-joint actuator
// (hip extension) is 200 N·m. Limits and tracking can briefly exceed that
// for stability transients, but we cap each so physics doesn't run away.
// Tracking cap is generous because legs need to push body weight UP, not
// just hold it static — a real squat / leg drive transmits 2–3× body
// weight through the knee on the concentric phase.
const MAX_LIMB_TORQUE = 400;     // Nm — cap to bound transients
const MAX_TRACK_TORQUE = 450;    // Nm — pose tracking; legs need this for
                                 //       active extension against gravity.

/** kd for critical damping at gain `kp` on a body with effective inertia `I`. */
function criticalKd(kp: number, I: number, ratio: number): number {
  return 2 * ratio * Math.sqrt(Math.max(1e-6, kp * I));
}

export function applyAngularLimits(
  parent: RigidBody,
  child: RigidBody,
  restRelative: Quat,
  limits: AngularLimits,
  kp: number,
  dampingRatio: number,
  dt: number,
): void {
  const err = relativeError(parent, child, restRelative);
  const [swing, twist] = Quat.swingTwist(err, new Vec3(0, 1, 0));
  const swingLog = swing.toLog(new Vec3());
  const swingX = swingLog.x;
  const swingZ = swingLog.z;
  const twistY = twist.toLog(new Vec3()).y;

  const k = (swingX / limits.swingX) ** 2 + (swingZ / limits.swingZ) ** 2;
  let swingExcessX = 0, swingExcessZ = 0;
  if (k > 1) {
    const scale = 1 - 1 / Math.sqrt(k);
    swingExcessX = swingX * scale;
    swingExcessZ = swingZ * scale;
  }
  let twistExcess = 0;
  if (twistY < limits.twistMin) twistExcess = twistY - limits.twistMin;
  else if (twistY > limits.twistMax) twistExcess = twistY - limits.twistMax;

  const relOmega = relativeAngularVelocity(parent, child);
  const I = effectiveInertiaParentFrame(child, computeRelative(parent, child));
  const kdX = criticalKd(kp, I[0], dampingRatio);
  const kdY = criticalKd(kp, I[1], dampingRatio);
  const kdZ = criticalKd(kp, I[2], dampingRatio);

  const tauX = stablePDTorque(swingExcessX, relOmega.x, 0, 0, kp, kdX, dt, I[0]);
  const tauY = stablePDTorque(twistExcess,  relOmega.y, 0, 0, kp, kdY, dt, I[1]);
  const tauZ = stablePDTorque(swingExcessZ, relOmega.z, 0, 0, kp, kdZ, dt, I[2]);

  applyClampedTorque(parent, child, new Vec3(tauX, tauY, tauZ), MAX_LIMB_TORQUE);
}

/**
 * Drive the joint's relative orientation toward `poseTarget` (parent-frame
 * relative quaternion). `availability` in [0,1] scales gains down with
 * fatigue so a pumped joint can't push as hard.
 */
export function applyPoseTracking(
  parent: RigidBody,
  child: RigidBody,
  poseTarget: Quat,
  kp: number,
  dampingRatio: number,
  dt: number,
  availability = 1,
): void {
  const currentRel = computeRelative(parent, child);
  const currentRelInv = new Quat(-currentRel.x, -currentRel.y, -currentRel.z, currentRel.w);
  const err = Quat.mul(poseTarget, currentRelInv, new Quat());
  if (err.w < 0) { err.x = -err.x; err.y = -err.y; err.z = -err.z; err.w = -err.w; }
  const errLog = err.toLog(new Vec3());

  const relOmega = relativeAngularVelocity(parent, child);
  const I = effectiveInertiaParentFrame(child, currentRel);
  const k = kp * availability;
  const kdX = criticalKd(k, I[0], dampingRatio);
  const kdY = criticalKd(k, I[1], dampingRatio);
  const kdZ = criticalKd(k, I[2], dampingRatio);

  const tauX = stablePDTorque(0, relOmega.x, errLog.x, 0, k, kdX, dt, I[0]);
  const tauY = stablePDTorque(0, relOmega.y, errLog.y, 0, k, kdY, dt, I[1]);
  const tauZ = stablePDTorque(0, relOmega.z, errLog.z, 0, k, kdZ, dt, I[2]);

  applyClampedTorque(parent, child, new Vec3(tauX, tauY, tauZ), MAX_TRACK_TORQUE);
}

// ---------- shared helpers ----------

function computeRelative(parent: RigidBody, child: RigidBody): Quat {
  const pi = new Quat(-parent.orientation.x, -parent.orientation.y, -parent.orientation.z, parent.orientation.w);
  return Quat.mul(pi, child.orientation, new Quat());
}

function relativeError(parent: RigidBody, child: RigidBody, restRelative: Quat): Quat {
  const rel = computeRelative(parent, child);
  const ri = new Quat(-restRelative.x, -restRelative.y, -restRelative.z, restRelative.w);
  return Quat.mul(ri, rel, new Quat());
}

function relativeAngularVelocity(parent: RigidBody, child: RigidBody): Vec3 {
  const w = new Vec3(
    child.angularVelocity.x - parent.angularVelocity.x,
    child.angularVelocity.y - parent.angularVelocity.y,
    child.angularVelocity.z - parent.angularVelocity.z,
  );
  return parent.orientation.rotateInverse(w, new Vec3());
}

function applyClampedTorque(parent: RigidBody, child: RigidBody, tauLocal: Vec3, cap: number): void {
  const m = tauLocal.length();
  if (m > cap) tauLocal.scale(cap / m);
  const torqueWorld = parent.orientation.rotate(tauLocal, new Vec3());
  child.applyTorque(torqueWorld);
  parent.applyTorque(torqueWorld.clone().negate());
}

function effectiveInertiaParentFrame(child: RigidBody, rel: Quat): [number, number, number] {
  const Ix = child.inertiaLocal[0];
  const Iy = child.inertiaLocal[1];
  const Iz = child.inertiaLocal[2];
  const ex = rel.rotate(new Vec3(1, 0, 0), new Vec3());
  const ey = rel.rotate(new Vec3(0, 1, 0), new Vec3());
  const ez = rel.rotate(new Vec3(0, 0, 1), new Vec3());
  return [
    Math.abs(ex.x) * Ix + Math.abs(ey.x) * Iy + Math.abs(ez.x) * Iz,
    Math.abs(ex.y) * Ix + Math.abs(ey.y) * Iy + Math.abs(ez.y) * Iz,
    Math.abs(ex.z) * Ix + Math.abs(ey.z) * Iy + Math.abs(ez.z) * Iz,
  ];
}
