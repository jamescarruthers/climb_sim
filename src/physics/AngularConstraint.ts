import { Vec3 } from '../math/Vec3';
import { Quat } from '../math/Quat';
import { RigidBody } from './RigidBody';
import { stablePDTorque } from '../control/SPD';

/**
 * Joint angular control split into two torque sources:
 *
 *   1. applyAngularLimits — hard limit cone + twist range. Only fires when
 *      the joint is outside its envelope (relative to the FIXED rest pose).
 *      Damping is applied unconditionally so unconstrained joints don't
 *      spin freely.
 *
 *   2. applyPoseTracking — soft pose tracking. Drives the joint toward an
 *      independently-set pose target via SPD, regardless of where the limit
 *      envelope is. The IK solver and the body-lean controller write into
 *      poseTargets without affecting the limit envelope.
 *
 * Both use per-axis Stable PD with inertia-weighted gains so distal segments
 * stay stable.
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
const MAX_LIMB_TORQUE = 300;     // Nm — cap to bound transients
const MAX_TRACK_TORQUE = 250;    // Nm — pose tracking, big enough to support
                                 //       body weight through the legs.

export function applyAngularLimits(
  parent: RigidBody,
  child: RigidBody,
  restRelative: Quat,
  limits: AngularLimits,
  kp: number,
  kd: number,
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

  const tauX = stablePDTorque(swingExcessX, relOmega.x, 0, 0, kp, kd, dt, I[0]);
  const tauY = stablePDTorque(twistExcess,  relOmega.y, 0, 0, kp, kd, dt, I[1]);
  const tauZ = stablePDTorque(swingExcessZ, relOmega.z, 0, 0, kp, kd, dt, I[2]);

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
  kd: number,
  dt: number,
  availability = 1,
): void {
  // currentRel = parent^-1 * child
  const currentRel = computeRelative(parent, child);
  // Error rotation = poseTarget * currentRel^-1
  const currentRelInv = new Quat(-currentRel.x, -currentRel.y, -currentRel.z, currentRel.w);
  const err = Quat.mul(poseTarget, currentRelInv, new Quat());
  // Continuity: choose shortest arc.
  if (err.w < 0) { err.x = -err.x; err.y = -err.y; err.z = -err.z; err.w = -err.w; }
  const errLog = err.toLog(new Vec3());

  const relOmega = relativeAngularVelocity(parent, child);
  const I = effectiveInertiaParentFrame(child, currentRel);

  const k = kp * availability;
  const d = kd * Math.sqrt(availability);
  const tauX = stablePDTorque(0, relOmega.x, errLog.x, 0, k, d, dt, I[0]);
  const tauY = stablePDTorque(0, relOmega.y, errLog.y, 0, k, d, dt, I[1]);
  const tauZ = stablePDTorque(0, relOmega.z, errLog.z, 0, k, d, dt, I[2]);

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
