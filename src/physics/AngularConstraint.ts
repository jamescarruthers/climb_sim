import { Vec3 } from '../math/Vec3';
import { Quat } from '../math/Quat';
import { RigidBody } from './RigidBody';
import { stablePDTorque } from '../control/SPD';

/**
 * Swing-twist angular limit + pose tracking. Uses a per-axis SPD controller
 * (stable for arbitrary gains and timesteps) to push the child back into the
 * elliptical swing cone and twist range, and to damp relative angular
 * velocity.
 *
 * Gains are scaled by the CHILD's effective transverse inertia so that we get
 * a consistent response regardless of segment mass (a hand and a thorax both
 * see similar impedance).
 */

export interface AngularLimits {
  swingX: number;
  swingZ: number;
  twistMin: number;
  twistMax: number;
}

const MAX_LIMB_TORQUE = 250;  // Nm — cap to prevent exotic transients

export function applyAngularLimits(
  parent: RigidBody,
  child: RigidBody,
  restRelative: Quat,
  limits: AngularLimits,
  kp = 200,
  kd = 20,
  dt = 1 / 120,
): void {
  // Relative orientation w.r.t. rest: err = (rest^-1 * parent^-1 * child)
  const parentInv = new Quat(-parent.orientation.x, -parent.orientation.y, -parent.orientation.z, parent.orientation.w);
  const rel = Quat.mul(parentInv, child.orientation, new Quat());
  const restInv = new Quat(-restRelative.x, -restRelative.y, -restRelative.z, restRelative.w);
  const err = Quat.mul(restInv, rel, new Quat());

  // Swing-twist about the child's proximal axis (parent's Y)
  const [swing, twist] = Quat.swingTwist(err, new Vec3(0, 1, 0));
  const swingLog = swing.toLog(new Vec3());
  const swingX = swingLog.x;
  const swingZ = swingLog.z;
  const twistLog = twist.toLog(new Vec3());
  const twistY = twistLog.y;

  // How far outside the elliptical swing cone? (0 = inside)
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

  // Relative angular velocity in parent-local frame
  const relOmegaWorld = new Vec3(
    child.angularVelocity.x - parent.angularVelocity.x,
    child.angularVelocity.y - parent.angularVelocity.y,
    child.angularVelocity.z - parent.angularVelocity.z,
  );
  const relOmega = parent.orientation.rotateInverse(relOmegaWorld, new Vec3());

  // Effective inertia for each axis (from child's body-local diag inertia,
  // rotated into parent frame via the relative orientation).
  const I = effectiveInertiaParentFrame(child, rel);

  // Per-axis SPD with thetaTarget implicitly 0 (we're already computing the
  // delta from the rest/limit pose) and omegaTarget = 0.
  const tauX = stablePDTorque(swingExcessX, relOmega.x, 0, 0, kp, kd, dt, I[0]);
  const tauY = stablePDTorque(twistExcess,  relOmega.y, 0, 0, kp, kd, dt, I[1]);
  const tauZ = stablePDTorque(swingExcessZ, relOmega.z, 0, 0, kp, kd, dt, I[2]);

  // Clamp magnitude to protect against any remaining transients.
  const torqueLocal = new Vec3(tauX, tauY, tauZ);
  const m = torqueLocal.length();
  if (m > MAX_LIMB_TORQUE) torqueLocal.scale(MAX_LIMB_TORQUE / m);

  const torqueWorld = parent.orientation.rotate(torqueLocal, new Vec3());
  child.applyTorque(torqueWorld);
  parent.applyTorque(torqueWorld.clone().negate());
}

/**
 * Rotate child's body-local diagonal inertia into parent's frame (as a
 * diagonal approximation — we take the absolute rotated-axis contributions).
 */
function effectiveInertiaParentFrame(child: RigidBody, rel: Quat): [number, number, number] {
  // Each principal axis of the child's body-local inertia rotates via rel.
  // Project its contribution onto each parent-frame axis.
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
