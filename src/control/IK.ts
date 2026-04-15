import { Vec3 } from '../math/Vec3';
import { Quat, quatFromTo } from '../math/Quat';
import { RigidBody } from '../physics/RigidBody';

/**
 * Cyclic Coordinate Descent (CCD) inverse kinematics.
 *
 * Operates on a virtual copy of the chain's current pose, so it does NOT
 * mutate any rigid bodies. Returns the parent-frame relative orientations
 * the joints would need to adopt to bring the end-effector to the target.
 * The caller wires these up as pose-tracking targets and lets the SPD layer
 * drive the actual physics there.
 *
 * CCD is greedy and over-determined for our 3-joint arm/leg chains, but it
 * converges quickly and is naturally well-behaved with joint limits because
 * each joint is rotated independently and we can clamp after each step.
 */

export interface IKJointSpec {
  jointId: string;
  parentBody: RigidBody;
  childBody: RigidBody;
  /** Anchor on parent in parent body-local coords. */
  parentLocalAnchor: Vec3;
}

interface VPose { pos: Vec3; orient: Quat; }

export function solveCCD(
  chain: IKJointSpec[],
  endEffectorChildLocal: Vec3,
  targetWorld: Vec3,
  iterations = 16,
  tolerance = 0.005,
): Map<string, Quat> {
  const result = new Map<string, Quat>();
  if (chain.length === 0) return result;

  // Snapshot current pose into a virtual chain.
  const virtual: VPose[] = chain.map(j => ({
    pos: j.childBody.position.clone(),
    orient: j.childBody.orientation.clone(),
  }));
  const root: VPose = {
    pos: chain[0].parentBody.position.clone(),
    orient: chain[0].parentBody.orientation.clone(),
  };

  const tipAt = (): Vec3 => {
    const tip = virtual[virtual.length - 1];
    return tip.orient.rotate(endEffectorChildLocal, new Vec3()).add(tip.pos);
  };

  for (let iter = 0; iter < iterations; iter++) {
    // Walk joints from end-effector back to root.
    for (let i = chain.length - 1; i >= 0; i--) {
      const parent = i > 0 ? virtual[i - 1] : root;
      const pivot = parent.orient.rotate(chain[i].parentLocalAnchor, new Vec3()).add(parent.pos);

      const end = tipAt();
      const v1 = end.clone().sub(pivot);
      const v2 = targetWorld.clone().sub(pivot);
      if (v1.lengthSq() < 1e-12 || v2.lengthSq() < 1e-12) continue;

      const rot = quatFromTo(v1, v2);
      // Rotate this joint and every descendant about the pivot.
      for (let k = i; k < chain.length; k++) {
        const b = virtual[k];
        const rel = b.pos.clone().sub(pivot);
        const newRel = rot.rotate(rel, new Vec3());
        b.pos.copy(pivot).add(newRel);
        Quat.mul(rot, b.orient, b.orient).normalize();
      }
    }

    if (tipAt().sub(targetWorld).lengthSq() < tolerance * tolerance) break;
  }

  // Extract parent-frame relative orientations.
  for (let i = 0; i < chain.length; i++) {
    const parent = i > 0 ? virtual[i - 1] : root;
    const child = virtual[i];
    const parentInv = new Quat(-parent.orient.x, -parent.orient.y, -parent.orient.z, parent.orient.w);
    const rel = Quat.mul(parentInv, child.orient, new Quat()).normalize();
    result.set(chain[i].jointId, rel);
  }
  return result;
}

/** Clamp a relative orientation into a swing-twist limit envelope. */
export function clampRelativeToLimits(
  rel: Quat,
  swingX: number,
  swingZ: number,
  twistMin: number,
  twistMax: number,
): Quat {
  const [swing, twist] = Quat.swingTwist(rel, new Vec3(0, 1, 0));
  const swingLog = swing.toLog(new Vec3());
  let sx = swingLog.x;
  let sz = swingLog.z;
  const k = (sx / swingX) ** 2 + (sz / swingZ) ** 2;
  if (k > 1) {
    const s = 1 / Math.sqrt(k);
    sx *= s;
    sz *= s;
  }
  const twistLog = twist.toLog(new Vec3());
  let ty = twistLog.y;
  if (ty < twistMin) ty = twistMin;
  else if (ty > twistMax) ty = twistMax;

  // Reconstruct: swing about (sx, 0, sz), twist about Y by ty.
  const swingAngle = Math.hypot(sx, sz);
  const swingClamped = swingAngle < 1e-9
    ? Quat.identity()
    : Quat.fromAxisAngle(new Vec3(sx / swingAngle, 0, sz / swingAngle), swingAngle);
  const twistClamped = Quat.fromAxisAngle(new Vec3(0, 1, 0), ty);
  return Quat.mul(swingClamped, twistClamped, new Quat()).normalize();
}
