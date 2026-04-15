import { Vec3 } from '../math/Vec3';
import { RigidBody } from './RigidBody';

/**
 * XPBD-style ball-joint point constraint: two points in each body's local frame
 * must coincide in world space. Supports a compliance α (1/stiffness) and a
 * maximum Lagrange multiplier, so that when it exceeds the limit the constraint
 * "breaks" (caller can test and remove the constraint).
 */
export class PointConstraint {
  a: RigidBody;
  b: RigidBody;
  localA: Vec3;
  localB: Vec3;
  compliance: number;     // α — 0 = rigid, higher = springier
  maxForce: number;       // N — if the implied force exceeds this, grip breaks
  lambda = new Vec3();    // Accumulated Lagrange multiplier (for maxForce check)

  // Cached temporaries
  private _pa = new Vec3();
  private _pb = new Vec3();
  private _ra = new Vec3();
  private _rb = new Vec3();
  private _dx = new Vec3();
  private _tmp = new Vec3();

  constructor(a: RigidBody, b: RigidBody, localA: Vec3, localB: Vec3,
              compliance = 0, maxForce = Infinity) {
    this.a = a; this.b = b;
    this.localA = localA.clone();
    this.localB = localB.clone();
    this.compliance = compliance;
    this.maxForce = maxForce;
  }

  resetLambda(): void { this.lambda.set(0, 0, 0); }

  /** Solve this constraint once, mutating body positions/orientations.
   *  Returns the implied force magnitude for this solve. */
  solve(dt: number): number {
    // World anchor points
    this.a.orientation.rotate(this.localA, this._ra);
    this._pa.copy(this._ra).add(this.a.position);
    this.b.orientation.rotate(this.localB, this._rb);
    this._pb.copy(this._rb).add(this.b.position);

    // Error vector
    this._dx.copy(this._pa).sub(this._pb);
    const C = this._dx.length();
    if (C < 1e-9) return 0;

    // Effective mass for XPBD point constraint:
    //   w = wA + wB + (r_a x n)^T I_A^-1 (r_a x n) + similar for b
    // We'll do 3 passes (one for each axis) in a simpler approach: apply
    // correction along the error direction directly.
    const n = this._tmp.copy(this._dx).scale(1 / C);

    // Compute generalized inverse mass
    const raXn = Vec3.cross(this._ra, n, new Vec3());
    const rbXn = Vec3.cross(this._rb, n, new Vec3());
    const IaInvRaXn = this.a.worldInvInertiaApply(raXn, new Vec3());
    const IbInvRbXn = this.b.worldInvInertiaApply(rbXn, new Vec3());
    const wA = this.a.invMass + Vec3.dot(raXn, IaInvRaXn);
    const wB = this.b.invMass + Vec3.dot(rbXn, IbInvRbXn);

    const alpha = this.compliance / (dt * dt);
    const deltaLambda = -C / (wA + wB + alpha);

    // Apply positional impulse p = deltaLambda * n
    const p = new Vec3(n.x * deltaLambda, n.y * deltaLambda, n.z * deltaLambda);

    this.a.position.addScaled(p, this.a.invMass);
    // Orientation: apply small rotation from (r x p) * I^-1
    {
      const dOmega = this.a.worldInvInertiaApply(Vec3.cross(this._ra, p, new Vec3()), new Vec3());
      this.a.orientation.integrate(dOmega, 1);
    }
    this.b.position.addScaled(p, -this.b.invMass);
    {
      const dOmega = this.b.worldInvInertiaApply(Vec3.cross(this._rb, p.clone().negate(), new Vec3()), new Vec3());
      this.b.orientation.integrate(dOmega, 1);
    }

    this.lambda.addScaled(n, deltaLambda);

    const forceMag = Math.abs(deltaLambda) / (dt * dt);
    return forceMag;
  }

  /** Total implied force magnitude (for break testing). */
  impliedForce(dt: number): number {
    return this.lambda.length() / (dt * dt);
  }
}
