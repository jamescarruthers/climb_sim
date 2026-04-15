import { Vec3 } from '../math/Vec3';
import { RigidBody } from './RigidBody';
import { PointConstraint } from './PointConstraint';

/**
 * Physics world — owns bodies, joint point-constraints, and runs the integrator
 * + XPBD solver each step.
 */
export class World {
  bodies: RigidBody[] = [];
  pointConstraints: PointConstraint[] = [];

  gravity = new Vec3(0, -9.81, 0);
  linearDamping = 0.4;
  angularDamping = 1.2;

  solverIterations = 16;

  addBody(b: RigidBody): RigidBody { this.bodies.push(b); return b; }
  addPointConstraint(c: PointConstraint): PointConstraint {
    this.pointConstraints.push(c);
    return c;
  }

  removePointConstraint(c: PointConstraint): void {
    const i = this.pointConstraints.indexOf(c);
    if (i >= 0) this.pointConstraints.splice(i, 1);
  }

  step(dt: number): void {
    // 1. Predict new poses from external forces/torques (also saves prev state).
    for (const b of this.bodies) {
      b.predict(dt, this.gravity, this.linearDamping, this.angularDamping);
    }

    // 2. XPBD constraint solver — project positions to satisfy constraints.
    for (const c of this.pointConstraints) c.resetLambda();
    for (let iter = 0; iter < this.solverIterations; iter++) {
      for (const c of this.pointConstraints) c.solve(dt);
    }

    // 3. Recompute linear/angular velocities from the corrected positions so
    //    constraint-induced impulses persist through future steps.
    for (const b of this.bodies) b.projectVelocities(dt);
  }
}
