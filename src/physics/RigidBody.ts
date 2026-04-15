import { Vec3 } from '../math/Vec3';
import { Quat } from '../math/Quat';

/**
 * Rigid body in maximal coordinates. Diagonal body-local inertia tensor stored
 * as a 3-vector; the world-space inverse inertia is recomputed each step from
 * the orientation.
 */
export class RigidBody {
  id: string;

  // State
  position = new Vec3();
  orientation = Quat.identity();
  linearVelocity = new Vec3();
  angularVelocity = new Vec3();

  // Accumulators (cleared each step)
  force = new Vec3();
  torque = new Vec3();

  // Mass properties
  mass: number;
  invMass: number;
  inertiaLocal: [number, number, number];   // body-frame diagonal inertia (Ix,Iy,Iz)
  invInertiaLocal: [number, number, number];

  // XPBD previous-step snapshot (used to compute post-constraint velocity)
  prevPosition = new Vec3();
  prevOrientation = Quat.identity();

  // Cached temporaries
  private _tmp = new Vec3();
  private _tmp2 = new Vec3();

  // Visual / grouping hint (e.g. "grip", "hand")
  tag?: string;

  constructor(id: string, mass: number, inertiaLocal: [number, number, number]) {
    this.id = id;
    this.mass = mass;
    this.invMass = mass > 0 ? 1 / mass : 0;
    this.inertiaLocal = inertiaLocal.slice() as [number, number, number];
    this.invInertiaLocal = [
      inertiaLocal[0] > 0 ? 1 / inertiaLocal[0] : 0,
      inertiaLocal[1] > 0 ? 1 / inertiaLocal[1] : 0,
      inertiaLocal[2] > 0 ? 1 / inertiaLocal[2] : 0,
    ];
  }

  clearForces(): void { this.force.set(0, 0, 0); this.torque.set(0, 0, 0); }

  applyForce(f: Vec3): void { this.force.add(f); }
  applyTorque(t: Vec3): void { this.torque.add(t); }

  /** Apply an impulse at world-space point (e.g. a grip reaction force * dt). */
  applyImpulseAt(impulse: Vec3, worldPoint: Vec3): void {
    // Linear: dv = J / m
    this.linearVelocity.addScaled(impulse, this.invMass);
    // Angular: domega = I^-1 * (r x J)
    const r = Vec3.sub(worldPoint, this.position, this._tmp);
    const rxJ = Vec3.cross(r, impulse, this._tmp2);
    const worldInvI = this.worldInvInertiaApply(rxJ, new Vec3());
    this.angularVelocity.add(worldInvI);
  }

  /** Transform a point from body-local to world coordinates. */
  localToWorld(local: Vec3, out = new Vec3()): Vec3 {
    this.orientation.rotate(local, out);
    out.add(this.position);
    return out;
  }

  worldToLocal(world: Vec3, out = new Vec3()): Vec3 {
    out.copy(world).sub(this.position);
    this.orientation.rotateInverse(out, out);
    return out;
  }

  /** Apply world-space inverse inertia to a world-space vector v. */
  worldInvInertiaApply(v: Vec3, out = new Vec3()): Vec3 {
    // Transform v into body frame
    this.orientation.rotateInverse(v, out);
    out.x *= this.invInertiaLocal[0];
    out.y *= this.invInertiaLocal[1];
    out.z *= this.invInertiaLocal[2];
    this.orientation.rotate(out, out);
    return out;
  }

  /** Get world-space inertia applied to v (i.e. I * v). */
  worldInertiaApply(v: Vec3, out = new Vec3()): Vec3 {
    this.orientation.rotateInverse(v, out);
    out.x *= this.inertiaLocal[0];
    out.y *= this.inertiaLocal[1];
    out.z *= this.inertiaLocal[2];
    this.orientation.rotate(out, out);
    return out;
  }

  /** XPBD step 1: save previous state, predict new pose from external forces. */
  predict(dt: number, gravity: Vec3, linearDamping: number, angularDamping: number): void {
    this.prevPosition.copy(this.position);
    this.prevOrientation.copy(this.orientation);
    if (this.invMass === 0) { this.clearForces(); return; }
    // Linear
    this.linearVelocity.addScaled(gravity, dt);
    this.linearVelocity.addScaled(this.force, this.invMass * dt);
    this.linearVelocity.scale(1 - linearDamping * dt);
    this.position.addScaled(this.linearVelocity, dt);
    // Angular: domega = I^-1 * (tau - omega x (I*omega))
    const Iw = this.worldInertiaApply(this.angularVelocity, new Vec3());
    const gyro = Vec3.cross(this.angularVelocity, Iw, new Vec3());
    const netTau = this.torque.clone().sub(gyro);
    const alpha = this.worldInvInertiaApply(netTau, new Vec3());
    this.angularVelocity.addScaled(alpha, dt);
    this.angularVelocity.scale(1 - angularDamping * dt);
    this.orientation.integrate(this.angularVelocity, dt);
    this.clearForces();
  }

  /** XPBD step 3: recompute velocities from the corrected positions/orientations. */
  projectVelocities(dt: number): void {
    if (this.invMass === 0) return;
    const invDt = 1 / dt;
    this.linearVelocity.copy(this.position).sub(this.prevPosition).scale(invDt);
    // Angular: qDelta = new * prev^-1; omega = 2 * log(qDelta) / dt
    const prevInv = new Quat(-this.prevOrientation.x, -this.prevOrientation.y, -this.prevOrientation.z, this.prevOrientation.w);
    const dq = Quat.mul(this.orientation, prevInv, new Quat());
    // log(dq) as rotation vector: 2 * vec(dq) if w positive; handle sign
    if (dq.w < 0) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; dq.w = -dq.w; }
    const wClamped = Math.max(-1, Math.min(1, dq.w));
    const angle = 2 * Math.acos(wClamped);
    const s = Math.sqrt(1 - wClamped * wClamped);
    if (s < 1e-6) {
      this.angularVelocity.set(dq.x * 2 * invDt, dq.y * 2 * invDt, dq.z * 2 * invDt);
    } else {
      const k = angle * invDt / s;
      this.angularVelocity.set(dq.x * k, dq.y * k, dq.z * k);
    }
  }
}
