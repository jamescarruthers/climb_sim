import { Vec3 } from './Vec3';

/**
 * Mutable quaternion (x, y, z, w), w is the scalar. Hamilton convention.
 * Represents rotations; |q| should be kept near 1.
 */
export class Quat {
  x: number;
  y: number;
  z: number;
  w: number;

  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x; this.y = y; this.z = z; this.w = w;
  }

  static identity(): Quat { return new Quat(0, 0, 0, 1); }

  set(x: number, y: number, z: number, w: number): this { this.x = x; this.y = y; this.z = z; this.w = w; return this; }
  copy(q: Quat): this { this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w; return this; }
  clone(): Quat { return new Quat(this.x, this.y, this.z, this.w); }

  setAxisAngle(axis: Vec3, angle: number): this {
    const h = angle * 0.5;
    const s = Math.sin(h);
    this.x = axis.x * s;
    this.y = axis.y * s;
    this.z = axis.z * s;
    this.w = Math.cos(h);
    return this;
  }

  static fromAxisAngle(axis: Vec3, angle: number): Quat {
    return new Quat().setAxisAngle(axis, angle);
  }

  length(): number { return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z + this.w*this.w); }

  normalize(): this {
    const l = this.length();
    if (l > 1e-9) { this.x /= l; this.y /= l; this.z /= l; this.w /= l; }
    else { this.x = 0; this.y = 0; this.z = 0; this.w = 1; }
    return this;
  }

  conjugate(): this { this.x = -this.x; this.y = -this.y; this.z = -this.z; return this; }

  // q = a * b
  static mul(a: Quat, b: Quat, out = new Quat()): Quat {
    const ax = a.x, ay = a.y, az = a.z, aw = a.w;
    const bx = b.x, by = b.y, bz = b.z, bw = b.w;
    out.x = aw * bx + ax * bw + ay * bz - az * by;
    out.y = aw * by - ax * bz + ay * bw + az * bx;
    out.z = aw * bz + ax * by - ay * bx + az * bw;
    out.w = aw * bw - ax * bx - ay * by - az * bz;
    return out;
  }

  mul(b: Quat): this { return Quat.mul(this, b, this) as unknown as this; }

  /** Rotate v by this quaternion, out = this * v * this^-1 */
  rotate(v: Vec3, out = new Vec3()): Vec3 {
    // Optimized formula: v' = v + 2 * (q.xyz x (q.xyz x v + q.w * v))
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
    const vx = v.x, vy = v.y, vz = v.z;
    // t = 2 * (q.xyz x v)
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    // v' = v + qw * t + q.xyz x t
    out.x = vx + qw * tx + (qy * tz - qz * ty);
    out.y = vy + qw * ty + (qz * tx - qx * tz);
    out.z = vz + qw * tz + (qx * ty - qy * tx);
    return out;
  }

  /** Rotate v by inverse of this quaternion. */
  rotateInverse(v: Vec3, out = new Vec3()): Vec3 {
    const qx = -this.x, qy = -this.y, qz = -this.z, qw = this.w;
    const vx = v.x, vy = v.y, vz = v.z;
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    out.x = vx + qw * tx + (qy * tz - qz * ty);
    out.y = vy + qw * ty + (qz * tx - qx * tz);
    out.z = vz + qw * tz + (qx * ty - qy * tx);
    return out;
  }

  /** Integrate angular velocity omega (in world space) over dt into this quaternion. */
  integrate(omega: Vec3, dt: number): this {
    // dq/dt = 0.5 * omega_quat * q
    const ox = omega.x * 0.5, oy = omega.y * 0.5, oz = omega.z * 0.5;
    const dx = ox * this.w + oy * this.z - oz * this.y;
    const dy = oy * this.w + oz * this.x - ox * this.z;
    const dz = oz * this.w + ox * this.y - oy * this.x;
    const dw = -ox * this.x - oy * this.y - oz * this.z;
    this.x += dx * dt;
    this.y += dy * dt;
    this.z += dz * dt;
    this.w += dw * dt;
    this.normalize();
    return this;
  }

  /** Returns the axis-angle log of the quaternion as a vec3 (axis * angle). */
  toLog(out = new Vec3()): Vec3 {
    const wClamped = Math.max(-1, Math.min(1, this.w));
    const angle = 2 * Math.acos(wClamped);
    const s = Math.sqrt(1 - wClamped * wClamped);
    if (s < 1e-6) {
      out.set(this.x * 2, this.y * 2, this.z * 2);
    } else {
      const scale = angle / s;
      out.set(this.x * scale, this.y * scale, this.z * scale);
    }
    return out;
  }

  /** q = b * a^-1 (the rotation that takes a to b) */
  static diff(a: Quat, b: Quat, out = new Quat()): Quat {
    const ax = -a.x, ay = -a.y, az = -a.z, aw = a.w;
    const bx = b.x, by = b.y, bz = b.z, bw = b.w;
    out.x = bw * ax + bx * aw + by * az - bz * ay;
    out.y = bw * ay - bx * az + by * aw + bz * ax;
    out.z = bw * az + bx * ay - by * ax + bz * aw;
    out.w = bw * aw - bx * ax - by * ay - bz * az;
    return out;
  }

  /** Swing-twist decomposition. Returns [swing, twist] such that q = swing * twist, twist is around axis. */
  static swingTwist(q: Quat, axis: Vec3, outSwing = new Quat(), outTwist = new Quat()): [Quat, Quat] {
    // Project quaternion vector part onto axis
    const d = q.x * axis.x + q.y * axis.y + q.z * axis.z;
    const tx = axis.x * d, ty = axis.y * d, tz = axis.z * d;
    outTwist.set(tx, ty, tz, q.w).normalize();
    // swing = q * twist^-1
    Quat.mul(q, new Quat(-outTwist.x, -outTwist.y, -outTwist.z, outTwist.w), outSwing);
    return [outSwing, outTwist];
  }

  toArray(): [number, number, number, number] { return [this.x, this.y, this.z, this.w]; }
}

/**
 * Spherical linear interpolation between two unit quaternions.
 * t = 0 → a, t = 1 → b. Picks the shortest arc.
 */
export function slerp(a: Quat, b: Quat, t: number): Quat {
  let bx = b.x, by = b.y, bz = b.z, bw = b.w;
  let dot = a.x * bx + a.y * by + a.z * bz + a.w * bw;
  if (dot < 0) { bx = -bx; by = -by; bz = -bz; bw = -bw; dot = -dot; }
  if (dot > 0.9995) {
    return new Quat(
      a.x + (bx - a.x) * t,
      a.y + (by - a.y) * t,
      a.z + (bz - a.z) * t,
      a.w + (bw - a.w) * t,
    ).normalize();
  }
  const theta0 = Math.acos(Math.min(1, Math.max(-1, dot)));
  const theta = theta0 * t;
  const sinTheta = Math.sin(theta);
  const sinTheta0 = Math.sin(theta0);
  const s0 = Math.cos(theta) - dot * sinTheta / sinTheta0;
  const s1 = sinTheta / sinTheta0;
  return new Quat(
    a.x * s0 + bx * s1,
    a.y * s0 + by * s1,
    a.z * s0 + bz * s1,
    a.w * s0 + bw * s1,
  ).normalize();
}

/** Shortest-arc quaternion rotating `from` to `to` (unit-ish vectors). */
export function quatFromTo(from: Vec3, to: Vec3): Quat {
  const a = from.clone().normalize();
  const b = to.clone().normalize();
  const d = a.dot(b);
  if (d > 0.9999) return Quat.identity();
  if (d < -0.9999) {
    // 180°: pick any perpendicular axis
    const axis = Math.abs(a.x) < 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 1, 0);
    const perp = Vec3.cross(a, axis, new Vec3()).normalize();
    return new Quat(perp.x, perp.y, perp.z, 0);
  }
  const axis = Vec3.cross(a, b, new Vec3());
  const s = Math.sqrt((1 + d) * 2);
  const invS = 1 / s;
  return new Quat(axis.x * invS, axis.y * invS, axis.z * invS, s * 0.5).normalize();
}
