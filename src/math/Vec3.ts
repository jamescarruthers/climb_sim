/**
 * Mutable 3D vector. Most operations mutate `this` and return it for chaining,
 * to avoid per-frame allocation in the physics loop.
 */
export class Vec3 {
  x: number;
  y: number;
  z: number;

  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  static zero(): Vec3 { return new Vec3(0, 0, 0); }
  static up(): Vec3 { return new Vec3(0, 1, 0); }
  static forward(): Vec3 { return new Vec3(0, 0, 1); }
  static right(): Vec3 { return new Vec3(1, 0, 0); }

  set(x: number, y: number, z: number): this { this.x = x; this.y = y; this.z = z; return this; }
  copy(v: Vec3): this { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
  clone(): Vec3 { return new Vec3(this.x, this.y, this.z); }

  add(v: Vec3): this { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
  sub(v: Vec3): this { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
  scale(s: number): this { this.x *= s; this.y *= s; this.z *= s; return this; }
  addScaled(v: Vec3, s: number): this { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; }
  negate(): this { this.x = -this.x; this.y = -this.y; this.z = -this.z; return this; }

  dot(v: Vec3): number { return this.x * v.x + this.y * v.y + this.z * v.z; }

  cross(v: Vec3): this {
    const ax = this.x, ay = this.y, az = this.z;
    this.x = ay * v.z - az * v.y;
    this.y = az * v.x - ax * v.z;
    this.z = ax * v.y - ay * v.x;
    return this;
  }

  lengthSq(): number { return this.x * this.x + this.y * this.y + this.z * this.z; }
  length(): number { return Math.sqrt(this.lengthSq()); }
  distanceTo(v: Vec3): number { const dx = this.x - v.x, dy = this.y - v.y, dz = this.z - v.z; return Math.sqrt(dx*dx+dy*dy+dz*dz); }

  normalize(): this {
    const l = this.length();
    if (l > 1e-9) { this.x /= l; this.y /= l; this.z /= l; }
    else { this.x = 0; this.y = 0; this.z = 0; }
    return this;
  }

  lerp(v: Vec3, t: number): this {
    this.x += (v.x - this.x) * t;
    this.y += (v.y - this.y) * t;
    this.z += (v.z - this.z) * t;
    return this;
  }

  // Static helpers that allocate
  static add(a: Vec3, b: Vec3, out = new Vec3()): Vec3 { out.x = a.x + b.x; out.y = a.y + b.y; out.z = a.z + b.z; return out; }
  static sub(a: Vec3, b: Vec3, out = new Vec3()): Vec3 { out.x = a.x - b.x; out.y = a.y - b.y; out.z = a.z - b.z; return out; }
  static cross(a: Vec3, b: Vec3, out = new Vec3()): Vec3 {
    const x = a.y * b.z - a.z * b.y;
    const y = a.z * b.x - a.x * b.z;
    const z = a.x * b.y - a.y * b.x;
    out.x = x; out.y = y; out.z = z; return out;
  }
  static scale(a: Vec3, s: number, out = new Vec3()): Vec3 { out.x = a.x * s; out.y = a.y * s; out.z = a.z * s; return out; }
  static dot(a: Vec3, b: Vec3): number { return a.x * b.x + a.y * b.y + a.z * b.z; }

  toArray(): [number, number, number] { return [this.x, this.y, this.z]; }
}
