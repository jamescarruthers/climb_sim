export class Vec2 {
  x: number;
  y: number;
  constructor(x = 0, y = 0) { this.x = x; this.y = y; }

  set(x: number, y: number): this { this.x = x; this.y = y; return this; }
  clone(): Vec2 { return new Vec2(this.x, this.y); }
  add(v: Vec2): this { this.x += v.x; this.y += v.y; return this; }
  sub(v: Vec2): this { this.x -= v.x; this.y -= v.y; return this; }
  scale(s: number): this { this.x *= s; this.y *= s; return this; }
  dot(v: Vec2): number { return this.x * v.x + this.y * v.y; }
  lengthSq(): number { return this.x * this.x + this.y * this.y; }
  length(): number { return Math.sqrt(this.lengthSq()); }
  normalize(): this { const l = this.length(); if (l > 1e-9) { this.x /= l; this.y /= l; } return this; }

  static sub(a: Vec2, b: Vec2): Vec2 { return new Vec2(a.x - b.x, a.y - b.y); }
  static cross(a: Vec2, b: Vec2): number { return a.x * b.y - a.y * b.x; }
}
