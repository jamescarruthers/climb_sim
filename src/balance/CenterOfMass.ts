import { Vec3 } from '../math/Vec3';
import { Vec2 } from '../math/Vec2';
import { convexHull2D } from '../math/util';
import { RigidBody } from '../physics/RigidBody';

export function computeCoM(bodies: RigidBody[]): Vec3 {
  let m = 0;
  const out = new Vec3();
  for (const b of bodies) {
    if (b.mass <= 0) continue;
    out.addScaled(b.position, b.mass);
    m += b.mass;
  }
  if (m > 0) out.scale(1 / m);
  return out;
}

/** Project active contact points onto the wall plane (XY) and take convex hull. */
export function supportPolygon(contactPoints: Vec3[]): Vec2[] {
  const pts = contactPoints.map(p => new Vec2(p.x, p.y));
  return convexHull2D(pts);
}

/** Signed distance from 2D point to polygon (positive inside). */
export function stabilityMargin(com2D: Vec2, polygon: Vec2[]): number {
  if (polygon.length < 3) return -Infinity;
  let minDist = Infinity;
  for (let i = 0; i < polygon.length; i++) {
    const j = (i + 1) % polygon.length;
    const ex = polygon[j].x - polygon[i].x;
    const ey = polygon[j].y - polygon[i].y;
    const nLen = Math.hypot(ex, ey);
    if (nLen < 1e-9) continue;
    const nx = -ey / nLen;
    const ny =  ex / nLen;
    const dx = com2D.x - polygon[i].x;
    const dy = com2D.y - polygon[i].y;
    const d = dx * nx + dy * ny;
    if (d < minDist) minDist = d;
  }
  return minDist;
}
