import { Vec3 } from '../math/Vec3';
import { Hold, createHold } from './Hold';

/**
 * A climbable wall. Plane angle: 0 = vertical, positive = overhung.
 * Wall surface is the z = 0 plane in the climber's local frame; overhang is
 * modelled by tilting the entire wall plane by `angle` about the x-axis, so
 * holds at high y move outward in +z.
 */
export interface Wall {
  angle: number;    // degrees; positive = overhung
  width: number;
  height: number;
  holds: Hold[];
}

/**
 * Transform a hold's wall-local (u, v) position to world coordinates given the
 * wall's angle. u is the horizontal axis, v is vertical along the wall
 * surface. For overhang angle θ:
 *   world.x = u
 *   world.y = v cos θ
 *   world.z = v sin θ        (since overhang leans toward climber = +z)
 */
export function uvToWorld(u: number, v: number, wall: Wall): Vec3 {
  const t = wall.angle * Math.PI / 180;
  return new Vec3(u, v * Math.cos(t), v * Math.sin(t));
}

export function wallNormal(wall: Wall): Vec3 {
  const t = wall.angle * Math.PI / 180;
  // Surface: in (x, y cosθ + z sinθ) plane. Normal points outward (+z when θ=0).
  // For overhang, the normal tilts toward climber and downward.
  return new Vec3(0, -Math.sin(t), Math.cos(t));
}

/** Simple default route: a zig-zag of holds with varied types. */
export function buildDefaultWall(angle = 15): Wall {
  const wall: Wall = { angle, width: 4, height: 8, holds: [] };
  const add = (id: string, u: number, v: number, type: Parameters<typeof createHold>[4] = 'jug', opts?: Parameters<typeof createHold>[5]) => {
    const p = uvToWorld(u, v, wall);
    wall.holds.push(createHold(id, p.x, p.y, p.z, type, { ...opts, normal: wallNormal(wall) }));
  };

  // Start jugs
  add('start_L', -0.25, 0.8, 'jug');
  add('start_R',  0.25, 0.8, 'jug');
  // Feet starters
  add('foot_L0', -0.30, 0.20, 'jug', { forHand: false, forFoot: true });
  add('foot_R0',  0.30, 0.20, 'jug', { forHand: false, forFoot: true });

  // Main line (alternating hand / foot / crimp)
  const line: Array<[string, number, number, Parameters<typeof createHold>[4], boolean?]> = [
    ['h1',  -0.30, 1.35, 'jug'],
    ['h2',   0.35, 1.75, 'crimp'],
    ['h3',  -0.28, 2.20, 'sloper'],
    ['h4',   0.30, 2.70, 'pinch'],
    ['h5',  -0.38, 3.15, 'crimp'],
    ['h6',   0.20, 3.60, 'jug'],
    ['h7',  -0.15, 4.10, 'pocket'],
    ['h8',   0.35, 4.55, 'crimp'],
    ['h9',  -0.20, 5.00, 'jug'],
    ['top',  0.00, 5.60, 'jug'],
  ];
  for (const [id, u, v, t] of line) add(id, u, v, t);

  // Feet along the route
  for (let i = 0; i < 6; i++) {
    const y = 1.0 + i * 0.7;
    add(`foot_L${i + 1}`, -0.35, y, 'jug', { forHand: false, forFoot: true });
    add(`foot_R${i + 1}`,  0.35, y, 'jug', { forHand: false, forFoot: true });
  }

  return wall;
}
