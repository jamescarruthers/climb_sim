import { Vec3 } from '../math/Vec3';

export type HoldType = 'jug' | 'crimp' | 'sloper' | 'pinch' | 'pocket';

/**
 * Hold friction & difficulty signatures (per spec §2.3).
 * The "difficulty" field is a baseline score influence the route uses.
 */
export const HOLD_SIGNATURES: Record<HoldType, {
  friction: number; depth: number; baseDifficulty: number; gripCost: number;
}> = {
  jug:    { friction: 0.8, depth: 0.06, baseDifficulty: 0.05, gripCost: 0.2 },
  crimp:  { friction: 0.6, depth: 0.015, baseDifficulty: 0.40, gripCost: 0.9 },
  sloper: { friction: 0.9, depth: 0.08, baseDifficulty: 0.35, gripCost: 0.6 },
  pinch:  { friction: 0.5, depth: 0.03, baseDifficulty: 0.30, gripCost: 0.7 },
  pocket: { friction: 0.4, depth: 0.025, baseDifficulty: 0.35, gripCost: 0.85 },
};

export interface Hold {
  id: string;
  position: Vec3;
  normal: Vec3;
  type: HoldType;
  depth: number;
  width: number;
  angle: number;       // surface angle from horizontal (degrees)
  friction: number;
  maxFingers: number;
  forHand: boolean;
  forFoot: boolean;
}

export function createHold(id: string, x: number, y: number, z: number,
                           type: HoldType = 'jug',
                           opts: Partial<Hold> = {}): Hold {
  const sig = HOLD_SIGNATURES[type];
  return {
    id,
    position: new Vec3(x, y, z),
    normal: new Vec3(0, 0, 1),
    type,
    depth: sig.depth,
    width: 0.08,
    angle: 90,
    friction: sig.friction,
    maxFingers: type === 'pocket' ? 2 : 4,
    forHand: true,
    forFoot: type === 'jug' || type === 'sloper' || type === 'pinch',
    ...opts,
  };
}
