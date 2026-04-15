import { clamp } from '../math/util';

/**
 * Joint Torque Generator — models the combined capability of all muscles
 * crossing a joint DOF as a single actuator with torque-angle (f_TA),
 * torque-velocity (f_TV, Hill curve), activation dynamics, and passive
 * resistance near limits.
 *
 *   τ = a(t) · T_max · f_TA(θ) · f_TV(ω) + τ_passive(θ)
 */

export interface MTGParams {
  tMax: number;      // peak isometric torque at θ_opt (N·m)
  omegaMax: number;  // max unloaded angular velocity (rad/s)
  thetaOpt: number;  // optimal angle (rad)
  faWidth: number;   // std-dev of torque-angle bell (rad)
  thetaMin: number;  // joint min angle (rad)
  thetaMax: number;  // joint max angle (rad)
  passiveStiffness: number;
  passiveExp: number;
  tauRise: number;   // activation rise time-constant (s)
  tauFall: number;   // activation fall time-constant (s)
}

export const DEFAULT_MTG: Omit<MTGParams, 'tMax' | 'omegaMax' | 'thetaOpt' | 'thetaMin' | 'thetaMax'> = {
  faWidth: 0.8,
  passiveStiffness: 5.0,
  passiveExp: 10.0,
  tauRise: 0.02,
  tauFall: 0.06,
};

export function torqueAngleFactor(theta: number, thetaOpt: number, width: number): number {
  const x = (theta - thetaOpt) / width;
  return Math.exp(-x * x);
}

export function torqueVelocityFactor(omega: number, omegaMax: number): number {
  if (omegaMax <= 0) return 1;
  if (omega <= 0) {
    // Concentric — Hill hyperbola
    // Force(v) = (v_max - v) / (v_max + c*v), with v = -omega (shortening positive)
    const v = -omega;
    if (v >= omegaMax) return 0;
    return (omegaMax - v) / (omegaMax + 4.0 * v);
  } else {
    // Eccentric — capped at 1.5
    return Math.min(1.5, 1.0 + 0.5 * omega / omegaMax);
  }
}

export function passiveTorque(theta: number, thetaMin: number, thetaMax: number,
                              stiffness: number, expScale: number): number {
  let torque = 0;
  if (theta < thetaMin + 0.1) {
    torque = stiffness * Math.exp(expScale * (thetaMin + 0.1 - theta)) - stiffness;
  }
  if (theta > thetaMax - 0.1) {
    torque = -(stiffness * Math.exp(expScale * (theta - (thetaMax - 0.1))) - stiffness);
  }
  return torque;
}

export function updateActivation(a: number, u: number, dt: number, tauRise: number, tauFall: number): number {
  const tau = u > a ? tauRise : tauFall;
  return a + (u - a) * (1 - Math.exp(-dt / tau));
}

/**
 * Instantiatable MTG with activation state.
 */
export class MTG {
  params: MTGParams;
  activation = 0;   // current activation in [0,1]
  targetU = 0;      // current desired u (set by controller / player)

  constructor(params: MTGParams) {
    this.params = params;
  }

  /** Update activation dynamics and compute output torque. */
  update(theta: number, omega: number, dt: number, availableStrength = 1.0): number {
    const p = this.params;
    this.activation = updateActivation(this.activation, this.targetU, dt, p.tauRise, p.tauFall);
    const a = clamp(this.activation, 0, 1) * clamp(availableStrength, 0, 1);

    const fa = torqueAngleFactor(theta, p.thetaOpt, p.faWidth);
    const fv = torqueVelocityFactor(omega, p.omegaMax);
    const active = a * p.tMax * fa * fv;
    const passive = passiveTorque(theta, p.thetaMin, p.thetaMax, p.passiveStiffness, p.passiveExp);
    return active + passive;
  }

  /** Set desired activation level in [0,1]. */
  setInput(u: number): void { this.targetU = clamp(u, 0, 1); }
}
