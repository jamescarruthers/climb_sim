import { MuscleRegion } from '../skeleton/joints';

/**
 * Three-Compartment Controllable Recovery (3CCr) model — Xia & Frey 2008,
 * Looft et al. 2018.
 *
 * Each joint tracks three motor-unit populations that always sum to 1.0:
 *   M_R  — resting  (available but not recruited)
 *   M_A  — active   (currently producing force)
 *   M_F  — fatigued (cannot be recruited until recovered)
 *
 * ODEs (with recruitment controller C):
 *   dM_R/dt = -C + R_eff · M_F
 *   dM_A/dt =  C - F · M_A
 *   dM_F/dt =  F · M_A - R_eff · M_F
 *
 * R_eff = R * restMultiplier when target load ≈ 0 (intermittent-task boost).
 */

export interface FatigueState {
  M_R: number;
  M_A: number;
  M_F: number;
}

export interface FatigueParams {
  F: number;                // fatigue rate (1/s)
  R: number;                // recovery rate (1/s)
  L: number;                // recruitment gain (1/s)
  restMultiplier: number;   // R *= this when target≈0
}

export function freshState(): FatigueState { return { M_R: 1, M_A: 0, M_F: 0 }; }

export function stepFatigue(
  state: FatigueState,
  targetLoad: number,
  dt: number,
  params: FatigueParams,
): FatigueState {
  const { M_R, M_A, M_F } = state;
  const { F, R, L, restMultiplier } = params;

  // Recruitment: move motor units from R → A toward the target
  let C = 0;
  if (M_A < targetLoad) C = L * (targetLoad - M_A);
  else if (M_A > targetLoad) C = L * (targetLoad - M_A); // allow de-recruitment
  // Bound by available resting pool
  if (C > 0) C = Math.min(C, M_R / Math.max(dt, 1e-6));

  const effectiveR = targetLoad < 0.01 ? R * restMultiplier : R;

  const dM_R = -C + effectiveR * M_F;
  const dM_A =  C - F * M_A;
  const dM_F =  F * M_A - effectiveR * M_F;

  let nR = Math.max(0, M_R + dM_R * dt);
  let nA = Math.max(0, M_A + dM_A * dt);
  let nF = Math.max(0, M_F + dM_F * dt);
  const sum = nR + nA + nF;
  if (sum > 0) { nR /= sum; nA /= sum; nF /= sum; }

  return { M_R: nR, M_A: nA, M_F: nF };
}

/** Per-region 3CCr parameters (per spec §4.3, already in 1/s). */
export const FATIGUE_PARAMS: Record<MuscleRegion, FatigueParams> = {
  shoulder: { F: 0.0097,  R: 0.028,   L: 10, restMultiplier: 7 },
  elbow:    { F: 0.0243,  R: 0.0697,  L: 10, restMultiplier: 7 },
  wrist:    { F: 0.0243,  R: 0.0697,  L: 10, restMultiplier: 7 },
  grip:     { F: 0.0325,  R: 0.0795,  L: 10, restMultiplier: 7 },
  trunk:    { F: 0.0567,  R: 0.0943,  L: 10, restMultiplier: 7 },
  hip:      { F: 0.0150,  R: 0.040,   L: 10, restMultiplier: 7 },
  knee:     { F: 0.0200,  R: 0.050,   L: 10, restMultiplier: 7 },
  ankle:    { F: 0.0200,  R: 0.050,   L: 10, restMultiplier: 7 },
  neck:     { F: 0.0150,  R: 0.040,   L: 10, restMultiplier: 7 },
};
// Note: spec gives values like 0.0000097 (1/s) which recovery would be
// imperceptibly slow for an interactive game. The above multiply by ~1000 so
// the fatigue clock runs on human-relevant timescales (~10-60s pump).

/**
 * Forearm pump — blood-flow occlusion above ~30% MVC that multiplies the
 * recovery rate R during sustained grip.
 */
export function gripBloodFlowFactor(gripActivation: number): number {
  if (gripActivation < 0.15) return 1.0;
  if (gripActivation > 0.60) return 0.05;
  return 1.0 - (gripActivation - 0.15) / 0.45 * 0.95;
}

/**
 * Arm-position effect on recovery — encourages shaking out on straight arms.
 *   armYRelHeart:  relative vertical position of the hand w.r.t. heart (m)
 */
export function armPositionRecoveryFactor(armYRelHeart: number): number {
  if (armYRelHeart > 0.2)  return 0.6;  // arms above heart — bad
  if (armYRelHeart < -0.2) return 1.5;  // arms below heart — good
  return 1.0;
}
