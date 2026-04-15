import { rad } from '../math/util';
import { MTGParams, DEFAULT_MTG } from './MTG';
import { MuscleRegion } from '../skeleton/joints';

/**
 * Peak isometric torques and velocity limits (per spec §3.4), indexed by
 * muscle region. These describe the MAXIMUM output torque the joint can
 * produce when fully fresh and at the optimal angle. Real output will be
 * reduced by fatigue and activation dynamics.
 */

export const REGION_TMAX: Record<MuscleRegion, number> = {
  hip:      200,  // extension-dominant
  knee:     180,
  ankle:    120,
  shoulder:  80,
  elbow:     60,
  wrist:     30,
  grip:      50,  // ~400 N of fingertip force at typical moment arm
  trunk:    200,
  neck:      30,
};

export const REGION_OMEGA_MAX: Record<MuscleRegion, number> = {
  hip: 8, knee: 10, ankle: 8,
  shoulder: 10, elbow: 10, wrist: 6, grip: 4,
  trunk: 6, neck: 6,
};

export function mtgParamsForRegion(region: MuscleRegion,
                                   thetaMin: number, thetaMax: number,
                                   thetaOpt = (thetaMin + thetaMax) * 0.5): MTGParams {
  return {
    tMax: REGION_TMAX[region],
    omegaMax: REGION_OMEGA_MAX[region],
    thetaOpt,
    thetaMin,
    thetaMax,
    ...DEFAULT_MTG,
  };
}
