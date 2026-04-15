/**
 * Three-tier metabolic energy model (per spec §5). Global pools — not per-joint.
 *
 *   phosphocreatine — burst pool, depletes in ~10s, recovers in ~30s
 *   glycolytic      — sustained, produces lactate
 *   aerobicCapacity — current aerobic output fraction (slow ramp-up)
 *   lactate         — metabolic byproduct; above threshold scales down T_max.
 */
import { clamp } from '../math/util';

export interface EnergyState {
  phosphocreatine: number;
  glycolytic: number;
  aerobicCapacity: number;
  lactate: number;
}

export function freshEnergy(): EnergyState {
  return { phosphocreatine: 1, glycolytic: 1, aerobicCapacity: 0, lactate: 0 };
}

const PCR_DEPLETION_RATE   = 0.10;   // per second at full output
const PCR_RECOVERY_RATE    = 0.017;
const GLYCO_DEPLETION_RATE = 0.005;
const GLYCO_LACTATE_RATE   = 2.0;    // relative production
const AEROBIC_RAMP_TAU     = 35;
const AEROBIC_MAX_RATE     = 0.4;    // max sustainable fraction of peak
const LACTATE_CLEARANCE    = 0.003;
const LACTATE_THRESHOLD    = 0.6;
const LACTATE_MAX          = 1.0;

/**
 * Update energy pools for one timestep.
 * @param totalActivation   Global load proxy (sum of activations / N), in [0,1+].
 */
export function stepEnergy(state: EnergyState, totalActivation: number, dt: number): EnergyState {
  const load = clamp(totalActivation, 0, 1.5);

  // Aerobic ramps toward AEROBIC_MAX_RATE with time-constant AEROBIC_RAMP_TAU
  // (or down toward 0 when load is low).
  const aerobicTarget = Math.min(load, AEROBIC_MAX_RATE);
  const aerobic = state.aerobicCapacity + (aerobicTarget - state.aerobicCapacity) * (1 - Math.exp(-dt / AEROBIC_RAMP_TAU));

  // Demand not met by aerobic goes first to PCr, then glycolytic.
  let unmet = Math.max(0, load - aerobic);
  const pcrDraw = Math.min(unmet, state.phosphocreatine * 10); // effectively unbounded in steady state
  unmet -= pcrDraw * 0;  // just reuse below

  // PCr depletes at rate proportional to PCR_DEPLETION_RATE * load_above_aerobic,
  // recovers exponentially when rest.
  let pcr = state.phosphocreatine - PCR_DEPLETION_RATE * unmet * dt;
  pcr += PCR_RECOVERY_RATE * (1 - pcr) * dt * Math.max(0, 1 - load);
  pcr = clamp(pcr, 0, 1);

  // When PCr is low, shift residual demand into glycolytic.
  const pcrAvailable = pcr;
  const glyDraw = Math.max(0, unmet - pcrAvailable);

  let gly = state.glycolytic - GLYCO_DEPLETION_RATE * glyDraw * dt;
  gly += 0.01 * (1 - gly) * dt * (1 - load); // slow recovery while low load
  gly = clamp(gly, 0, 1);

  // Lactate production and clearance
  let lact = state.lactate + GLYCO_LACTATE_RATE * GLYCO_DEPLETION_RATE * glyDraw * dt;
  lact -= LACTATE_CLEARANCE * dt;
  lact = clamp(lact, 0, LACTATE_MAX);

  return { phosphocreatine: pcr, glycolytic: gly, aerobicCapacity: aerobic, lactate: lact };
}

/** Returns the global strength multiplier from lactate accumulation. */
export function lactateStrengthScale(lactate: number): number {
  if (lactate <= LACTATE_THRESHOLD) return 1.0;
  return 1.0 - 0.5 * (lactate - LACTATE_THRESHOLD) / (LACTATE_MAX - LACTATE_THRESHOLD);
}
