/**
 * Stable PD controller (Tan, Liu, Turk 2011). Evaluates PD error one step into
 * the future to avoid instability at high gains. Scalar version (single DOF).
 */
export function stablePDTorque(
  theta: number,
  omega: number,
  thetaTarget: number,
  omegaTarget: number,
  kp: number,
  kd: number,
  dt: number,
  inertia: number,
): number {
  const thetaErr = thetaTarget - theta - omega * dt;
  const omegaErr = omegaTarget - omega;
  const denom = inertia + kd * dt + kp * dt * dt;
  if (denom < 1e-9) return 0;
  return (kp * thetaErr * dt + kd * omegaErr) * inertia / denom;
}

/** 3D SPD applied axis-wise in a common (body-local) frame. */
export function stablePDTorqueVec(
  theta: [number, number, number],
  omega: [number, number, number],
  thetaTarget: [number, number, number],
  omegaTarget: [number, number, number],
  kp: number,
  kd: number,
  dt: number,
  inertia: [number, number, number],
  out: [number, number, number] = [0, 0, 0],
): [number, number, number] {
  for (let i = 0; i < 3; i++) {
    out[i] = stablePDTorque(theta[i], omega[i], thetaTarget[i], omegaTarget[i], kp, kd, dt, inertia[i]);
  }
  return out;
}
