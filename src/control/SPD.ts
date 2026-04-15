/**
 * Stable PD controller (Tan, Liu, Turk 2011). Evaluates PD error one step into
 * the future to avoid instability at high gains. Scalar version (single DOF).
 *
 * Derivation:
 *   τ_{n+1} = K_p (q_d - q_{n+1}) - K_d q̇_{n+1}
 *   q̇_{n+1} = q̇_n + (τ/M) Δt    (semi-implicit step)
 *   q_{n+1}  ≈ q_n + q̇_{n+1} Δt
 *
 *   ⇒ τ = M · (K_p · (q_d - q_n - q̇_n Δt) - K_d · q̇_n) /
 *           (M + K_d Δt + K_p Δt²)
 *
 * Note: (q_d - q_n - q̇_n Δt) is the position error evaluated at the *predicted*
 * state. The numerator does NOT have an extra `· Δt` on the K_p term — that
 * was a long-standing bug in this file that effectively reduced the
 * controller's output by a factor of (1/Δt), which made every joint feel
 * orders of magnitude weaker than its gain implied (legs collapsed under
 * body weight, etc.).
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
  // Position error at the predicted next state.
  const posErr = thetaTarget - theta - omega * dt;
  const omegaErr = omegaTarget - omega;
  const denom = inertia + kd * dt + kp * dt * dt;
  if (denom < 1e-9) return 0;
  return (kp * posErr + kd * omegaErr) * inertia / denom;
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
