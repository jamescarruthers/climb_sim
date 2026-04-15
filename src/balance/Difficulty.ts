/**
 * Per-move difficulty scoring (spec §8). Aggregates torque demand, stability
 * margin, friction margin, and reach into a [0, 1] score.
 */
export function computeMoveDifficulty(args: {
  jointTorques: Map<string, number>;
  jointMaxTorques: Map<string, number>;
  stabilityMargin: number;   // m, signed
  frictionMargins: number[]; // per-contact (μ·Fn - |Ft|) / (μ·Fn)
  reachRatio: number;        // [0,1]
}): number {
  const { jointTorques, jointMaxTorques, stabilityMargin, frictionMargins, reachRatio } = args;

  // Torque demand (quadratic, averaged)
  let torqueCost = 0;
  let n = 0;
  for (const [joint, tau] of jointTorques) {
    const max = jointMaxTorques.get(joint) ?? 1;
    if (max > 0) {
      const r = Math.min(2, Math.abs(tau) / max);
      torqueCost += r * r;
      n++;
    }
  }
  torqueCost = n > 0 ? torqueCost / n : 0;

  const stabilityCost = Math.max(0, 1 - stabilityMargin / 0.3);
  const frictionCost  = frictionMargins.length
    ? frictionMargins.reduce((s, m) => s + Math.max(0, 1 - m), 0) / frictionMargins.length
    : 0;
  const reachCost = reachRatio > 0.85 ? (reachRatio - 0.85) / 0.15 : 0;

  const score = 0.4 * torqueCost + 0.25 * stabilityCost + 0.2 * frictionCost + 0.15 * reachCost;
  return Math.max(0, Math.min(1, score));
}

export function classifyDifficulty(score: number): string {
  if (score < 0.2) return 'Easy';
  if (score < 0.4) return 'Moderate';
  if (score < 0.6) return 'Hard';
  if (score < 0.8) return 'Very Hard';
  return 'Desperate';
}
