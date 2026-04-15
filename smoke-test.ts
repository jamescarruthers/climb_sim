/**
 * End-to-end Phase 3 test: settle, reach R hand to h2 (no pre-lean),
 * then reach L hand to h1, then verify multi-step climb works.
 */
import { Game } from './src/game/Game.ts';

const game = new Game(15);
const dt = 1 / 120;
const tick = (n: number) => { for (let i = 0; i < n; i++) game.tick(dt); };

// Settle
tick(300);
console.log(`Settled. attached=${Object.values(game.snapshot().onHolds).filter(Boolean).length}/4`);

// Reach R hand → h2
const h2 = game.wall.holds.find(h => h.id === 'h2')!;
game.selectLimb('R_hand');
game.selectHold(h2);
game.requestReach();
tick(180);  // 1.5s
let s = game.snapshot();
console.log(`After reach R→h2: R_hand=${s.onHolds.R_hand} attached=${Object.values(s.onHolds).filter(Boolean).length}/4`);

// Reach L hand → h1 (cross-body for L hand → moderate)
tick(60);
const h1 = game.wall.holds.find(h => h.id === 'h1')!;
game.selectLimb('L_hand');
game.selectHold(h1);
game.requestReach();
tick(240);  // 2s
s = game.snapshot();
console.log(`After reach L→h1: L_hand=${s.onHolds.L_hand} attached=${Object.values(s.onHolds).filter(Boolean).length}/4`);

// Lean and check fatigue
console.log(`\nFinal: attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount} pump=${s.forearmPump.toFixed(3)} fell=${s.fell}`);
console.log(`Holds: ${JSON.stringify(s.onHolds)}`);
