import { Game } from './src/game/Game.ts';

(globalThis as any).__DEBUG_GRIP = true;

const game = new Game(15);
const dt = 1 / 120;

for (let i = 0; i < 600; i++) game.tick(dt);

console.log('Settled.');
const h2 = game.wall.holds.find(h => h.id === 'h2')!;
console.log(`h2 at (${h2.position.x.toFixed(2)}, ${h2.position.y.toFixed(2)}, ${h2.position.z.toFixed(2)})`);

game.selectLimb('R_hand');
game.selectHold(h2);
game.requestReach();
console.log('Reach requested.');

for (let i = 0; i < 360; i++) {
  game.tick(dt);
  if (i % 12 === 0) {
    const tip = game.climber.limbTipWorld('R_hand');
    const dist = tip.clone().sub(h2.position).length();
    const s = game.snapshot();
    console.log(`  t=${game.t.toFixed(2)} tip=(${tip.x.toFixed(2)},${tip.y.toFixed(2)},${tip.z.toFixed(2)}) dist=${dist.toFixed(3)} R_hand=${s.onHolds.R_hand ?? '—'} reachActive=${s.reachTargets.R_hand ?? '—'}`);
  }
}
