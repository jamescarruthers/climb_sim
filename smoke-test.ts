import { Game } from './src/game/Game.ts';

(globalThis as any).__DEBUG_GRIP = true;

const game = new Game(15);
const dt = 1 / 120;
// Run for 10 simulated seconds
for (let i = 0; i < 1200; i++) {
  game.tick(dt);
  if (i % 60 === 0) {
    const forces = game.climber.grips.map(g => g.constraint ? g.constraint.impliedForce(dt) : 0);
    const s = game.snapshot();
    console.log(`t=${game.t.toFixed(2)}s ` +
      `CoM=(${s.com.map(v => v.toFixed(2)).join(',')}) ` +
      `forces=${forces.map(f => f.toFixed(0).padStart(5)).join(' ')} ` +
      `pump=${s.forearmPump.toFixed(3)} ` +
      `lact=${s.lactate.toFixed(3)} ` +
      `attached=${Object.values(s.onHolds).filter(Boolean).length}/4`);
  }
}
console.log('\nFinal snapshot:');
const s = game.snapshot();
console.log(`  fall=${s.fell} top=${s.topReached} fails=${s.gripFailCount}`);
