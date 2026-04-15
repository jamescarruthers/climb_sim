import { Game } from './src/game/Game.ts';

(globalThis as any).__DEBUG_GRIP = true;

const game = new Game(15);
const dt = 1 / 120;

console.log('--- 15s settle stability ---');
for (let i = 0; i < 1800; i++) {
  game.tick(dt);
  if (i % 180 === 0) {
    const s = game.snapshot();
    console.log(`t=${game.t.toFixed(1)}s pelvis_y=${game.climber.bodies.get('pelvis')!.position.y.toFixed(3)} attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount} pump=${s.forearmPump.toFixed(3)}`);
  }
}
const s = game.snapshot();
console.log(`\nFinal: fell=${s.fell} fails=${s.gripFailCount}`);
