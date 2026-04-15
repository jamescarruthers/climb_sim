import { Game } from './src/game/Game.ts';

(globalThis as any).__DEBUG_GRIP = true;

const game = new Game(15);
const dt = 1 / 120;
const tick = (n: number) => { for (let i = 0; i < n; i++) game.tick(dt); };

console.log('--- 8s settle ---');
tick(960);
let s = game.snapshot();
console.log(`pelvis=${game.climber.bodies.get('pelvis')!.position.y.toFixed(3)} attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Reach R-hand → h2 ---');
const h2 = game.wall.holds.find(h => h.id === 'h2')!;
game.selectLimb('R_hand'); game.selectHold(h2); game.requestReach();
let attached = -1;
for (let i = 0; i < 360; i++) {
  game.tick(dt);
  if (game.snapshot().onHolds.R_hand === 'h2' && attached < 0) attached = game.t;
}
s = game.snapshot();
console.log(`attached at t=${attached.toFixed(2)}s, attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Hold SPACE for 1s, release ---');
game.setLegDrive(1.0);
tick(120);
let p = game.climber.bodies.get('pelvis')!.position;
console.log(`At end of drive: pelvis=${p.y.toFixed(3)}`);
game.setLegDrive(0);
tick(120);
p = game.climber.bodies.get('pelvis')!.position;
console.log(`After release: pelvis=${p.y.toFixed(3)} attached=${Object.values(game.snapshot().onHolds).filter(Boolean).length}/4 fails=${game.snapshot().gripFailCount}`);
