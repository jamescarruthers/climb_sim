/** Full-sequence test for gravity comp architecture. */
import { Game } from './src/game/Game.ts';
import { Quat } from './src/math/Quat.ts';

(globalThis as any).__DEBUG_GRIP = true;

const game = new Game(15);
const dt = 1 / 120;
const tick = (n: number) => { for (let i = 0; i < n; i++) game.tick(dt); };

console.log('--- 10s settle ---');
for (let i = 0; i < 1200; i++) {
  game.tick(dt);
  if (i % 120 === 0 || i === 1199) {
    const p = game.climber.bodies.get('pelvis')!.position;
    const s = game.snapshot();
    console.log(`  t=${game.t.toFixed(1)} pelvis=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount} pump=${s.forearmPump.toFixed(3)}`);
  }
}

console.log('\n--- Reach R-hand → h2 ---');
const h2 = game.wall.holds.find(h => h.id === 'h2')!;
game.selectLimb('R_hand'); game.selectHold(h2); game.requestReach();
let attached_t = -1;
for (let i = 0; i < 360; i++) {
  game.tick(dt);
  if (game.snapshot().onHolds.R_hand === 'h2' && attached_t < 0) attached_t = game.t;
}
let s = game.snapshot();
console.log(`  attached_t=${attached_t.toFixed(2)} att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Reach L-hand → h1 (cross-body reach) ---');
const h1 = game.wall.holds.find(h => h.id === 'h1')!;
game.selectLimb('L_hand'); game.selectHold(h1); game.requestReach();
attached_t = -1;
for (let i = 0; i < 360; i++) {
  game.tick(dt);
  if (game.snapshot().onHolds.L_hand === 'h1' && attached_t < 0) attached_t = game.t;
}
s = game.snapshot();
console.log(`  attached_t=${attached_t.toFixed(2)} att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Hold SPACE 1s, release ---');
game.setLegDrive(1.0);
for (let i = 0; i < 120; i++) game.tick(dt);
let p = game.climber.bodies.get('pelvis')!.position;
console.log(`  end drive: pelvis_y=${p.y.toFixed(3)}`);
game.setLegDrive(0);
for (let i = 0; i < 120; i++) game.tick(dt);
p = game.climber.bodies.get('pelvis')!.position;
s = game.snapshot();
console.log(`  after release: pelvis_y=${p.y.toFixed(3)} att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);
