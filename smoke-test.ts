import { Game } from './src/game/Game.ts';

(globalThis as any).__DEBUG_GRIP = true;

const game = new Game(15);
const dt = 1 / 120;
const tick = (n: number) => { for (let i = 0; i < n; i++) game.tick(dt); };

console.log('--- 5s settle ---');
tick(600);
let s = game.snapshot();
console.log(`pelvis_y=${game.climber.bodies.get('pelvis')!.position.y.toFixed(3)} CoM=(${s.com.map(v=>v.toFixed(2)).join(',')}) attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Reach R→h2 ---');
const h2 = game.wall.holds.find(h => h.id === 'h2')!;
game.selectLimb('R_hand'); game.selectHold(h2); game.requestReach();
tick(180);
s = game.snapshot();
console.log(`R_hand=${s.onHolds.R_hand} CoM=(${s.com.map(v=>v.toFixed(2)).join(',')}) attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Lean left, then reach L→h1 ---');
game.setLean(-0.6, 0.2);
tick(120);
const h1 = game.wall.holds.find(h => h.id === 'h1')!;
game.selectLimb('L_hand'); game.selectHold(h1); game.requestReach();
tick(240);
s = game.snapshot();
console.log(`L_hand=${s.onHolds.L_hand} CoM=(${s.com.map(v=>v.toFixed(2)).join(',')}) attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log('\n--- Center lean, reach R foot up ---');
game.setLean(0, 0);
tick(120);
const fR1 = game.wall.holds.find(h => h.id === 'foot_R1')!;
game.selectLimb('R_foot'); game.selectHold(fR1); game.requestReach();
tick(240);
s = game.snapshot();
console.log(`R_foot=${s.onHolds.R_foot} CoM=(${s.com.map(v=>v.toFixed(2)).join(',')}) attached=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

console.log(`\nFinal: pump=${s.forearmPump.toFixed(3)} fell=${s.fell} fails=${s.gripFailCount}`);
console.log(`Holds: ${JSON.stringify(s.onHolds)}`);
