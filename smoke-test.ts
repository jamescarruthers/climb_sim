/** Compare baseline vs one-shot-at-attach IK for a REACH scenario. */
import { Game } from './src/game/Game.ts';
import { Quat } from './src/math/Quat.ts';

const dt = 1 / 120;

function runTest(useIK: boolean, label: string) {
  console.log(`\n=== ${label} (useGrippedLimbIK = ${useIK}) ===`);
  const game = new Game(15);
  game.climber.useGrippedLimbIK = useIK;

  // Settle 5s
  for (let i = 0; i < 600; i++) game.tick(dt);
  let s = game.snapshot();
  let p = game.climber.bodies.get('pelvis')!.position;
  console.log(`settled: pelvis=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

  // Reach R-hand to h2
  const h2 = game.wall.holds.find(h => h.id === 'h2')!;
  game.selectLimb('R_hand'); game.selectHold(h2); game.requestReach();
  let attached_t = -1;
  for (let i = 0; i < 360; i++) {
    game.tick(dt);
    if (game.snapshot().onHolds.R_hand === 'h2' && attached_t < 0) attached_t = game.t;
  }
  s = game.snapshot();
  p = game.climber.bodies.get('pelvis')!.position;
  console.log(`after reach R→h2: attached_t=${attached_t.toFixed(2)} pelvis=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);

  // Settle 2s post-reach
  for (let i = 0; i < 240; i++) game.tick(dt);
  s = game.snapshot();
  p = game.climber.bodies.get('pelvis')!.position;
  console.log(`post-settle: pelvis=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) att=${Object.values(s.onHolds).filter(Boolean).length}/4 fails=${s.gripFailCount}`);
}

runTest(false, 'BASELINE');
runTest(true, 'ONE-SHOT IK AT ATTACH');
