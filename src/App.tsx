import React, { useEffect, useRef, useState } from 'react';
import { Game, GameSnapshot } from './game/Game';
import { Renderer } from './render/Renderer';
import { HUD } from './ui/HUD';
import { Controls } from './ui/Controls';
import { LimbGrip } from './climber/Climber';

/**
 * Top-level React component. Creates the Game + Renderer, runs the
 * animation loop, and mirrors game state into React via a ticking snapshot.
 */
export function App() {
  const containerRef = useRef<HTMLDivElement>(null);
  const gameRef = useRef<Game | null>(null);
  const rendererRef = useRef<Renderer | null>(null);
  const [snap, setSnap] = useState<GameSnapshot | null>(null);
  const [, forceUpdate] = useState(0);

  useEffect(() => {
    if (!containerRef.current) return;
    const game = new Game(15);
    const renderer = new Renderer(containerRef.current);
    renderer.buildWall(game);
    renderer.buildClimber(game);
    renderer.setGame(game);

    renderer.setHoldPickHandler((hold) => {
      game.selectHold(hold);
      forceUpdate(x => x + 1);
    });

    gameRef.current = game;
    rendererRef.current = renderer;

    // Held WASD keys produce a continuous body-lean bias.
    const heldKeys = new Set<string>();

    let lastT = performance.now();
    let uiAccum = 0;
    let raf = 0;
    const tick = () => {
      const now = performance.now();
      const dt = (now - lastT) / 1000;
      lastT = now;

      // Translate held WASD into a smoothed lean target.
      const lx = (heldKeys.has('d') ? 1 : 0) - (heldKeys.has('a') ? 1 : 0);
      const ly = (heldKeys.has('w') ? 1 : 0) - (heldKeys.has('s') ? 1 : 0);
      // Smoothed approach so lean ramps in/out instead of snapping.
      const cur = game.bodyLean;
      const tau = 5;  // 1/s — higher = snappier
      const a = 1 - Math.exp(-dt * tau);
      game.setLean(cur.x + (lx - cur.x) * a, cur.y + (ly - cur.y) * a);

      game.tick(dt);
      const s = game.snapshot();
      renderer.applySnapshot(s);
      renderer.render();

      uiAccum += dt;
      if (uiAccum >= 0.05) {
        uiAccum = 0;
        setSnap(s);
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);

    const onKeyDown = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (['w', 'a', 's', 'd'].includes(k)) {
        heldKeys.add(k);
        return;
      }
      switch (e.key) {
        case '1': game.selectLimb('L_hand'); break;
        case '2': game.selectLimb('R_hand'); break;
        case '3': game.selectLimb('L_foot'); break;
        case '4': game.selectLimb('R_foot'); break;
        case 'f': case 'F': game.requestReach(); break;
        case 'g': case 'G': game.requestGrip(); break;
        case 'x': case 'X': game.requestRelease(); break;
        case 'r': case 'R': game.reset(); break;
      }
      forceUpdate(x => x + 1);
    };
    const onKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (['w', 'a', 's', 'd'].includes(k)) heldKeys.delete(k);
    };
    const onBlur = () => heldKeys.clear();
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    window.addEventListener('blur', onBlur);

    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      window.removeEventListener('blur', onBlur);
      renderer.dispose();
    };
  }, []);

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <div ref={containerRef} style={{ position: 'absolute', inset: 0 }} />
      {snap && <HUD snap={snap} />}
      {snap && gameRef.current && (
        <Controls game={gameRef.current} snap={snap} onAction={() => forceUpdate(x => x + 1)} />
      )}
      <div style={{
        position: 'absolute', top: 12, right: 12,
        background: 'rgba(10,10,16,0.82)', padding: 10, borderRadius: 8,
        fontSize: 11, color: '#aaa', maxWidth: 240,
      }}>
        <b style={{ color: '#eee' }}>Hold Legend</b>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr auto', gap: 3, marginTop: 6 }}>
          <Legend color="#4caf50" label="Jug — forgiving, low pump" />
          <Legend color="#f44336" label="Crimp — small, high load" />
          <Legend color="#2196f3" label="Sloper — friction-dependent" />
          <Legend color="#ffeb3b" label="Pinch — squeeze required" />
          <Legend color="#9c27b0" label="Pocket — high per-finger" />
        </div>
      </div>
    </div>
  );
}

function Legend({ color, label }: { color: string; label: string }) {
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8, gridColumn: '1 / span 2' }}>
      <span style={{ width: 10, height: 10, background: color, borderRadius: '50%', display: 'inline-block', flexShrink: 0 }} />
      <span>{label}</span>
    </div>
  );
}
