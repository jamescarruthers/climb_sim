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

    let lastT = performance.now();
    let uiAccum = 0;
    let raf = 0;
    const tick = () => {
      const now = performance.now();
      const dt = (now - lastT) / 1000;
      lastT = now;

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

    // Keyboard
    const onKey = (e: KeyboardEvent) => {
      switch (e.key) {
        case '1': game.selectLimb('L_hand'); break;
        case '2': game.selectLimb('R_hand'); break;
        case '3': game.selectLimb('L_foot'); break;
        case '4': game.selectLimb('R_foot'); break;
        case 'g': case 'G': game.requestGrip(); break;
        case 'x': case 'X': game.requestRelease(); break;
        case 'r': case 'R': game.reset(); break;
      }
      forceUpdate(x => x + 1);
    };
    window.addEventListener('keydown', onKey);

    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener('keydown', onKey);
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
