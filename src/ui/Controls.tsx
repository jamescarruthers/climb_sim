import React from 'react';
import { Game, GameSnapshot } from '../game/Game';
import { LimbGrip } from '../climber/Climber';

const limbs: LimbGrip['limb'][] = ['L_hand', 'R_hand', 'L_foot', 'R_foot'];
const limbLabels: Record<LimbGrip['limb'], string> = {
  L_hand: 'LH', R_hand: 'RH', L_foot: 'LF', R_foot: 'RF',
};

export function Controls({ game, snap, onAction }: {
  game: Game;
  snap: GameSnapshot;
  onAction: () => void;
}) {
  const reachActive = !!snap.reachTargets[snap.activeLimb];

  return (
    <div style={{
      position: 'absolute', bottom: 12, left: 12,
      background: 'rgba(10, 10, 16, 0.82)', padding: 12, borderRadius: 8,
      fontSize: 12, lineHeight: 1.4, pointerEvents: 'auto', display: 'flex', flexDirection: 'column', gap: 8,
      minWidth: 320,
    }}>
      <div style={{ fontSize: 11, color: '#aaa' }}>Active limb</div>
      <div style={{ display: 'flex', gap: 4 }}>
        {limbs.map(l => (
          <button
            key={l}
            onClick={() => { game.selectLimb(l); onAction(); }}
            style={{
              padding: '6px 10px', borderRadius: 6, border: '1px solid #444',
              background: snap.activeLimb === l ? '#ffe066' : '#222',
              color: snap.activeLimb === l ? '#000' : '#ddd', cursor: 'pointer',
              fontWeight: 600, minWidth: 38,
              outline: snap.reachTargets[l] ? '2px solid #42a5f5' : 'none',
            }}
            title={snap.reachTargets[l] ? `reaching → ${snap.reachTargets[l]}` : ''}
          >
            {limbLabels[l]}
          </button>
        ))}
      </div>

      <div style={{ display: 'flex', gap: 6, marginTop: 4, flexWrap: 'wrap' }}>
        <button
          onClick={() => { game.requestReach(); onAction(); }}
          disabled={!snap.selectedHold}
          style={buttonStyle(snap.selectedHold != null, '#42a5f5')}
          title="Move limb toward selected hold (auto-grip when close)"
        >
          Reach (F)
        </button>
        <button
          onClick={() => { game.requestGrip(); onAction(); }}
          disabled={!snap.selectedHold}
          style={buttonStyle(snap.selectedHold != null, '#4caf50')}
          title="Snap-attach to selected hold immediately"
        >
          Grip (G)
        </button>
        <button
          onClick={() => { game.requestRelease(); onAction(); }}
          style={buttonStyle(true, '#f44336')}
        >
          Release (X)
        </button>
        <button
          onClick={() => { game.reset(); onAction(); }}
          style={buttonStyle(true, '#666')}
        >
          Reset (R)
        </button>
      </div>

      <LeanPad lean={snap.bodyLean} legDrive={snap.legDrive} />

      <div style={{ fontSize: 10, color: '#888', maxWidth: 320, marginTop: 4, lineHeight: 1.5 }}>
        <b>Climb:</b> click a hold → pick a limb (1–4) → press <b>Reach (F)</b>.
        The limb extends toward the hold and auto-grips when close.<br/>
        <b>Balance:</b> hold <b>WASD</b> to lean the body — shift weight before
        a hard reach so the climber doesn't barn-door off.<br/>
        <b>Push:</b> hold <b>SPACE</b> to drive through the legs — quads + glutes
        fire to extend the legs hard, launching the body upward.
      </div>
    </div>
  );
}

/** Mini compass + leg-drive bar showing the player's body lean and push. */
function LeanPad({ lean, legDrive }: { lean: [number, number]; legDrive: number }) {
  const size = 70;
  const radius = (size - 16) / 2;
  const cx = lean[0] * radius + size / 2;
  const cy = -lean[1] * radius + size / 2;
  const drivePct = Math.max(0, Math.min(1, legDrive)) * 100;
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 10, marginTop: 6 }}>
      <svg width={size} height={size} style={{ background: '#1a1a22', borderRadius: 8 }}>
        <circle cx={size / 2} cy={size / 2} r={radius} fill="none" stroke="#333" strokeWidth={1} />
        <line x1={size / 2} y1={4} x2={size / 2} y2={size - 4} stroke="#333" strokeWidth={1} />
        <line x1={4} y1={size / 2} x2={size - 4} y2={size / 2} stroke="#333" strokeWidth={1} />
        <circle cx={cx} cy={cy} r={6} fill="#ffe066" />
      </svg>
      <div style={{ fontSize: 10, color: '#aaa', flex: 1 }}>
        Body lean (WASD)<br/>
        <span style={{ color: '#ddd' }}>x = {lean[0].toFixed(2)} · y = {lean[1].toFixed(2)}</span>
        <div style={{ marginTop: 6 }}>
          Leg drive (SPACE)
          <div style={{ height: 6, background: '#222', borderRadius: 3, overflow: 'hidden', marginTop: 2 }}>
            <div style={{ height: '100%', width: `${drivePct}%`, background: '#ff7043', transition: 'width 60ms' }} />
          </div>
        </div>
      </div>
    </div>
  );
}

function buttonStyle(enabled: boolean, color: string): React.CSSProperties {
  return {
    padding: '8px 14px', borderRadius: 6, border: '1px solid #444',
    background: enabled ? color : '#222', color: enabled ? '#fff' : '#666',
    fontWeight: 600, cursor: enabled ? 'pointer' : 'not-allowed',
  };
}
