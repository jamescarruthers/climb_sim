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
  return (
    <div style={{
      position: 'absolute', bottom: 12, left: 12,
      background: 'rgba(10, 10, 16, 0.82)', padding: 12, borderRadius: 8,
      fontSize: 12, lineHeight: 1.4, pointerEvents: 'auto', display: 'flex', flexDirection: 'column', gap: 8,
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
            }}
          >
            {limbLabels[l]}
          </button>
        ))}
      </div>

      <div style={{ display: 'flex', gap: 6, marginTop: 4 }}>
        <button
          onClick={() => { game.requestGrip(); onAction(); }}
          disabled={!snap.selectedHold}
          style={buttonStyle(snap.selectedHold != null, '#4caf50')}
        >
          Grip
        </button>
        <button
          onClick={() => { game.requestRelease(); onAction(); }}
          style={buttonStyle(true, '#f44336')}
        >
          Release
        </button>
        <button
          onClick={() => { game.reset(); onAction(); }}
          style={buttonStyle(true, '#666')}
        >
          Reset (R)
        </button>
      </div>

      <div style={{ fontSize: 10, color: '#888', maxWidth: 280, marginTop: 4 }}>
        Click a hold on the wall, pick a limb, then press Grip.
        Keys: 1–4 select limb · G grip · X release · R reset.
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
