import React from 'react';
import { GameSnapshot } from '../game/Game';

const limbLabels = {
  L_hand: 'Left Hand',
  R_hand: 'Right Hand',
  L_foot: 'Left Foot',
  R_foot: 'Right Foot',
} as const;

function Bar({ label, value, color, warnAt = 0.7 }: { label: string; value: number; color?: string; warnAt?: number }) {
  const pct = Math.max(0, Math.min(1, value)) * 100;
  const c = value > warnAt ? '#ff4d4d' : color || '#4caf50';
  return (
    <div style={{ marginBottom: 4 }}>
      <div style={{ fontSize: 10, color: '#aaa', display: 'flex', justifyContent: 'space-between' }}>
        <span>{label}</span>
        <span>{(value * 100).toFixed(0)}%</span>
      </div>
      <div style={{ height: 6, background: '#222', borderRadius: 3, overflow: 'hidden' }}>
        <div style={{ height: '100%', width: `${pct}%`, background: c, transition: 'width 80ms' }} />
      </div>
    </div>
  );
}

export function HUD({ snap }: { snap: GameSnapshot }) {
  const difficultyColor =
    snap.moveClass === 'Easy' ? '#4caf50'
    : snap.moveClass === 'Moderate' ? '#8bc34a'
    : snap.moveClass === 'Hard' ? '#ffc107'
    : snap.moveClass === 'Very Hard' ? '#ff9800'
    : '#f44336';

  return (
    <div style={{
      position: 'absolute', top: 12, left: 12, width: 260,
      background: 'rgba(10, 10, 16, 0.82)', padding: 12, borderRadius: 8,
      fontSize: 12, lineHeight: 1.4, pointerEvents: 'auto',
    }}>
      <div style={{ fontSize: 14, fontWeight: 600, marginBottom: 8 }}>Climb Sim</div>

      <Bar label="Grip availability" value={snap.gripAvailability} warnAt={1} />
      <Bar label="Forearm pump (M_F)" value={snap.forearmPump} color="#d04060" warnAt={0.5} />
      <Bar label="Lactate" value={snap.lactate} color="#ff9800" warnAt={0.6} />
      <Bar label="PCr (burst pool)" value={snap.pcr} color="#42a5f5" warnAt={1} />
      <Bar label="Aerobic output" value={snap.aerobic / 0.4} color="#26a69a" warnAt={1} />

      <div style={{ marginTop: 10, paddingTop: 8, borderTop: '1px solid #333' }}>
        <div style={{ fontSize: 11, color: '#aaa' }}>Current move</div>
        <div style={{ fontSize: 16, fontWeight: 600, color: difficultyColor }}>
          {snap.moveClass}
        </div>
        <div style={{ fontSize: 10, color: '#999' }}>score {snap.moveDifficulty.toFixed(2)} — stability {snap.stabilityMargin.toFixed(2)}m</div>
      </div>

      <div style={{ marginTop: 10, paddingTop: 8, borderTop: '1px solid #333' }}>
        <div style={{ fontSize: 11, color: '#aaa' }}>Grips</div>
        {(['L_hand', 'R_hand', 'L_foot', 'R_foot'] as const).map(l => (
          <div key={l} style={{ display: 'flex', justifyContent: 'space-between',
                                 color: snap.activeLimb === l ? '#ffe066' : '#ccc' }}>
            <span>{limbLabels[l]}</span>
            <span style={{ color: snap.onHolds[l] ? '#4caf50' : '#888' }}>
              {snap.onHolds[l] ?? '—'}
            </span>
          </div>
        ))}
      </div>

      <div style={{ marginTop: 10, paddingTop: 8, borderTop: '1px solid #333', fontSize: 10, color: '#888' }}>
        Wall angle: {snap.wallAngle}° · t={snap.t.toFixed(1)}s · fails={snap.gripFailCount}
      </div>

      {snap.fell && (
        <div style={{ marginTop: 8, color: '#f44336', fontWeight: 600 }}>Fell! Press R to reset.</div>
      )}
      {snap.topReached && (
        <div style={{ marginTop: 8, color: '#4caf50', fontWeight: 600 }}>Top reached!</div>
      )}
    </div>
  );
}
