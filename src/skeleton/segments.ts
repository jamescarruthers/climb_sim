/**
 * Segment anthropometry based on De Leva (1996) adjustments to
 * Zatsiorsky-Seluyanov segment parameters, scaled for a 70 kg / 1.75 m climber.
 *
 * Mass fractions sum to 1.0. Lengths are approximate; moments of inertia are
 * derived from a cylinder approximation (see inertia.ts).
 */

export interface SegmentDef {
  id: string;
  massFraction: number;
  length: number;   // metres — long axis of the segment
  radius: number;   // metres — cross-section radius for cylinder inertia
  parent: string | null;
}

// Height and total body mass (tunable). The player default is 70 kg, 1.75 m.
export const DEFAULT_HEIGHT = 1.75;
export const DEFAULT_MASS = 70;

export const SEGMENTS: SegmentDef[] = [
  { id: 'pelvis',         massFraction: 0.142, length: 0.15, radius: 0.13, parent: null },

  { id: 'lumbar',         massFraction: 0.150, length: 0.18, radius: 0.11, parent: 'pelvis' },
  { id: 'thorax',         massFraction: 0.285, length: 0.32, radius: 0.13, parent: 'lumbar' },
  { id: 'head',           massFraction: 0.069, length: 0.26, radius: 0.08, parent: 'thorax' },

  { id: 'L_upper_arm',    massFraction: 0.027, length: 0.28, radius: 0.045, parent: 'thorax' },
  { id: 'L_forearm',      massFraction: 0.016, length: 0.27, radius: 0.035, parent: 'L_upper_arm' },
  { id: 'L_hand',         massFraction: 0.006, length: 0.19, radius: 0.03,  parent: 'L_forearm' },

  { id: 'R_upper_arm',    massFraction: 0.027, length: 0.28, radius: 0.045, parent: 'thorax' },
  { id: 'R_forearm',      massFraction: 0.016, length: 0.27, radius: 0.035, parent: 'R_upper_arm' },
  { id: 'R_hand',         massFraction: 0.006, length: 0.19, radius: 0.03,  parent: 'R_forearm' },

  { id: 'L_thigh',        massFraction: 0.142, length: 0.42, radius: 0.08, parent: 'pelvis' },
  { id: 'L_shin',         massFraction: 0.043, length: 0.40, radius: 0.05, parent: 'L_thigh' },
  { id: 'L_foot',         massFraction: 0.014, length: 0.26, radius: 0.04, parent: 'L_shin' },

  { id: 'R_thigh',        massFraction: 0.142, length: 0.42, radius: 0.08, parent: 'pelvis' },
  { id: 'R_shin',         massFraction: 0.043, length: 0.40, radius: 0.05, parent: 'R_thigh' },
  { id: 'R_foot',         massFraction: 0.014, length: 0.26, radius: 0.04, parent: 'R_shin' },
];

// Note: mass fractions sum to > 1.0 above because we lumped abdomen+thorax and
// split lumbar off. Normalisation at runtime keeps total = CLIMBER_MASS.

export function segmentById(id: string): SegmentDef {
  const s = SEGMENTS.find(s => s.id === id);
  if (!s) throw new Error(`Unknown segment: ${id}`);
  return s;
}
