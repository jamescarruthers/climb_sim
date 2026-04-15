import { rad } from '../math/util';

/**
 * Joint definitions. Each joint connects a child segment's proximal end to an
 * anchor on the parent. `localAnchorParent`/`localAnchorChild` are given in the
 * parent/child local frame as fractional positions along the segment long axis
 * (Y+), with X being lateral (to climber's right).
 *
 * For simplicity all joints are modelled as ball joints in the physics layer;
 * the per-DOF limits are enforced via swing-twist decomposition against the
 * parent orientation.
 */

export type JointKind = 'ball' | 'hinge' | 'universal';

export interface JointDef {
  id: string;
  parent: string;
  child: string;

  // Position of the joint anchor in each segment's local frame.
  // Long axis is Y+; origin is the segment's centre of mass.
  localAnchorParent: [number, number, number];
  localAnchorChild:  [number, number, number];

  kind: JointKind;

  // Swing cone half-angles (radians) about parent X and Z axes.
  swingX: number;
  swingZ: number;
  // Twist limits (radians) about parent Y axis.
  twistMin: number;
  twistMax: number;

  // Muscle region used for torque generation and fatigue book-keeping.
  muscleRegion: MuscleRegion;
}

export type MuscleRegion =
  | 'hip' | 'knee' | 'ankle'
  | 'shoulder' | 'elbow' | 'wrist' | 'grip'
  | 'trunk' | 'neck';

// Helper: anchor at the proximal end of a segment of length L (origin at CoM).
const prox = (L: number): [number, number, number] => [0, -L / 2, 0];
const dist = (L: number): [number, number, number] => [0,  L / 2, 0];

import { SEGMENTS } from './segments';
function L(id: string): number { return SEGMENTS.find(s => s.id === id)!.length; }

export const JOINTS: JointDef[] = [
  // Spine
  { id: 'lumbar_j',   parent: 'pelvis', child: 'lumbar',
    localAnchorParent: dist(L('pelvis')), localAnchorChild: prox(L('lumbar')),
    kind: 'ball', swingX: rad(25), swingZ: rad(20),
    twistMin: rad(-25), twistMax: rad(25), muscleRegion: 'trunk' },

  { id: 'thoracic_j', parent: 'lumbar', child: 'thorax',
    localAnchorParent: dist(L('lumbar')), localAnchorChild: prox(L('thorax')),
    kind: 'ball', swingX: rad(20), swingZ: rad(15),
    twistMin: rad(-20), twistMax: rad(20), muscleRegion: 'trunk' },

  { id: 'cervical_j', parent: 'thorax', child: 'head',
    localAnchorParent: dist(L('thorax')), localAnchorChild: prox(L('head')),
    kind: 'ball', swingX: rad(40), swingZ: rad(30),
    twistMin: rad(-60), twistMax: rad(60), muscleRegion: 'neck' },

  // Left arm
  { id: 'L_shoulder_j', parent: 'thorax', child: 'L_upper_arm',
    // Shoulder anchor: top-lateral of thorax
    localAnchorParent: [-0.16, L('thorax') / 2 - 0.02, 0],
    localAnchorChild: prox(L('L_upper_arm')),
    kind: 'ball', swingX: rad(120), swingZ: rad(120),
    twistMin: rad(-40), twistMax: rad(40), muscleRegion: 'shoulder' },

  { id: 'L_elbow_j', parent: 'L_upper_arm', child: 'L_forearm',
    localAnchorParent: dist(L('L_upper_arm')), localAnchorChild: prox(L('L_forearm')),
    // Hinge modeled as a very narrow swing cone in one axis.
    kind: 'hinge', swingX: rad(2), swingZ: rad(80),
    twistMin: rad(-5), twistMax: rad(5), muscleRegion: 'elbow' },

  { id: 'L_wrist_j', parent: 'L_forearm', child: 'L_hand',
    localAnchorParent: dist(L('L_forearm')), localAnchorChild: prox(L('L_hand')),
    kind: 'universal', swingX: rad(60), swingZ: rad(60),
    twistMin: rad(-10), twistMax: rad(10), muscleRegion: 'grip' },

  // Right arm (mirrored)
  { id: 'R_shoulder_j', parent: 'thorax', child: 'R_upper_arm',
    localAnchorParent: [0.16, L('thorax') / 2 - 0.02, 0],
    localAnchorChild: prox(L('R_upper_arm')),
    kind: 'ball', swingX: rad(120), swingZ: rad(120),
    twistMin: rad(-40), twistMax: rad(40), muscleRegion: 'shoulder' },

  { id: 'R_elbow_j', parent: 'R_upper_arm', child: 'R_forearm',
    localAnchorParent: dist(L('R_upper_arm')), localAnchorChild: prox(L('R_forearm')),
    kind: 'hinge', swingX: rad(2), swingZ: rad(80),
    twistMin: rad(-5), twistMax: rad(5), muscleRegion: 'elbow' },

  { id: 'R_wrist_j', parent: 'R_forearm', child: 'R_hand',
    localAnchorParent: dist(L('R_forearm')), localAnchorChild: prox(L('R_hand')),
    kind: 'universal', swingX: rad(60), swingZ: rad(60),
    twistMin: rad(-10), twistMax: rad(10), muscleRegion: 'grip' },

  // Left leg
  { id: 'L_hip_j', parent: 'pelvis', child: 'L_thigh',
    localAnchorParent: [-0.09, -L('pelvis') / 2, 0],
    localAnchorChild: prox(L('L_thigh')),
    kind: 'ball', swingX: rad(90), swingZ: rad(90),
    twistMin: rad(-40), twistMax: rad(40), muscleRegion: 'hip' },

  { id: 'L_knee_j', parent: 'L_thigh', child: 'L_shin',
    localAnchorParent: dist(L('L_thigh')), localAnchorChild: prox(L('L_shin')),
    kind: 'hinge', swingX: rad(80), swingZ: rad(2),
    twistMin: rad(-5), twistMax: rad(5), muscleRegion: 'knee' },

  { id: 'L_ankle_j', parent: 'L_shin', child: 'L_foot',
    localAnchorParent: dist(L('L_shin')), localAnchorChild: [0, -L('L_foot') / 2 + 0.04, -0.05],
    kind: 'hinge', swingX: rad(30), swingZ: rad(10),
    twistMin: rad(-5), twistMax: rad(5), muscleRegion: 'ankle' },

  // Right leg (mirrored)
  { id: 'R_hip_j', parent: 'pelvis', child: 'R_thigh',
    localAnchorParent: [0.09, -L('pelvis') / 2, 0],
    localAnchorChild: prox(L('R_thigh')),
    kind: 'ball', swingX: rad(90), swingZ: rad(90),
    twistMin: rad(-40), twistMax: rad(40), muscleRegion: 'hip' },

  { id: 'R_knee_j', parent: 'R_thigh', child: 'R_shin',
    localAnchorParent: dist(L('R_thigh')), localAnchorChild: prox(L('R_shin')),
    kind: 'hinge', swingX: rad(80), swingZ: rad(2),
    twistMin: rad(-5), twistMax: rad(5), muscleRegion: 'knee' },

  { id: 'R_ankle_j', parent: 'R_shin', child: 'R_foot',
    localAnchorParent: dist(L('R_shin')), localAnchorChild: [0, -L('R_foot') / 2 + 0.04, -0.05],
    kind: 'hinge', swingX: rad(30), swingZ: rad(10),
    twistMin: rad(-5), twistMax: rad(5), muscleRegion: 'ankle' },
];

export function jointById(id: string): JointDef {
  const j = JOINTS.find(j => j.id === id);
  if (!j) throw new Error(`Unknown joint: ${id}`);
  return j;
}
