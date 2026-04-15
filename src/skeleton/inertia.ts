/**
 * Moment of inertia helpers for cylindrical segment approximations.
 * I_axial     = 0.5 * m * r^2
 * I_transverse = m * (3 r^2 + L^2) / 12
 */

export interface PrincipalInertia {
  axial: number;     // about the segment's long axis (Y)
  transverse: number; // about X and Z
}

export function cylinderInertia(mass: number, length: number, radius: number): PrincipalInertia {
  return {
    axial: 0.5 * mass * radius * radius,
    transverse: mass * (3 * radius * radius + length * length) / 12,
  };
}

/** Inertia tensor as a diagonal 3-vector in body-local frame (Ix, Iy, Iz), Y = axial. */
export function diagInertia(mass: number, length: number, radius: number): [number, number, number] {
  const { axial, transverse } = cylinderInertia(mass, length, radius);
  return [transverse, axial, transverse];
}
