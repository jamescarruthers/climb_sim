# climb_sim

Browser-based climbing game with physics-based ragdoll, muscle fatigue,
forearm pump, and energy metabolism. TypeScript + React + Three.js,
custom XPBD physics layer, no WASM dependencies.

## Running

```
npm install
npm run dev
```

Then open http://localhost:5173.

## Project structure

```
src/
├── math/        Vec2, Vec3, Quat, convex hull, misc helpers
├── skeleton/    20-segment de Leva-scaled articulation, joint limits, inertias
├── physics/     RigidBody, PointConstraint, AngularConstraint (swing-twist), World
├── muscle/      Joint torque generators (Hill curves, activation dynamics)
├── fatigue/     3CCr per-region fatigue + forearm pump + arm-position recovery
├── energy/      Three-tier metabolic model (PCr / glycolytic / aerobic, lactate)
├── wall/        Wall geometry, hold types and friction signatures
├── balance/     CoM, support polygon, stability margin, difficulty scoring
├── control/     Stable PD (Tan, Liu & Turk 2011)
├── climber/     Climber assembly — ties everything together; grip constraints
├── game/        Top-level Game controller, fixed-substep loop, snapshots
├── render/      Three.js scene, capsule rig, hold picking
├── ui/          React HUD (fatigue/energy bars), control panel
├── App.tsx      Top-level React component, animation loop, keybindings
└── main.tsx     Entry point
```

## Controls

- Click a hold on the wall to select it.
- Buttons `LH / RH / LF / RF` or keys `1 2 3 4` select the active limb.
- Press `Grip` (or `G`) to attach the active limb to the selected hold.
- Press `Release` (or `X`) to release the active limb.
- Press `R` to reset.

## Implementation notes

Physics uses XPBD-style position projection for joint ball constraints, plus
torque-based swing-twist limiters for joint ranges of motion (with elliptical
cone + twist). Gripping creates a point constraint to a world-fixed anchor
with a `maxForce` that scales with fatigue-driven grip availability; when the
implied constraint force exceeds the max, the grip breaks.

Joint torque generators (one per joint DOF, aggregated by muscle region) model
activation dynamics, Hill torque-velocity curves, and passive joint limits.
Per-region three-compartment fatigue (Xia & Frey 2008, Looft et al. 2018)
tracks resting / active / fatigued motor units with the 3CCr intermittent-task
recovery boost.

The forearm pump specialisation adds blood-flow occlusion above 30% grip MVC
and an arm-position recovery multiplier (arms below heart recover faster).

Centre of mass, convex-hull support polygon and stability margin are computed
every step and feed a per-move difficulty score (torque cost + stability cost
+ friction cost + reach cost).

Phase 1–2 of the spec are implemented; phases 3–4 (learned balance, route
editor, energy sim polish, sound) are stubs / future work.
