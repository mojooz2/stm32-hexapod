# Alternating-Tripod Gait

## Tripod groups

The robot uses two alternating support groups:

```text
Tripod 1: A, C, E
Tripod 2: B, D, F
```

The physical mounting angles used by the planner are:

| Leg | `beta` |
|---|---:|
| A | 30° |
| B | 90° |
| C | 150° |
| D | 210° |
| E | 270° |
| F | 330° |

The user command `dir` is expressed in radians. Positive direction is clockwise when the robot is viewed from above.

## Critical foot positions

At the beginning of `Walk()`, each leg's persistent `current_position` is copied into `C0`.

For each leg:

```text
gamma = beta - dir

half_dx = (step_size / 2) cos(gamma)
half_dy = (step_size / 2) sin(gamma)

Cb = (C0.x - half_dx,
      C0.y - half_dy,
      C0.z)

Cf = (C0.x + half_dx,
      C0.y + half_dy,
      C0.z)
```

`Cb` is the backward critical point and `Cf` is the forward critical point.

## Motion primitives

Two paths are combined to create the gait:

- **Swing:** upward `SemiCircle()` path.
- **Stance:** `StraightLine()` path.

The semicircle provides foot clearance while the opposite tripod remains in contact with the ground.

## Sequence

### Preparation 1

```text
A C E: C0 -> Cb  (semicircle)
B D F: remain at C0
```

### Preparation 2

```text
A C E: remain at Cb
B D F: C0 -> Cf  (semicircle)
```

### Full step type 1

```text
A C E: Cb -> Cf  (semicircle / swing)
B D F: Cf -> Cb  (straight / stance)
```

### Full step type 2

```text
A C E: Cf -> Cb  (straight / stance)
B D F: Cb -> Cf  (semicircle / swing)
```

The two full-step types alternate until `step_num` has been completed.

## Recovery

The recovery sequence depends on whether the requested number of steps is odd or even. The tripod currently at `Cf` is first returned to `C0` by a semicircular half-step, then the tripod at `Cb` is returned to `C0`.

This leaves every foot at the same local Cartesian position from which `Walk()` began.

## Stance adjustment compatibility

`Vertical()` modifies only the stored local `z` coordinate of every foot. `Spread()` modifies only the stored local `x` coordinate of every foot.

Because `Walk()` uses the current stored coordinates as `C0`, the walking gait can be executed after either stance adjustment without resetting to the original hard-coded geometry.
