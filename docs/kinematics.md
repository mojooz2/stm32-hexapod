# Leg Kinematics

## Coordinate convention

Each of the six legs is solved in its own local Cartesian frame. The local `+x` direction points radially outward from the robot body, `y` is tangential in the leg plane convention used by the firmware, and `z` is vertical in the leg IK convention.

The leg geometry used in `motion.c` is:

```text
L0 = 50 mm   // coxa
L1 = 75 mm   // femur
L2 = 110 mm  // tibia
```

The default endpoint state stored for every leg is:

```text
(DEFAULT_X, DEFAULT_Y, DEFAULT_Z)
= (151.851578, 0.0, -66.1263275) mm
```

## Forward model

For mathematical joint angles `theta0`, `theta1`, and `theta2`, define

```text
rho = L0 + L1 cos(theta1) + L2 cos(theta1 + theta2)
```

then

```text
x = cos(theta0) * rho
y = sin(theta0) * rho
z = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

## Analytical inverse kinematics

The implemented inverse solution first computes the horizontal radius

```text
r = sqrt(x^2 + y^2)
```

and

```text
D = ((r-L0)^2 + z^2 - L1^2 - L2^2) / (2 L1 L2)
```

If `|D| > 1`, the requested Cartesian position is unreachable.

The selected branch is

```text
theta0 = atan2(y, x)

theta2 = atan2(-sqrt(1-D^2), D)

theta1 = atan2(z, r-L0)
       - atan2(L2 sin(theta2), L1 + L2 cos(theta2))
```

The mathematical angles are converted to the robot's servo convention as

```text
servo0 = theta0 [deg]
servo1 = theta1 [deg] - 5
servo2 = -theta2 [deg] - 60
```

A Cartesian point is accepted only if all three resulting servo commands lie in `[-90°, +90°]`.

## Nominal pose

The nominal physical servo commands used in `main.c` are

```text
joint 0 = 0.0 deg
joint 1 = 25.0 deg
joint 2 = 40.4 deg
```

which correspond to the default Cartesian position stored in `motion.c`.

## Trajectory validation

`StraightLine()` and `SemiCircle()` both generate 51 Cartesian samples. Inverse kinematics is evaluated at every sample. If any one sample fails, the entire generated primitive is rejected.

For whole-body operations such as `Walk()`, `Vertical()`, and `Spread()`, the six required leg trajectories are generated before `ExecutePhase()` is called. This prevents a motion from beginning when one of the requested leg paths is invalid.
