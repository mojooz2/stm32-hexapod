# Hardware and Power Architecture

## Main electronics

- STM32 NUCLEO-F429ZI development board
- 2 × PCA9685 16-channel PWM controller boards
- 18 × MG996R-class servos
- 4S LiPo battery, 14.8 V nominal
- 2 × high-current DC-DC buck converters adjusted to approximately 6 V
- High-current busbar distribution for servo power and ground
- Inline fuse on the battery positive lead

## Servo/PWM topology

Both PCA9685 boards share the same STM32 I²C1 bus.

```text
NUCLEO PB8 / I2C1_SCL ──+── PCA9685 #1 SCL
                         └── PCA9685 #2 SCL

NUCLEO PB9 / I2C1_SDA ──+── PCA9685 #1 SDA
                         └── PCA9685 #2 SDA
```

Firmware addresses:

```text
PCA #1 = 0x40
PCA #2 = 0x41
```

Logical servo mapping:

```text
Servo 0-8   -> PCA #1 channels 0-8
Servo 9-17  -> PCA #2 channels 0-8
```

Leg mapping:

```text
A -> 0,  1,  2
B -> 3,  4,  5
C -> 6,  7,  8
D -> 9, 10, 11
E -> 12, 13, 14
F -> 15, 16, 17
```

## Power distribution

Servo current is not routed through the PCA9685 PCB power path. Instead, the servo power rails are distributed directly from two dedicated buck converters.

```text
4S LiPo
  |
25 A fuse
  |
power split
  |-----------------------|
  |                       |
Buck #1 (~6 V)        Buck #2 (~6 V)
  |                       |
+6 V + GND busbars    +6 V + GND busbars
  |                       |
9 servos              9 servos
```

Important implementation rules:

- The two positive ~6 V buck outputs are kept separate.
- All grounds are common: battery negative, both buck grounds, both servo ground distributions, PCA9685 grounds, and STM32 ground.
- PCA9685 `VCC` is the logic supply, not the high-current servo supply.
- The MCU is not powered from the servo rail in the development setup.

## Individual servo calibration

The 18 servos were calibrated individually. `servo.h` stores one PWM count for each of `-90°`, `0°`, and `+90°` for every logical servo.

`Servo_SetAngle()` then performs piecewise-linear interpolation:

```text
[-90°, 0°]  -> interpolate from pulse_neg90 to pulse_0
[0°, +90°]  -> interpolate from pulse_0 to pulse_pos90
```

This avoids treating mechanical zero and endpoint PWM values as identical across all actuators.

## Mechanical integration

The robot's structural parts were 3D printed and iterated throughout development. Bearing-supported pivots were incorporated into the chassis/leg interfaces, and the central structure was designed to carry the controller and high-current power hardware above the leg assembly.

![Power system during integration](../media/development/06_power_stack_prototype.jpg)
