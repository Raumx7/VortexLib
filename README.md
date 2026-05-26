# VortexLib

PROS library for chassis control and odometry of our robot.

## Installation

Fetch and apply the library using the PROS CLI:

```bash
pros c fetch https://github.com/Raumx7/VortexLib/releases/download/v0.1.0/vortexlib@0.1.0.zip
pros c apply VortexLib
```

Then include the header in your project:

```cpp
#include "vortexlib/drivetrain.h"
```

---

## Drivetrain class

![Drivetrain UML diagram](images/drivetrainUML.png)

### Constructor

The `Drivetrain` constructor takes references to all hardware objects and a `DriveConfig` struct containing every tunable parameter. No sensor initialization happens here — sensors must be declared globally before being passed in.

```cpp
vortex::Drivetrain chasis(m_left, m_right, imu, rot_l, rot_r, configs, 8.25, (60.0 / 36.0));
```

| Parameter | Type | Description |
|---|---|---|
| `left_motors` | `pros::MotorGroup&` | Left drive motor group |
| `right_motors` | `pros::MotorGroup&` | Right drive motor group |
| `imu_sensor` | `pros::IMU&` | Inertial sensor |
| `left_rotation` | `pros::Rotation&` | Left tracking encoder |
| `right_rotation` | `pros::Rotation&` | Right tracking encoder |
| `config` | `const DriveConfig&` | PID constants and motion parameters |
| `wheel_diameter` | `double` | Wheel diameter in centimeters |
| `gear_ratio` | `double` | `motor_teeth / wheel_teeth` — defaults to `1.0` |

---

### DriveConfig

`DriveConfig` groups all tunable values into one struct so they stay in `main.cpp` and never require touching the library source. It contains two `PIDConstants` structs and one `MoveParams` struct.

```cpp
vortex::DriveConfig configs {
    {.kp = 2.5, .ki = 0.0, .kd = 0.02},    // move_pid
    {.kp = 2.5, .ki = 0.0, .kd = 0.02},    // turn_pid
    {
        .angle_kp = 10.0,
        .accel_st = 7.0,
        .min_move = 30.0,
        .min_turn = 20.0
    }
};
```

#### PIDConstants

Used for both `move_pid` and `turn_pid`.

| Field | Type | Description |
|---|---|---|
| `kp` | `double` | Proportional gain — primary correction force |
| `ki` | `double` | Integral gain — corrects steady-state error; keep near `0.0` until `kp` and `kd` are tuned |
| `kd` | `double` | Derivative gain — dampens overshoot; acts as a predictive brake |

#### MoveParams

| Field | Type | Description |
|---|---|---|
| `angle_kp` | `double` | Heading correction gain inside `move_centimeters()` — how aggressively the robot steers straight |
| `accel_st` | `double` | Acceleration step per 20 ms cycle — controls ramp-up rate; deceleration is applied at 2.5× this value |
| `min_move` | `double` | Minimum motor output for `move_centimeters()` — overcomes static friction on the drivetrain |
| `min_turn` | `double` | Minimum motor output for `face_angle()` — overcomes static friction during turns |

> **Tuning order:** set `kp` first at low speed until the robot reaches the target without oscillating. Then raise `kd` to eliminate overshoot at full speed. Only add `ki` if a consistent position error remains after the robot stops.

---

### Setup

Declare all hardware objects globally, then create the `Drivetrain` instance. Start the odometry task in `initialize()`.

```cpp
#include "main.h"
#include "vortexlib/drivetrain.h"

pros::MotorGroup m_left({-1, -12, 11});
pros::MotorGroup m_right({10, -15, 18});
pros::IMU        imu(16);
pros::Rotation   rot_l(13);
pros::Rotation   rot_r(19);

vortex::DriveConfig configs {
    {.kp = 2.5, .ki = 0.0, .kd = 0.02},
    {.kp = 2.5, .ki = 0.0, .kd = 0.02},
    {.angle_kp = 10.0, .accel_st = 7.0, .min_move = 30.0, .min_turn = 20.0}
};

vortex::Drivetrain chasis(m_left, m_right, imu, rot_l, rot_r, configs, 8.25, (60.0 / 36.0));

void initialize() {
    chasis.calibrate_imu();
    chasis.reset_positions();

    pros::Task odom_task([](void* param) {
        auto* c = static_cast<vortex::Drivetrain*>(param);
        while (true) { c->update_odom(); pros::delay(10); }
    }, &chasis, "odom");
}
```

---

## Methods

### Configuration

---

#### `set_brake_mode()`

Sets the brake mode for both motor groups simultaneously.

```cpp
chasis.set_brake_mode(pros::MotorBrake::coast);
```

| Parameter | Type | Description |
|---|---|---|
| `mode` | `const pros::MotorBrake&` | `coast`, `brake`, or `hold` |

---

#### `set_reversed()`

Reverses both tracking encoders in software. Call this in `initialize()` if forward motion produces negative encoder readings due to physical mounting orientation.

```cpp
chasis.set_reversed();
```

---

### Telemetry

---

#### `get_distance()`

Returns the signed average displacement of both tracking encoders since the last `reset_positions()` call, in centimeters. Forward motion returns a positive value; reverse returns negative.

```cpp
double dist = chasis.get_distance();
```

---

#### `get_pose()`

Returns a snapshot of the current odometry position.

```cpp
vortex::Pose pose = chasis.get_pose();
// pose.x       → cm East from origin
// pose.y       → cm North from origin
// pose.heading → degrees [0, 360), 0° = North
```

---

#### `set_pose()`

Forces the odometry position to a known value. Call at the start of each autonomous routine to define the field origin relative to the robot's starting tile.

```cpp
chasis.set_pose({.x = 0.0, .y = 0.0, .heading = 0.0});
```

| Parameter | Type | Description |
|---|---|---|
| `pose` | `const Pose&` | Target position `{x, y, heading}` |

---

#### `is_overheating()`

Returns `true` if any motor in either group exceeds 55 °C. Useful for displaying a warning on the controller during long matches.

```cpp
if (chasis.is_overheating()) {
    master.set_text(2, 0, "! TEMP HIGH !");
}
```

---

### Driver Control

---

#### `arcade()`

Maps joystick axes to differential drive using a cubic response curve. A ±5 deadband on both axes prevents motor chatter from stick drift.

```cpp
void opcontrol() {
    while (true) {
        int fwd  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chasis.arcade(fwd, turn);
        pros::delay(20);
    }
}
```

| Parameter | Type | Description |
|---|---|---|
| `forward` | `int` | Left stick Y axis `[-127, 127]` |
| `turn` | `int` | Right stick X axis `[-127, 127]` |

---

#### `hold()`

Applies active HOLD braking to both motor groups. Saves the current brake mode and restores it after `duration_ms` if a duration is provided.

```cpp
chasis.hold();        // hold indefinitely (autonomous)
chasis.hold(150);     // hold for 150 ms, then restore previous mode (driver control)
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `duration_ms` | `int` | `0` | Hold duration in milliseconds; `0` holds indefinitely |

---

### Autonomous — Primitives

---

#### `calibrate_imu()`

Resets the IMU and waits up to 2 seconds for calibration to complete. Must be called before any method that reads heading. Call in `initialize()`.

```cpp
void initialize() {
    chasis.calibrate_imu();
}
```

---

#### `reset_positions()`

Zeroes both tracking encoder positions. Call before a routine that needs fresh relative distance tracking, or use the `start_dist` snapshot pattern inside `move_centimeters()` to avoid resetting shared state.

```cpp
chasis.reset_positions();
```

---

#### `move_centimeters()`

Drives a signed distance in centimeters along the heading held at the moment of the call. Positive values move forward; negative values move in reverse.

Uses a PID controller with an asymmetric slew rate (deceleration at 2.5× the acceleration step) and a heading correction term to keep the robot straight. Exits when position error is within 1 cm or the timeout elapses.

```cpp
chasis.move_centimeters(60);         // drive 60 cm forward at full speed
chasis.move_centimeters(-30, 80);    // drive 30 cm in reverse at 80/127 speed
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_cm` | `int` | — | Target displacement in centimeters (signed) |
| `max_vel` | `int` | `127` | Maximum motor output `[0, 127]` |

---

#### `swing_turn()`

Pivots the chassis around one stationary wheel. The pivot side is set to HOLD and braked; the active side drives with a PD controller. Useful for aligning to field walls without lateral drift.

```cpp
chasis.swing_turn(45, vortex::Side::RIGHT, 70);   // pivot right 45°
chasis.swing_turn(-30, vortex::Side::LEFT);        // pivot left -30° at default speed
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `degrees` | `int` | — | Rotation in degrees (signed) |
| `side` | `const Side&` | — | `Side::LEFT` or `Side::RIGHT` — the wheel that stays fixed |
| `max_vel` | `int` | `80` | Maximum motor output for the active side |

---

### Odometry

---

#### `update_odom()`

Integrates encoder deltas and IMU heading into the `(x, y, heading)` pose using an arc model with averaged heading to reduce integration error on curved paths. Must run in a background task every ~10 ms — it is not called automatically.

```cpp
pros::Task odom_task([](void* param) {
    auto* c = static_cast<vortex::Drivetrain*>(param);
    while (true) {
        c->update_odom();
        pros::delay(10);
    }
}, &chasis, "odom");
```

Field convention: **North = +Y, East = +X**, heading 0° = North — consistent with `imu.get_heading()`.

---

### Autonomous — High Level

---

#### `face_angle()`

Rotates to an absolute field heading using a PID controller. Always takes the shortest path (normalized to `[-180°, 180°]`). Includes integral windup mitigation and friction compensation via `min_turn`.

```cpp
chasis.face_angle(0);          // face North
chasis.face_angle(90, 100);    // face East at speed 100
chasis.face_angle(180);        // face South
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_heading` | `double` | — | Absolute field heading in degrees `[0, 360)` |
| `max_vel` | `int` | `80` | Maximum motor output |

---

#### `drive_to_point()`

Moves the robot to a field coordinate `(x, y)` in two phases: rotate to face the target with `face_angle()`, then drive the Euclidean distance with `move_centimeters()`. Automatically drives in reverse if the required turn exceeds 90°.

Requires `update_odom()` running in the background. Call `set_pose()` before the autonomous routine to establish the correct origin.

```cpp
chasis.set_pose({.x = 0.0, .y = 0.0, .heading = 0.0});

chasis.drive_to_point(60.0, 90.0);         // go to (60, 90) at full speed
chasis.drive_to_point(-30.0, 45.0, 90);    // go to (-30, 45) at speed 90
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `x` | `double` | — | Target X coordinate in centimeters (East) |
| `y` | `double` | — | Target Y coordinate in centimeters (North) |
| `max_vel` | `int` | `127` | Maximum motor output |

---

## Field Coordinate System

```
         0° (North) +Y
              ↑
              │
 270° ────────┼──────── 90° (East) +X
 (West)       │
              ↓
           180° (South)
```

`set_pose()` and `drive_to_point()` use this convention. Match `imu.get_heading()` — the sensor returns `0°` when facing the direction calibrated as North at startup.
