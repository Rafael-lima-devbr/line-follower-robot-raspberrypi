# Autonomous Line-Following Robot with Odometry

Personal Python robotics project focused on **PID line following, differential-drive odometry, sensor-based navigation, and movement-control experimentation**.

**Status:** Experimental / under development

<!-- MEDIA: Adicione aqui uma foto ou GIF curto do robô seguindo a linha. Idealmente, use uma imagem que mostre o robô inteiro e a pista. -->

This repository explores an odometry-oriented navigation architecture for an autonomous line-following robot. It was developed during the same period as my participation in Team LOGOS and OBR 2026, but it is **not the competition software used by Team LOGOS**.

## Highlights

- Custom PID line-following control
- Differential-drive odometry with estimated `x`, `y`, and `theta`
- Odometry-based 90° and 180° rotation helpers
- Intersection and sharp-turn handling
- Green and red color-marking logic
- Line-gap and lost-line recovery
- Obstacle handling
- Independent hardware scripts for calibration and testing

## Control Architecture

```text
Sensors
  |
  v
Track / environment detection
  |
  +-------------------+
  |                   |
  v                   v
PID line following   Special maneuvers
  |                   |
  +---------+---------+
            |
            v
       Motor commands
            |
            v
         Odometry
            |
            v
   Estimated pose (x, y, theta)
```

<!-- MEDIA: Se você fizer um diagrama visual da arquitetura, substitua ou complemente o diagrama em texto aqui. -->

The current program connects to two traction motors, one line sensor, one distance sensor and two color sensors through Open-RDK.

## PID Line Following

Normal navigation uses line-position feedback to calculate an error and apply differential correction to the motors.

Conceptually:

```text
line position -> error -> PID -> left/right motor correction
```

The controller implements proportional, integral and derivative terms, time-step calculation and output limiting.

PID and odometry have different responsibilities: **PID keeps the robot aligned with the line; odometry estimates how the robot has moved.**

## Differential-Drive Odometry

The `Odometry` class reads cumulative motor position, converts wheel rotation into traveled distance and estimates translation and rotation using differential-drive kinematics.

The estimated pose contains:

- `x` — horizontal position
- `y` — vertical position
- `theta` — orientation

For turns, the program stores the initial orientation, commands opposite wheel speeds and updates odometry until the estimated angular displacement reaches the target.

This allows some rotations to be expressed in terms of estimated movement rather than relying exclusively on a predetermined duration.

<!-- MEDIA: Um vídeo/GIF curto de uma rotação de 90° ou 180° baseada em odometria fica bem aqui. -->

## Navigation Logic

The main loop combines sensor feedback and dedicated routines for:

- obstacle detection;
- color markings;
- intersections;
- possible 90° turns;
- line gaps;
- red stop conditions;
- normal PID line following.

The architecture is intentionally mixed. Some maneuvers use odometry while others still use calibrated timing. The project should therefore be understood as **odometry-oriented**, not as a fully closed-loop odometry navigation system.

## Project Structure

```text
line-follower-robot-raspberrypi/
├── scripts/
│   ├── calibrate_line_sensor.py
│   ├── get_devices.py
│   ├── remote_control.py
│   ├── test_line_sensor.py
│   └── walk_straight.py
├── src/
│   ├── CommandDriver.py
│   ├── line_functions.py
│   ├── main.py
│   ├── odometry.py
│   └── pid.py
└── README.md
```

## Core Modules

| Module | Responsibility |
|---|---|
| `src/main.py` | Initializes hardware interfaces, reads sensors and selects navigation behavior |
| `src/line_functions.py` | Navigation logic, line following, intersections, turns, gaps, recovery and obstacle routines |
| `src/odometry.py` | Differential-drive pose estimation and angle calculations |
| `src/pid.py` | Custom PID controller for line following |
| `src/CommandDriver.py` | Asynchronous motor-command handling |

## Hardware Utility Scripts

The scripts in `scripts/` are manual hardware tools rather than an automated test suite.

| Script | Purpose |
|---|---|
| `calibrate_line_sensor.py` | Calibrates and saves line-sensor configuration |
| `get_devices.py` | Lists devices detected by Open-RDK |
| `test_line_sensor.py` | Displays line-sensor readings independently |
| `remote_control.py` | Provides keyboard-based manual motor control |
| `walk_straight.py` | Tests straight motor movement independently |

## Testing

The repository does **not yet contain an automated `tests/` suite**. Validation currently focuses on direct hardware experiments and the utility scripts above.

Future automated tests would be useful for hardware-independent logic such as:

- PID calculations;
- odometry equations;
- angle normalization;
- sensor-pattern classification;
- navigation decision logic.

## Engineering Limitations

Odometry is an estimate rather than an absolute position measurement. Accuracy can be affected by:

- wheel slip;
- wheel-radius and wheel-base calibration;
- differences between wheels;
- mechanical tolerances;
- surface conditions;
- collisions and external disturbances.

Small errors can accumulate over time. Some straight movements, scans and positioning routines also remain time-based.

## Relationship with Team LOGOS

This is my personal experimental implementation. Team LOGOS' OBR 2026 repository represents the collective competition project and used engineering decisions that had received substantially more physical validation on the competition robot.

Official Team LOGOS repository:

https://github.com/LOGOSIFBA/LOGOS

The **Extra Innovation Award at OBR 2026 belongs to Team LOGOS and the team's work**, not to this personal repository.

## Development Direction

Possible future improvements include:

- automated tests for hardware-independent logic;
- quantitative odometry-error measurements;
- wheel-radius and wheel-base calibration procedures;
- replacement of selected time-based translations with distance-based control where useful;
- improved fusion between movement estimation and external sensing.
