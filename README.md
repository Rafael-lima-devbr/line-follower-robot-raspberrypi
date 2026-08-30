# Autonomous Line-Following Robot with Odometry

A personal Python robotics project focused on **PID line following, differential-drive odometry, sensor-based navigation, and movement-control experimentation**.

This repository was developed entirely by **Rafael Lima Ribeiro dos Santos** and explores an odometry-oriented navigation architecture for an autonomous line-following robot.

It was developed during the same period as Rafael's participation in **Team LOGOS** and OBR 2026, but it is **not the competition software used by Team LOGOS**.

---

## Overview

The project combines two different control responsibilities:

### Line Following

During normal navigation:

    Line Sensor
         |
         v
    Line Position
         |
         v
        Error
         |
         v
         PID
         |
         v
    Motor Correction

### Movement Estimation

At the same time:

    Motor / Wheel Position
             |
             v
      Wheel Displacement
             |
             v
    Differential-Drive Odometry
             |
             v
      Estimated Robot Pose
        (x, y, theta)

These systems are related but solve different problems.

The PID controller keeps the robot aligned with the line.

Odometry estimates how the robot has moved.

---

## Motivation

A line-following robot can use relatively simple control for normal navigation because the line itself provides continuous feedback.

Special maneuvers are different.

Operations such as:

- Rotating approximately 90 degrees
- Rotating approximately 180 degrees
- Moving through intersections
- Repositioning after detecting special markings
- Recovering from certain navigation situations

often require some estimate of how the robot has moved.

One possible strategy is to use calibrated movement times.

Another is to use information about wheel movement to estimate displacement and rotation.

This repository explores the second approach more extensively.

The objective is not to claim that odometry is universally superior.

Instead, the project investigates how movement estimates can be incorporated into the robot's navigation logic and what engineering trade-offs this introduces.

---

## Project Goals

The current project explores:

- Autonomous line following
- Custom PID control
- Line sensor interpretation
- Differential motor control
- Differential-drive odometry
- Rotation based on estimated orientation change
- Intersection handling
- Sharp-turn detection
- Color-marking interpretation
- Line-gap handling
- Lost-line recovery
- Obstacle handling
- Independent hardware utilities for calibration and testing

The project remains under development and has not received the same level of physical validation as the Team LOGOS competition strategy.

---

## Architecture

The current software is organized around a main runtime loop and a set of navigation functions.

    Open-RDK
        |
        +--------------------+
        |                    |
        v                    v
      Sensors              Motors
        |                    ^
        v                    |
    Track Detection          |
        |                    |
        +--------+-----------+
                 |
                 v
         Navigation Logic
          /             \
         v               v
    PID Following    Special Maneuvers
         |               |
         +-------+-------+
                 |
                 v
            Motor Commands
                 |
                 v
             Odometry
                 |
                 v
          Pose Estimation

The current main program connects to:

- Two traction motors
- One line sensor
- One distance sensor
- Two color sensors

through Open-RDK.

---

## PID Line Following

Normal line following is handled by a custom PID controller.

The sensor reports an estimated line position and the controller compares it with the desired set point.

Conceptually:

    error = set_point - line_position

    correction = PID(error)

    left_speed  = base_speed + correction
    right_speed = base_speed - correction

The current controller implements:

- Proportional response
- Integral accumulation
- Derivative response
- Time-step calculation using a monotonic clock
- Derivative protection for excessively large or invalid time intervals
- Output saturation

The PID output is then converted into a differential correction between the left and right motor commands.

### PID Is Not Odometry

PID line following uses **line-position feedback**.

Odometry uses **wheel/motor movement**.

Therefore:

    Line Sensor
       -> PID
       -> trajectory correction

and:

    Wheel Movement
       -> Odometry
       -> displacement / rotation estimate

are separate control responsibilities.

---

## Differential-Drive Odometry

Odometry is the main technical focus of this repository.

The robot has independently driven left and right sides.

The current `Odometry` class reads cumulative motor position in degrees and calculates how much each wheel has moved since the previous update.

The process is approximately:

    Motor Position Difference
              |
              v
        Wheel Rotation
              |
              v
       Wheel Displacement
         /           \
        /             \
     Left             Right
        \             /
         \           /
          v         v
      Differential-Drive
           Kinematics
              |
        +-----+------+
        |            |
        v            v
    Translation    Rotation
        |            |
        +-----+------+
              |
              v
        x, y, theta

The current implementation maintains an estimated pose:

- `x` — horizontal position
- `y` — vertical position
- `theta` — robot orientation

It also normalizes orientation to remain within a bounded angular range.

---

## How the Odometry Estimate Works

For each wheel, motor rotation is converted to wheel travel using the configured wheel radius.

Conceptually:

    wheel_turns = delta_degrees / 360

    wheel_distance =
        wheel_turns * wheel_circumference

The average movement of the two sides estimates forward displacement.

The difference between the right and left wheel displacement estimates robot rotation.

Conceptually:

    linear_displacement =
        (right_distance + left_distance) / 2

    angular_displacement =
        (right_distance - left_distance) / wheel_base

The current implementation then uses the estimated orientation during the movement interval to update `x` and `y`.

This is a standard differential-drive odometry approach.

---

## Odometry-Based Rotation

One of the clearest differences in this repository is the implementation of rotation helpers.

For a turn, the program:

1. Updates odometry
2. Stores the initial orientation
3. Commands opposite speeds to the two motors
4. Continuously updates odometry
5. Calculates the angular difference
6. Stops when the estimated target rotation is reached

Conceptually:

    start_angle = current_angle

    while estimated_rotation < target_rotation:
        rotate()
        update_odometry()

    stop()

This allows functions such as 90-degree and 180-degree rotations to use estimated wheel movement instead of relying exclusively on a predetermined rotation duration.

---

## Why Odometry?

A time-calibrated movement can be represented as:

    command motors
         |
         v
    wait N seconds
         |
         v
       stop

This is an open-loop strategy with respect to the resulting displacement.

If a specific movement has been extensively tested on a fixed robot and track environment, this can be a practical and reliable solution.

An odometry-oriented strategy instead attempts to use movement information:

    command motors
         |
         v
    measure wheel movement
         |
         v
    estimate robot movement
         |
         v
    compare with target
         |
         v
       stop

This provides a way to reason about physical movement rather than elapsed time alone.

Potential advantages include:

- Movement commands expressed in physical terms
- Reusable rotation functions
- Better separation between desired motion and execution duration
- Access to an estimated robot pose
- Greater potential for state-aware navigation

However, these advantages come with additional complexity and calibration requirements.

---

## Odometry Limitations

Odometry is an estimate, not an absolute measurement of the robot's real-world position.

Its accuracy depends on assumptions about the physical robot.

Error can be introduced by:

- Wheel slip
- Incorrect wheel radius
- Different effective wheel diameters
- Incorrect wheel-base measurement
- Mechanical tolerances
- Floor conditions
- Wheel deformation
- Motor-position measurement error
- Robot collisions or external disturbances

Small errors can accumulate over time.

For example, if a wheel slips while rotating, the motor telemetry may indicate wheel movement even though the robot did not rotate by exactly the predicted amount.

Therefore:

> Odometry provides useful motion feedback, but it does not eliminate the need for physical testing, calibration, and external sensing.

---

## Navigation Logic

The main execution loop continuously reads the sensors and determines which behavior should run.

The current priority is approximately:

    Read sensors
         |
         v
    Update odometry
         |
         v
    Check obstacle
         |
         v
    Check color marking
         |
         v
    Check intersection
         |
         v
    Check left/right 90-degree candidate
         |
         v
    Check line gap
         |
         v
    Check red stop condition
         |
         v
    Normal PID line following

Special situations interrupt normal PID line following and execute dedicated navigation logic.

---

## Track Handling

### Intersections

The current code can identify a clear intersection when all digital line-sensor channels are active.

Dedicated intersection handling is then executed.

### Possible 90-Degree Turns

The software detects characteristic left-side and right-side sensor patterns.

The robot performs an additional check to determine whether the center line continues.

Depending on the result, the situation can be treated as an intersection or as a sharp turn.

Odometry-based rotation functions are used for confirmed 90-degree rotations.

### Green Markings

The two color sensors are sampled to classify green markings.

The current logic can return:

- `LEFT`
- `RIGHT`
- `180`

The corresponding navigation behavior is then executed.

Some detection movements involved in this process are still time-based.

### Red Marking

The current main loop stops the robot when both color sensors report red.

### Line Gaps

When the line sensor reports that the line is no longer detected, the robot attempts to move forward while searching for it.

If the line cannot be recovered during the attempt, the robot returns approximately over the movement performed and starts lost-line recovery.

This behavior currently combines sensor feedback, odometry updates, and time-based movement.

### Lost-Line Recovery

The program stores the previous line position.

When recovery is required, the last known position is used to determine the initial search direction.

The robot then searches until the central line-sensor channel detects the line again.

### Obstacles

The distance sensor triggers obstacle handling below a configured distance threshold.

The current maneuver combines:

- Odometry-based rotations
- Timed straight segments
- Line-sensor reacquisition

Therefore, the obstacle routine should not currently be described as fully closed-loop or fully odometry-based.

---

## Current Implementation vs. Experimental Direction

This repository is **odometry-oriented**, but it is not a completely odometry-controlled navigation system.

The current code already uses estimated angular displacement for rotation functions.

At the same time, several behaviors still contain calibrated timing, including:

- Straight movement helpers
- Some line-search movements
- Color-marking scans
- Parts of obstacle avoidance
- Short positioning movements

This mixed architecture reflects the current experimental state of the project.

A future evolution could replace some of these time-based translations with distance-based movement functions where this provides a meaningful advantage.

---

## Project Structure

    line-follower-odometry-python/
    ├── scripts/
    │   ├── calibrate_line_sensor.py
    │   ├── get_devices.py
    │   ├── remote_control.py
    │   ├── test_line_sensor.py
    │   └── walk_straight.py
    │
    ├── src/
    │   ├── CommandDriver.py
    │   ├── line_functions.py
    │   ├── main.py
    │   ├── odometry.py
    │   └── pid.py
    │
    └── README.md

---

## Core Modules

### `src/main.py`

Main application entry point.

It:

- Initializes Open-RDK
- Connects to motors
- Connects to the line sensor
- Connects to the distance sensor
- Connects to both color sensors
- Creates the PID controller
- Creates the odometry object
- Reads sensors continuously
- Selects navigation behaviors
- Stops the motors safely when execution ends

### `src/line_functions.py`

Contains most of the robot navigation logic.

Current responsibilities include:

- Motor helper functions
- Odometry updates
- Intersection detection
- Sharp-turn detection
- Color detection
- 90-degree and 180-degree maneuvers
- PID line following
- Gap crossing
- Lost-line recovery
- Obstacle handling

### `src/odometry.py`

Implements differential-drive odometry.

Current parameters include:

- Wheel radius
- Wheel base
- Last left motor position
- Last right motor position

The class provides:

- Pose updates
- Position retrieval
- Reset
- Normalized angle difference calculation

### `src/pid.py`

Custom PID implementation for line following.

It includes:

- `kp`
- `ki`
- `kd`
- Set point
- Integral state
- Previous error
- Previous timestamp
- Output limiting

### `src/CommandDriver.py`

Provides asynchronous handling of motor commands through a background worker thread.

The class keeps the latest pending speed command and provides methods for:

- Setting motor speed
- Stopping
- Shutting down the command worker

---

## Utility Scripts

The `scripts/` directory contains independent hardware utilities.

These are manual hardware tools, not an automated unit-test suite.

### `calibrate_line_sensor.py`

Configures the line sensor for a dark track, performs calibration through Open-RDK, and saves the resulting calibration.

Run with:

    python scripts/calibrate_line_sensor.py

### `get_devices.py`

Lists devices detected by the Open-RDK runtime.

Run with:

    python scripts/get_devices.py

### `test_line_sensor.py`

Used to inspect line-sensor readings independently from the full autonomous navigation program.

Run with:

    python scripts/test_line_sensor.py

### `remote_control.py`

Provides keyboard-based manual motor control.

Current commands include forward, reverse, left rotation, right rotation, stop, and exit.

It can be used during hardware verification and mechanical testing.

Run with:

    python scripts/remote_control.py

### `walk_straight.py`

Runs both motors together as a simple straight-movement hardware test.

Run with:

    python scripts/walk_straight.py

---

## Testing Strategy

The current repository does **not yet contain an automated `tests/` suite**.

Testing is currently focused primarily on hardware-level validation using the utility scripts and direct robot experiments.

The existing scripts help isolate individual subsystems before running the complete navigation logic.

A useful development sequence is:

    Device Detection
          |
          v
    Sensor Calibration
          |
          v
    Sensor Validation
          |
          v
    Manual Motor Test
          |
          v
    Straight Movement Test
          |
          v
    Integrated Navigation

Future automated tests would be particularly useful for hardware-independent logic such as:

- PID calculations
- Odometry equations
- Angle normalization
- Sensor-pattern classification
- Navigation decision logic

These tests are future work and are not currently present in the repository.

---

## Relationship with Team LOGOS

This project was developed by **Rafael Lima Ribeiro dos Santos** during the same general robotics-development period in which he participated as Captain of Team LOGOS in OBR 2026.

It is related to the same field of study and many of the same robotics problems, but it is **not the software used by Team LOGOS during the competition**.

The official Team LOGOS repository is:

https://github.com/LOGOSIFBA/LOGOS

The distinction is important.

### Team LOGOS Repository

The Team LOGOS project represents:

- A collective engineering effort
- The robot prepared and tested for competition
- Integration of software, electronics, mechanics, and strategy
- Extensive physical validation
- Competition-oriented decisions
- Several special maneuvers based on calibrated timing

### This Repository

This repository represents:

- Rafael's individual implementation
- A software architecture with greater emphasis on odometry
- Movement-estimation experimentation
- Odometry-based rotation control
- Separate hardware utility scripts
- Continued experimentation outside the competition strategy

The **Extra Innovation Award received at OBR 2026 belongs to Team LOGOS and the team's work**, not to this personal repository.

---

## Engineering Trade-Off

The existence of both repositories illustrates a practical engineering trade-off.

A more sophisticated control architecture can provide useful abstractions and additional information about robot state.

However, sophistication alone does not make a solution more suitable for competition.

The Team LOGOS competition strategy had a major advantage:

**physical validation.**

The team had performed significantly more real-world testing with the time-calibrated maneuvers used on the competition robot.

That provided:

- Known behavior
- Better-understood failure modes
- Proven calibration
- Greater confidence before competition
- Lower integration risk

This repository explores an alternative with greater dependence on movement estimation.

That can make commands such as rotation more directly related to physical motion, but it introduces additional sources of uncertainty:

- Wheel-model accuracy
- Geometry calibration
- Encoder or motor telemetry interpretation
- Wheel slip
- Accumulated error
- Increased software complexity

Therefore, the trade-off is not:

> simple solution vs. good solution

or:

> time-based control vs. correct control

It is closer to:

> extensively validated open-loop movement vs. a more state-aware approach that still requires sufficient calibration and physical validation.

For a competition robot, a simpler and thoroughly tested strategy can be the better engineering choice.

For an experimental project, a more elaborate architecture can be valuable because it provides room to study new control techniques.

Both approaches can therefore coexist for valid reasons.

---

## Technologies

Current software and concepts include:

- Python
- Open-RDK
- Embedded robotics
- PID control
- Differential-drive kinematics
- Differential-drive odometry
- Motor position telemetry
- Infrared line sensing
- RGB/color sensing
- Distance sensing
- Autonomous navigation
- Motor control

---

## Running the Project

Clone the repository:

    git clone https://github.com/Rafael-lima-devbr/line-follower-odometry-python.git

Enter the project directory:

    cd line-follower-odometry-python

Run the main program:

    python src/main.py

The project requires:

- Compatible robot hardware
- Open-RDK
- Correct motor and sensor configuration
- Correct device serial numbers
- A calibrated line sensor

There is currently no dependency file such as `requirements.txt` in the repository, so environment setup is not yet fully automated or documented.

---

## Limitations

The current project should be considered experimental.

Important limitations include:

- Odometry accumulates error
- Wheel slip is not directly corrected
- Pose estimation depends on wheel geometry calibration
- Some movements remain time-based
- There is no absolute localization system
- There is currently no sensor fusion
- There is no automated unit-test suite
- Hardware serial numbers are currently configured directly in the source code
- Navigation behavior still requires extensive physical validation
- Movement parameters remain dependent on the physical robot

The estimated `x`, `y`, and `theta` values should therefore not be interpreted as exact ground-truth position.

---

## Future Work

Potential future development includes:

- Distance-based straight movement using odometry
- Further odometry calibration
- Comparison between estimated and measured physical displacement
- Improved rotation accuracy
- Reduction of remaining time-dependent maneuvers where appropriate
- Better navigation-state organization
- Automated unit tests
- Sensor-pattern tests
- PID tests
- Odometry tests
- Hardware configuration separated from source code
- Better recovery strategies
- Investigation of sensor fusion
- Quantitative comparison between timing-based and odometry-oriented movement

These are development directions and are not presented as currently implemented features.

---

## Author

**Rafael Lima Ribeiro dos Santos**

Personal robotics project developed entirely by Rafael.

GitHub:  
https://github.com/Rafael-lima-devbr
```
