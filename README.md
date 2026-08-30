# Autonomous Line-Following Robot with Odometry

A personal **Python robotics project** focused on autonomous line-following navigation using **PID control, infrared sensing, motor control, and odometry-based movement estimation**.

This repository contains my own implementation and experimentation with a more structured navigation approach, where odometry is used to estimate robot displacement and rotation instead of relying exclusively on fixed movement times.

The project was developed alongside my participation as **Captain of Team LOGOS** during the **2026 Brazilian Robotics Olympiad (OBR)**.

However, this repository is **not the same software used by Team LOGOS during the competition**.

The competition robot used a separate repository and prioritized extensively tested time-based movement strategies.

This project represents my personal exploration of a more advanced and odometry-driven approach.

---

# Motivation

One of the challenges in autonomous robotics is performing movements such as rotations and controlled straight motion consistently.

A simple strategy is to control movement through execution time.

For example:

~~~python
motors.move()
time.sleep(0.5)
motors.stop()
~~~

This approach can work extremely well when extensively tested.

In fact, this type of strategy was preferred for several behaviors in the Team LOGOS competition robot because it had received significantly more physical validation before OBR 2026.

However, time-based movement has limitations.

The physical displacement produced during the same amount of time may vary according to:

- Battery voltage
- Motor response
- Surface friction
- Robot weight
- Wheel traction
- Mechanical differences
- Environmental conditions

This project explores another approach.

Instead of asking:

> How long should the motor run?

The objective is increasingly to ask:

> How far has the robot actually moved?

or:

> How much has the robot actually rotated?

This is where **odometry** becomes important.

---

# Project Goals

The main objectives are:

- Implement autonomous line following
- Develop a custom PID controller
- Estimate robot displacement using odometry
- Estimate robot rotation
- Reduce dependency on fixed movement times
- Develop modular navigation logic
- Detect different track situations
- Create independent hardware testing tools
- Explore more deterministic autonomous navigation

---

# System Overview

The project combines:

- Infrared line sensing
- PID control
- Differential motor control
- Odometry
- Track interpretation
- Autonomous decision-making

The general architecture can be represented as:

~~~text
             Line Sensor
                  │
                  ▼
          Track Interpretation
                  │
                  ▼
          Decision-Making Logic
                  │
           ┌──────┴──────┐
           │             │
           ▼             ▼
      PID Control    Special Logic
           │             │
           └──────┬──────┘
                  ▼
             Motor Control
                  │
                  ▼
               Odometry
                  │
                  ▼
          Movement Estimation
~~~

---

# PID Line Following

During normal navigation, the robot uses a PID controller to remain aligned with the line.

The infrared sensor provides the estimated line position.

The controller calculates the difference between the target position and the detected position.

Conceptually:

~~~text
error = target_position - measured_position

correction = PID(error)

left_motor  = base_speed + correction
right_motor = base_speed - correction
~~~

This correction is continuously recalculated while the robot moves.

The PID controller helps improve:

- Stability
- Curve response
- Direction correction
- Line-following precision

---

# Odometry

Odometry is the main experimental focus of this repository.

Instead of estimating movement exclusively through time, the robot attempts to estimate movement using wheel displacement.

For a differential-drive robot:

~~~text
Left Wheel Movement
         +
Right Wheel Movement
         │
         ▼
Differential Drive Model
         │
         ▼
Linear Displacement
         +
Angular Displacement
         │
         ▼
Estimated Robot State
~~~

Using the relative displacement of both sides of the robot, it becomes possible to estimate:

- Distance traveled
- Rotation
- Orientation changes
- Robot position

---

# Why Use Odometry?

Consider a time-based rotation:

~~~python
rotate()
time.sleep(0.7)
stop()
~~~

The assumption is that 0.7 seconds corresponds to a known rotation.

But that relationship may change when:

- The battery becomes weaker
- The wheels slip
- The floor changes
- Robot weight changes
- Motor performance differs

An odometry-oriented approach instead attempts to perform:

~~~text
Rotate until estimated_angle ≈ target_angle
~~~

Similarly, straight movement can conceptually become:

~~~text
Move until estimated_distance ≈ target_distance
~~~

This creates the possibility of navigation based on the estimated physical state of the robot rather than only elapsed time.

---

# Current Features

The project currently includes:

- PID-based line following
- Infrared line sensor reading
- Motor control
- Differential-drive odometry
- Movement estimation
- Straight movement control
- Controlled rotation
- Intersection detection
- Line gap detection
- Possible 90-degree turn detection
- Lost-line recovery

---

# Track Handling

Normal PID line following is not sufficient for every track condition.

The project therefore contains dedicated navigation logic.

---

## Intersections

Sensor readings can indicate that multiple possible line directions exist.

The system can detect these patterns and execute dedicated intersection behavior.

---

## Line Gaps

Temporary interruptions in the line can occur.

The robot attempts to distinguish between:

- A real lost-line situation
- A temporary line gap

When a gap is suspected, the robot can attempt to continue forward and recover the line.

---

## 90-Degree Turns

Specific sensor patterns can indicate possible sharp turns.

These conditions can trigger dedicated rotation logic.

Odometry can potentially be used to control these rotations based on estimated angle rather than only timing.

---

## Lost-Line Recovery

If the line disappears completely, the robot can execute recovery behavior using previous sensor information and movement context.

---

# Project Structure

~~~text
line-follower-odometry-python/
├── scripts/
│   ├── calibrate_line_sensor.py
│   ├── get_devices.py
│   ├── remote_control.py
│   ├── test_line_sensor.py
│   └── walk_straight.py
│
├── src/
│   ├── main.py
│   ├── pid.py
│   ├── odometry.py
│   └── line_functions.py
│
└── README.md
~~~

---

# Core Modules

## `src/main.py`

Contains the main execution loop.

Responsibilities include:

- Reading sensors
- Updating robot state
- Running PID control
- Updating odometry
- Controlling motors
- Detecting track situations
- Coordinating navigation behaviors

---

## `src/pid.py`

Contains the PID controller.

The implementation handles the components required for trajectory correction:

- Proportional response
- Integral accumulation
- Derivative response
- Output correction

---

## `src/odometry.py`

Contains the odometry implementation.

Its purpose is to estimate robot movement based on wheel displacement.

The module can be used to support:

- Distance-based movement
- Rotation control
- Position estimation
- Autonomous navigation decisions

---

## `src/line_functions.py`

Contains navigation and line-handling functions.

Responsibilities include:

- Line following
- Intersection handling
- Gap detection
- Lost-line recovery
- Possible 90-degree turn detection
- Controlled movement
- Navigation decisions

---

# Utility Scripts

Independent scripts are included to test specific robot components without running the complete autonomous system.

---

## Line Sensor Calibration

~~~bash
python scripts/calibrate_line_sensor.py
~~~

Used to calibrate the infrared line sensor.

---

## Device Detection

~~~bash
python scripts/get_devices.py
~~~

Lists devices detected by Open-RDK.

---

## Line Sensor Test

~~~bash
python scripts/test_line_sensor.py
~~~

Displays sensor readings independently from the main control loop.

Useful for:

- Calibration
- Debugging
- Sensor validation
- Threshold testing

---

## Manual Motor Control

~~~bash
python scripts/remote_control.py
~~~

Allows manual control of the motors.

Useful for checking:

- Motor direction
- Motor response
- Hardware communication
- Mechanical behavior

---

## Straight Movement Test

~~~bash
python scripts/walk_straight.py
~~~

Tests straight movement independently from the complete autonomous navigation logic.

---

# Development Approach

The project follows an incremental development and testing strategy.

Instead of implementing the entire navigation system at once, each component can be validated independently.

~~~text
Device Detection
       │
       ▼
Sensor Calibration
       │
       ▼
Sensor Testing
       │
       ▼
Motor Testing
       │
       ▼
Straight Movement
       │
       ▼
PID Line Following
       │
       ▼
Odometry
       │
       ▼
Special Navigation
       │
       ▼
System Integration
~~~

This makes it easier to identify problems and understand the behavior of each subsystem.

---

# Relationship with Team LOGOS

This project is related to my work in robotics and my participation in **Team LOGOS**, but it should not be confused with the team's competition repository.

The official Team LOGOS repository is:

https://github.com/LOGOSIFBA/LOGOS

Both projects address similar robotics challenges, but they represent different implementations.

---

## Team LOGOS Competition Version

The competition version prioritizes:

- Reliability
- Repeated physical testing
- Predictable movement
- Time-based special maneuvers
- Reduced competition risk

The navigation strategies used there were selected according to the amount of real-world validation available before OBR 2026.

---

## This Repository

This repository prioritizes:

- Odometry
- Movement estimation
- Reduced dependency on fixed timing
- Modular software architecture
- Navigation experimentation
- More state-aware movement control

---

# Engineering Trade-Off

The comparison between both projects demonstrates an important engineering principle:

> The most technically sophisticated solution is not always the best solution for a competition.

A more advanced navigation approach can provide theoretical advantages but still be less appropriate if it has not received enough physical testing.

For OBR 2026, Team LOGOS prioritized the strategy that demonstrated greater reliability on the physical robot.

This personal repository allows me to continue exploring the odometry-based approach without the same competition-time constraints.

---

# Technologies

- Python
- Open-RDK
- Robotics
- Embedded Systems
- PID Control
- Differential Drive Odometry
- Autonomous Navigation
- Infrared Sensors
- Motor Control
- Control Systems
- Software Development

---

# Running the Project

Clone the repository:

~~~bash
git clone https://github.com/Rafael-lima-devbr/line-follower-odometry-python.git
~~~

Enter the project:

~~~bash
cd line-follower-odometry-python
~~~

Run the main program:

~~~bash
python src/main.py
~~~

The robot hardware and Open-RDK environment must be correctly configured before running the project.

---

# Status

This project is currently under development.

Future work may include:

- Improved odometry accuracy
- Better rotation estimation
- Closed-loop distance control
- Improved intersection handling
- Sensor fusion
- More advanced navigation states
- Additional autonomous behaviors
- Comparison between odometry and timing-based navigation

---

# OBR 2026

This project was developed alongside my participation as **Captain of Team LOGOS** in the 2026 Brazilian Robotics Olympiad.

Team LOGOS represented the **Federal Institute of Bahia (IFBA)** and received the:

> 🏆 **Extra Innovation Award — OBR 2026**

The award belongs to the work developed by **Team LOGOS** and should not be interpreted as an award specifically for this personal repository.

---

# Related Repository

## Team LOGOS — OBR 2026

https://github.com/LOGOSIFBA/LOGOS

---

# Author

**Rafael Lima Ribeiro dos Santos**

Industrial Automation Student — IFBA  
Captain — Team LOGOS, OBR 2026

GitHub:

https://github.com/Rafael-lima-devbr
