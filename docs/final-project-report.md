## Overview

The [project](https://github.com/bachmak/robbi) consists of two parts:
* [teensy-part](https://github.com/bachmak/robbi/tree/main/src/teensy): motor-rotation control and configuration
* [raspberry-part](https://github.com/bachmak/robbi/blob/main/ros_nodes/labyrinthi.py): high-level logic to navigate the robot through the labyrinth

## General idea

The raspberry-part (implemented as a ROS2 node) contains a hard-coded [plan](https://github.com/bachmak/robbi/blob/main/ros_nodes/labyrinthi.py#L44) of the given labyrinth and sends two types of commands via a ROS2-topic:
1) `move d t` - to drive `d` meters (can be negative) straight, within `t` seconds
2) `rotate d t` - to rotate `d` degrees (can be negative), within `t` seconds

The teensy-part calculates necesarry rotation angles and velocities for each motor, based on the accepted commands from ROS2 and controls the rotation.

## Teensy-part in details

### Code structure

1. [robot/](https://github.com/bachmak/robbi/tree/main/src/teensy/ros) contains entities specific to robot movement control and configuration: `Robot`, `Motor`, and motor control states:
    - `VelocityControl` for maintaining a desired linear and angular velocity for unlimited time (**not used in the final project**)
    - `PositionControl` for rotating a wheel towards the specified position within the given time interval.

2. [ros/](https://github.com/bachmak/robbi/tree/main/src/teensy/ros) contains C++-primitives wrapping a C-library for interaction with ROS2 to make further work with the library simpler and less error-prone through increased type-safety and encapsulation.

3. [utils/](https://github.com/bachmak/robbi/tree/main/src/teensy/utils) contains helper functions and classes not specific to the robot details.

4. [main.cpp](https://github.com/bachmak/robbi/blob/main/src/teensy/main.cpp) creates and initializes the used entities (Robot, ROS2-subscribtions and -publishers, logging, debug, etc.) implements the main loop and checks for connection to the micro-ROS2 agent.

### Execution flow

The execution is organised to keep blocking at minimum, i.e., teensy always stays interactive, accepting new commands and controlling the movement. For this, the used approach is to have non-blocking `update(dt)` methods called continiously and `set_x(x)` methods called on-demand and affecting the behaviour.

### Important entities

1. `Robot` contains the two motors (left and right) and is responsible for converting high-level commands to low-level commands for each motor, taking into account the geometrical parameters (wheel-radius, distance between the wheels) and the wheel orientation (if the wheel is inverted or not).
**Converstion**:
    - `move D m in T sec` -> `left: rotate to X deg in T sec` and `right: rotate to -X deg in T sec`,
    - `rotate D deg in T sec` -> `left, right: rotate to X deg in T sec`,
    -  `rotate with linear and yaw velocity L m/sec and Y deg/sec` -> `left: rotate with velocity X deg/sec` and `right: rotate with velocity Y deg/sec` (not used in the final project).

2. `Motor` is responsible for:
    - choosing appropriate control algorithm (control state) based on the required behaviour (`VelocityControl` if the target is velocity, `PositionControl` if the target is distance),
    - interaction with the servo-pins using the `Servo.h` library, i.e., reading from the feedback-pin and calculating the current angle and writing PWM-values from the current control algorithm to the control-pin.

3. `states::PositionControl`
    - calculates the setpoint (trapezoidal) trajectory to allow for bumpless rotation provided the target position, target time, and the configured acceleration/deceleration,
    - implements the control algorithm (feed-forward on required velocity and P-part on the position error), provides the current PWM value for `Motor`.

4. `states::VelocityControl` (**not used in the final project**)
    - ramps setpoint change to ensure bumpless transition to the new target velocity,
    - implements the control algorithm (feed-forward on required velocity and P-part on the velocity error), provides the current PWM value for `Motor`.

5. `Controller` is basically a ROS2-subscription, responsible for accepting and parsing (if applicable) movement commands (`Twist`, `move`, and `rotate`) and applying them through setting new target for `Robot`.

6. `Configurator` is another ROS2-subscription based class that allows runtime configuration adjustments of all settings to simplify fine-tuning without having to recompile and reupload the firmware. The settings visitors in `settings_visitor.h` allow hierarchical access to settings which is used by corresponding getters and setters in `settings.cpp`. A typical ROS2-message to log the current setting value looks as follows: `data: 'left.motor.pwm-deadband-fwd get'`; respectively, the corresponding message to set the setting: `data: 'left.motor.pwm-deadband-fwd 25'`.

## Raspberry-part in details

The [ROS2-node](https://github.com/bachmak/robbi/blob/main/ros_nodes/labyrinthi.py) is implemented as a simple state machine, with three states always changing sequentially: `MOVE` -> `WAIT` -> `ROTATE` -> `MOVE`.

To each state, there is a pair of corresponding `enter_` and `update_` methods for one-time initialization and continious updating respectively.

### Move-state
In the beginning of the `MOVE` state, the next section of the predefined labyrinth is fetched and set as the current one. A `move` command is sent subsequently to make the robot drive along a straight line. While the robot is moving, the ROS2-node is checking if the time required for the movement is over.

### Wait-state
The `WAIT` state has the only reason to make a short stop to demonstrate the current position.

### Rotate-state
On entering the `ROTATE` state, a `rotate` command is sent. Afterwards, similarly to the `MOVE` state, the elapsed time is checked against the time required for the rotation.

### Sections
Although the teensy-part provides good movement-precision, it was not fine-tuned perfectly, so that the three used motor-rotation combinations
- left motor forward, right motor forward (right turn)
- left motor forward, right motor backward (straight movement)
- left motor backward, right motor forward (left turn)
- ~~left motor backward, right motor backward~~ (not used)

were symmetrically aligned. So, the values (mostly, the angles of the turns) were adjusted to ground-truth experiments.

## Usage of AI

Although some of the code parts were generated by AI, they are very restricted and only implement straightforward algorithmic/computational tasks:
- conversion functions in [robot.cpp](https://github.com/bachmak/robbi/blob/main/src/teensy/robot/robot.cpp)
- some string conversion functions in [utils/str.cpp](https://github.com/bachmak/robbi/blob/main/src/teensy/utils/str.cpp)
- some conversion functions in [utils/geometry.cpp](https://github.com/bachmak/robbi/blob/main/src/teensy/utils/geometry.cpp)

Apart from that, all the code was designed, written, and checked by me.