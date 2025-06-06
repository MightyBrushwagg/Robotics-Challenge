# Robotics Challenge: Obstacle Course

This repository contains the codebase and documentation for Team 404 (Group 9) in the UCL Robotics and AI Year 1 Challenge. The team members are Morgan, Ian, Xavier, and Helitha.

Our robot, **Paxman**, is designed to autonomously traverse a complex obstacle course using a combination of advanced mobility, sensors, and preprogrammed logic. Paxman features:

- **Passively transforming wheels** to assist in climbing over uneven terrain while being smooth on even surfaces.
- **Ackermann steering and differential drive**, allowing both agile turning and stable movement.
- A **servo-actuated gripper** mechanism to hook onto elevated structures such as the lava pit and zipline for obstacle traversal.

---

## Repository Structure

- Trial run: Component-level test scripts used during early development

- wall following versions: Iterative prototypes of the wall-following algorithm

- control versions: Full control scripts including line following, crossroad handling, and obstacle interaction

- Final_control.ino: The final version of the code deployed to the Arduino for the obstacle course challenge
---

## How to Operate Paxman

1. **Start/Stop:**  
   - Press the onboard button to start the robot.
   - Press again to stop.
   - Alternatively, a `"stop"` command received over WiFi will halt the robot.

   These inputs are handled by the `button_check()` and `wifi_check()` functions.

2. **Navigation Modes:**  
   The robot operates in four distinct modes, each represented by an integer:
   - `MODE_DEAD`: Robot is stopped (kill switch activated).
   - `MODE_LINE_FOLLOWING`: Follows the black line using reflectance sensors.
   - `MODE_CROSSROAD_TAKING`: Executes a predefined direction at a detected crossroad.
   - `MODE_WALL_FOLLOWING`: Follows walls using IR distance sensors.

3. **Line Following and Crossroads:**  
   - Crossroads must be preprogrammed in the `crossroadDirections[]` array (e.g., `{LEFT, STRAIGHT, RIGHT}`).
   - Functions involved:
     - `auto_tracking()`: Main PID-based line following.
     - `check_crossroad()`: Detects forks and crossroads.
     - `take_crossroad()`: Executes the programmed direction.

4. **Wall Following:**  
   - The `wall_following()` function uses data from IR sensors to navigate parallel to walls.
   - If the robot encounters a frontal obstacle, `turn_right()` is invoked to change direction.

5. **Obstacle Handling (Lava Pit & Zipline):**  
   - The robot uses the `lifting()` function to raise its gripper mechanism and latch onto elevated supports.
   - Once hooked, the robot drives forward to cross the obstacle.

---

## Core Sensor Functions

- `Reflectance_Sensor_Reading()`:  
  Populates `sensorValuesFull[]` with raw discharge times from each reflectance sensor.  
  Applies a threshold to produce binary readings in `sensorValuesBinary[]`.
  Both arrays are stored in the same order and direction as the actual robot.

- `IR_Sensor_Reading()`:  
  Reads analog values from IR distance sensors and estimates distances (in cm), stored in `distanceIR[]`.

---

## Motor Control Functions

- `go_Advance_4_wheel(int leftFrontSpeed, int rightFrontSpeed, int leftBackSpeed, int rightBackSpeed)`:  
  Controls all four motors individually.

- `go_Advance_2_wheel(int leftSpeed, int rightSpeed)`:  
  Controls the left and right side motors together for simpler movement.

- `stop_Stop()`:  
  Stops all motors immediately, alternatively either of the previous two functions can be called with speed 0 for all motors.

---

## Notes

- The PID controller for line following is tuned using:
  - `Kp = 30`
  - `Ki = 0`
  - `Kd = 0`

- Reflectance sensor data is processed in real-time to determine lateral error, which is corrected by PID output.
