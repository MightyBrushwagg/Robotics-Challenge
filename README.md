# Robotics-Challenge: Obstacle Course

This is the repository for team 404 (group 9) in the UCL RAI first year robotics challenge.

The team members are Morgan, Ian, Xavier and Helitha.

Our robot Paxman utilises passively transforming wheels to climb and an ackerman steering differential drive combo to move. A gripper to hook onto any high lines (lava pit and zipline) to get over obstacles.

Final_control.ino is the script that was loaded into the arduino for the run.

The folder structure is as follows:

- Trial run: Individual scripts for components, specifically for the first trial run
- wall following versions: Successive code prototypes for our wall following algorithm
- control versions: The main code that goes on the robot for the challenge, includes all line following logic



How to use the robot: 
-
First, to set the robot off, press the button. Pressing the button again stopped the robot, alternatively, it could receive the message "stop" over wifi which had the same effect. button_check() and wifi_check() were the two funcitons used for the kill switch testing.

To do line following that involves crossroads and turns, the route must be preprogrammed in crossroadDirections as it may not pick the right path otherwise.

All four motors could be controlled separately or left and right can be controlled by using  go_Advance_4_wheel(int leftFrontSpeed, int rightFrontSpeed, int leftBackSpeed, int rightBackSpeed) and go_Advance_2_wheel(int leftSpeed, int rightSpeed) respectively. To stop all motors, either set those speeds to 0 or use stop_Stop() which does that.

To get readings from the reflectance sensors, use the function Reflectance_Sensor_Reading() which stores values from all the reflectance sensors in the array sensorvaluesFull in the same order they are on the robot. In addition, by using a threshold (ReflectanceThreshold), sensorValuesBinary tracks whether a sensor is seeing black or white line. These are accessed by the line following logic.

IR_Sensor_Reading() does the same as the reflectance sensor reading function, the distances are stored in distanceIR.

The robot has 4 robotMode:

- MODE_DEAD: motors stopped, kill switch activated
- MODE_LINE_FOLLOWING, follows line using algorithm
- MODE_CROSSROAD_TAKING, is taking a crossroad
- MODE_WALL_FOLLOWING, follows wall using algorithm

Control algorithms:
-

take_crossroad(), check_crossroad and auto_tracking() do the line following.
check_crossroad tells the robot if it is at a crossroad, if it is, take_crossroad takes over and the robot takes the crossroad from the predefined path. Otherwise auto_tracking takes over and the robot moves following the line.

wall_following() contains all the logic to follow the wall when there is no line to follow, utilising the turn_right() function when needed.

To hook onto the lava pit and zipline, the robot uses the lifting() mechanism which extends the arms up to grip on, it then drives forward












