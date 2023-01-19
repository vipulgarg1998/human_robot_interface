# human_robot_interface
This project provides an interface to control and monitor the state of BlueROV. 

## Teleoperation
The connected ROV can be controlled using the keyboaard using the following command
```
rosrun human_robot_interface teleop_bluerov.py
```
Assuming the coordinate frame attached to BueROV follows the standard notation where x axis
is from stern to bow, y axis is from port to starboard, and z axis is from top to botton, then, 
- w for surge in positve direction
- s for surge in negative direction
- d for sway in positive direction
- a for sway in negative direction
- q for heave in negative direction
- z for heave in positive direction
- l for yaw in positive direction
- j for yaw in negative direction

Special keys for advance options
- f1 to switch between autonomous and manual mode
- Esc to quit

#### Note: Default mode of operation is AUTO

## Monitoring
A rosnode is dedicated to subscribe the sensors and check if there is any leak or not. It warns the operator if any leak is detected. 
To run the node, run the following
```
rosrun human_robot_interface sensors.py
```

## Visual Servoing
The BlueROV can run in autonomous mode where it detects the Aruco Marker and move towards the marker. It uses 3 PID controllers for surge, sway, and heave motion.

To run the ROV in auto mode, run the following
```
roslaunch human_robot_interface visual_servo.launch
```

## Camera Calibration:
To run obtain the camera calibration matrix, you can simply run the python script 'cam_calibrate.py'. But first you need to put all images in a specific folder and specifiy the path to this folder inside the script by updating the variable 'images'. It will take some time to finish the calibration process and the result will be the camera calibration matrix


