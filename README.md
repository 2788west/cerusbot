![Cerusbot](https://github.com/2788west/cerusbot/blob/master/cerusbot.jpg?raw=true)

# Cerusbot

Cerus is a four-wheeled mobile robot platform. It operates based on an Arduino Mega 2560 and NVIDIA Jetson Nano. The code in this repo enables both teleoperation via an Xbox controller as well as a simple go-to-goal behaviour with encoder feedback. Even though Cerus features mecanum wheels, thes basic Arduino and Jetson Nano code treats the robot's base as a differential drive system, pairing left and right wheels together. This simplification is intended to keep the code more beginner-friendly.

## Hardware Overview
![Block Diagram](https://raw.githubusercontent.com/2788west/cerusbot/8304e2776a1b0b4a7896b6be7f84befea37baf97/block_diagram.svg)

## arduino
`move.cpp` enables low-level control of four motors using Cytron motor drivers and reads two quadrature motor encoders (left side and right side). The Arduino will accept linear and angular speed in the format `<float, float>` and continuously return the left and right wheel position as a pair of integer values `int, int`. This code is intended to be used with the standard Teleop and Go-To-Goal notebooks. To use this code with a Cytron motor driver, please add their driver library in the Arduino IDE under Tools > Manage Libraries > Search for: "Cytron Motor Drivers Library".

## jetson

Enables two behaviors: Teleop and Go-To-Goal.

### Teleop

To teleoperate Cerus via an XBox controller or similar gamepad, simply run `gamepad_control.ipynb` in Jupyter. The left and right analog stick will control the wheels on either side of the robot.

### Go-To-Goal

![Go-To-Goal Behavior](https://raw.githubusercontent.com/2788west/cerusbot/c68817b593123b444a1b4bb1e494d2b1625c4db6/go-to-goal_behavior.svg)

## Make your own
With the parts in the bill of materials (BOM.csv) and the 3D print files (STL.zip) you can build your own version of Cerus!

# Cerusbot with ROS
The ROS directory contains more advanced code for teleoperation that takes full advantage of Cerus' mecanum wheels and enables movement in the X and Y direction as well as around the Z axis. 
