![Cerusbot](https://github.com/2788west/cerusbot/blob/master/cerusbot.jpg?raw=true)

# Cerusbot

Cerus is a four-wheeled mobile robot platform. It operates based on an Arduino Mega 2560 and NVIDIA Jetson Nano. The code in this repo enables both teleoperation via an Xbox controller as well as a simple go-to-goal behaviour with encoder feedback. Even though Cerus features mecanum wheels, this code treats the robot's base as a differential drive system, pairing left and right wheels together.

## Hardware Overview
![Block Diagram](https://raw.githubusercontent.com/2788west/cerusbot/8304e2776a1b0b4a7896b6be7f84befea37baf97/block_diagram.svg)


## arduino

Enables low-level motor control via PWM and reads two motor encoders (left side and right side). The Arduino will accept linear and angular speed in the format `<float, float>` (based on the [ROS Twist Message](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)) and continuously return the left and right wheel position as a pair of integer values `<int, int>`. 

## jetson

Enables two behaviors: Teleop and Go-To-Goal.

### Teleop

To teleoperate Cerus via an XBox controller or similar gamepad, simply run `gamepad_control.ipynb` in Jupyter. The left and right analog stick will control the wheels on either side of the robot

### Go-To-Goal

TK
