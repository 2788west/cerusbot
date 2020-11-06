![Cerusbot](https://github.com/2788west/cerusbot/blob/master/cerusbot.jpg?raw=true)

# Cerusbot

Cerus is a four-wheeled mobile robot platform. It operates based on an Arduino Mega 2560 and NVIDIA Jetson Nano. The code in this repo enables both teleoperation via an Xbox controller as well as a simple go-to-goal behaviour with encoder feedback. Even though Cerus features mecanum wheels, this code treats the robot's base as a differential drive system, pairing left and right wheels together.

## System Overview
![Block Diagram](https://raw.githubusercontent.com/2788west/cerusbot/c17cc8c20f015579d99ba5fb8604a7d372f4df92/block_diagram.svg)


## arduino

Enables low-level motor control via PWM and reads two motor encoders (left side and right side). The Arduino will accept linear and angular speed in the format `<float, float>` (based on the [ROS Twist Message](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) and continuously return the left and right wheel position as a pair of integer values `<int, int>`. 

## jetson
