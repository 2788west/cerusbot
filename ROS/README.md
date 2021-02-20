# ROS

ROS packages for the Cerus mobile robot developed in ROS Noetic. These packages have been tested on a Jetson Nano 4GB with an Arduino MEGA 2560. 

## firmware

The firmware is intended to run on the Arduino Mega 2560 and handles low-level motor control. It's written to work with Cytron 3A motor drivers, but can easily be changed to work with a different type of driver.

## cerus_teleop_joy

Enables remote control of Cerus. Teleop has been tested with an XBOX One controller and should work with any other Linux-compatible controller but the axis mapping might differ.
To install cerus_teleop_joy, download the folder to your catkin_ws folder. Then compile your workspace and source your setup.bash file:

```sh
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Then you can simply launch the package with:

```sh
roslaunch cerus_teleop_joy cerus_teleop.launch
```

You should be able to drive the robot around. Enjoy.
Please note that I am not a ROS expert and some of this code may not work as expected. Always use caution when working on a live robot.
