#ROS

ROS packages for the Cerus mobile robot developed in ROS Noetic. These packages have been tested on a Jetson Nano 4GB with an Arduino MEGA 2560. 

##cerus_teleop_joy

Enables remote control of Cerus. Teleop has been tested with an XBOX One controller and should work with any other Linux-compatible controller but the axis mapping might differ.
To install cerus_teleop_joy, download the folder to your catkin_ws folder. Then compile your workspace:

```cd ~/catkin_ws/
catkin_make```

