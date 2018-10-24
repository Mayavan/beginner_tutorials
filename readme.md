# beginner_tutorials

This repository contains the files from the ROS tutorial on publisher and subscribers.

There are two nodes created in the package, a talker node which publishes data and a listener node which subscribes to the topic to listen to the published data. The published string has been modified to a custom string.

---
## Dependency:
* [ROS Kinetic](http://wiki.ros.org/ROS/Installation) on Ubuntu 16.04

## Standard install via command-line
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ cd src/
$ git clone https://github.com/Mayavan/beginner_tutorials.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
To run talker node:
```
$ roscore
$ rosrun beginner_tutorials talker
```
To run listener node:
```
$ roscore
$ rosrun beginner_tutorials listener
```