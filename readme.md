# beginner_tutorials

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---
This repository contains the files from the ROS tutorial on publisher and subscribers modified to include a service in the talker node which allows you to set the message being published.

The launch file parameter can be used to set the frequency at which the publisher publishes the message.

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

## To run demo

### Run using launch file

```
roslaunch beginner_tutorials full.launch
```

Run using launch file and pass parameter value for frequency

```
roslaunch beginner_tutorials full.launch hertz:=1
```

### Run calling individual nodes

To start ROS master:

```
$ roscore
```

To run talker node:

```
$ rosrun beginner_tutorials talker
```

To run listener node:

```
$ rosrun beginner_tutorials listener
```

### To call service to set message

```
$ rosservice call /setMessage "message: 'Mayavan'"
```
