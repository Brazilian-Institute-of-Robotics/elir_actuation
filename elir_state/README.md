# Elir State

This repository contains the code to create the elir_state topic, in order to be used with MoveIt!

## Functionality
It uses the controler manager motor_states and the joint offsets of each joint yaml to create an ROS
specific structure containing the robot joints and their values in rad

## Avaiable Applications
Elir_state launch:

```
$ roslaunch elir_state robot_state.launch
```