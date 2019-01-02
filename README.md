# Elir Actuation 

This repository contains the packages necessary for the ELIR Robot actuation.


## Installation
The installation of the package elir_simulation already contains all the necessary packages

## Included Packages and features
* control_services - Services for specific movements, like opening the claw
* elir_actuation - Robot Controllers Spawner
* elir_line_control - Robot line control
* elir_state - Creates one robot_state topic for elir robot
* [SKETCH] joy_control - SKetch package for robot control using joystick

## Avaiable Applications

Controllers spawner:

```
$ roslaunch elir_actuation elir_controllers.launch
```

Gazebo Simulation:
```
$ roslaunch elir_gazebo elir_world.launch
```

MoveIt! Interaction with gazebo, launch your world then:
```
$ roslaunch moveit_with_gazebo new_moveit_planning_execution.launch
```

Elir robot_state topic:
```
$ roslaunch elir_state robot_state.launch
```

Line horizontal displacement with keyboard inputs
```
$ roslaunch elir_line_control line_control.launch
```

Line stretch in order to move up the central unit for object trepassing tested in simulation with keyboard input
```
$ roslaunch elir_line_control robot_stretch.launch
```