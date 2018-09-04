# Elir Actuation Package
The actuation package is responsible for the dynamixel controllers, creating the joint controllers and the robot state

#Launch Files

`roslaunch elir_actuation f/b_arm_bringup.launch`

-Launches the controller_manager, the trajectory client and the robot_state files

`dual_servo/servo.launch`

-Launches 2/1 servos simple tilt controller

`single_joint.launch`

-Launches 1 joint of 2 servos

`controller_manager.launch`

-Dynamixel necessary control interface

#YAML files

`arm_joints.yaml`

-Contains the f_arm and b_arm joints parameters

`f/b_arm_trajectory.yaml`

-Contains the f_arm and b_arm trajectory controllers

`servo.yaml`

-Contains the parameter for single servo test

`single_joint.yaml`

-Contains the parameter for single joint test