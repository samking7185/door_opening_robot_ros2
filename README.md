# door_opening_robot_ros2

Simulate a Door Opening Robot and Control with a Joystick

## do_bringup
This package launches the relevant packages for the project.  
The launch file references relevant launch files in each package.

## do_description
This package loads a URDF model of the robot in Rviz.

## do_gazebo
This package loads the URDF model and world file into the Gazebo Simulator.

## do_teleop
This package launches the joystick node and publishes messages to control the motors.

## hardware_interface
The hardware includes raspberry pi's running individual nodes for the system.
Each Pi has its own copy of ROS so the code is not included in this directory.
The Pi's subscribe to /cmd_vel, /robo1_cmd, /robo2_cmd, and /robo3_cmd node.
