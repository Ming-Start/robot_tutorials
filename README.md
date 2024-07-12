# How to play the new added function 

## Build the project using this following README file in ros-noetic

    https://github.com/Ming-Start/robot_tutorials/blob/feature_build/ros1/gazebo/README_build_ros1.md

## Running the simulation demo

1. Once the package has been built and the workspace sourced, run the simulation:

    ```bash
    roslaunch fg_gazebo_example simulation.launch

## Moving the robot to any position in the defined moveit workspace 

1. Run the Python script in the simulation demo package to execute robot motions using MoveIt's Python API:

    ```bash 
    rosrun fg_gazebo_example move_viewpoints.py

2. Run the Python script in the simulation demo package to execute robot motions to any position in the moveit workspace using MoveIt's Python API:

    ```bash
    rosrun fg_gazebo_example target_reach.py 0.4 0.1 0.4
