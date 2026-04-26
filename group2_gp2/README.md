# Course: ENPM605 
### Section: 0101
### Professor: Zeid Kootbally
### Assignment: GP 2
### Date: 04/26/2026


## Group Members:
Member 1: Nam Facchetti

UID: 118215693

Member 2: Yossaphat Kulvatunyou

UID: 112362550

## Contributions:
Nam: Nam: Implemented the NavigateToGoalClient action client node, including loading and validating goal parameters from the YAML configuration file, handling asynchronous goal execution, feedback processing, and result callbacks. Also implemented sequential goal dispatch logic to ensure each goal is completed before sending the next, along with mission summary reporting. Also developed the gp2.launch.py file to integrate the action server and client, configure launch arguments, and load parameters properly.

Yossaphat: Implemented the NavigateToGoalServer action server node, including the two-phase proportional controller ported from the reference P-controller demo. Also helped with setting up the group2_gp2_interfaces package including the NavigateToGoal.action definition, CMakeLists.txt, and package.xml configuration. Captured the cancel_demo pngs and webm for verifying the demonstration portion of this project.

## Project Summary:
This project implements a ROS 2-based autonomous navigation system for a differential-drive robot using an action-based architecture. A custom NavigateToGoal action interface is defined to support goal submission, feedback, and result reporting. The system consists of an action server that executes a two-phase proportional controller to drive the robot to a desired position and orientation, and an action client that sequentially sends three navigation goals loaded from a YAML configuration file. The system is integrated through a launch file that configures parameters and starts both nodes, enabling the robot to complete a multi-goal navigation mission in simulation.

## Build/Run Instructions:

### Navigate to workspace and pull latest code
- cd ~/enpm605_ws
- git pull

### Install dependencies
- rosdep install --from-paths src --ignore-src -y --skip-keys "micro_ros_agent"

### Remove old build files
- cd ~/enpm605_ws
- rm -rf build/ install/ log/

### Build the workspace
- colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to gp2_meta

### Source the workspace
- source ~/enpm605_ws/install/setup.bash

## For Regular Launch
### Launch the simulation (Terminal A)
- ros2 launch rosbot_gazebo gp2_world.launch.py

### Launch the full system (Terminal B)
- ros2 launch group2_gp2 gp2.launch.py

## For Cancellation Demonstration
### Launch the simulation (Terminal A)
- ros2 launch rosbot_gazebo gp2_world.launch.py

### Run only the action server (Terminal B)
- ros2 run group2_gp2 navigate_to_goal_server

### Send a long goal (Terminal C)
- ros2 action send_goal /navigate_to_goal \
    group2_gp2_interfaces/action/NavigateToGoal \
    "{goal_position: {x: 8.0, y: 0.0, z: 0.0}, final_heading: 0.0}" \
    --feedback

## Known Issues:
To our knowledge at this moment, there are currently no none errors/issues.
