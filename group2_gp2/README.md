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
Nam: Implemented the NavigateToGoalClient action client node, including loading and validating goal parameters from the YAML configuration file, handling asynchronous goal execution, feedback processing, and result callbacks. Also implemented sequential goal dispatch logic to ensure each goal is completed before sending the next, along with mission summary reporting. Also developed the gp2.launch.py file to integrate the action server and client, configure launch arguments, and load parameters properly. Finally, created the README.md

Yossaphat: Implemented the NavigateToGoalServer action server node, including the two-phase proportional controller ported from the reference P-controller demo. Also helped with setting up the group2_gp2_interfaces package including the NavigateToGoal.action definition, CMakeLists.txt, and package.xml configuration. Captured the cancel_demo pngs and webm for verifying the demonstration portion of this project.
