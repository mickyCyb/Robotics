#!/bin/bash

# Move the box

# we first need to run these commands

# source devel/setup.bash

# roslaunch arm_gazebo arm.launch

# rosrun arm_gazebo ik.py


rostopic pub -1 /updateJointAngles arm_gazebo/pos "{x: 1.0, y: 2.1, z: 0.5}" 

sleep 5s

rostopic pub -1 /updateJointAngles arm_gazebo/pos "{x: 1.0, y: 2.1, z: 0.1}" 

sleep 5s

rostopic pub -1 /updateGripper arm_gazebo/UpdateGripper "release: false"

sleep 2s

rostopic pub -1 /updateJointAngles arm_gazebo/pos "{x: 1.0, y: 2.1, z: 0.8}" 

sleep 2s

rostopic pub -1 /updateJointAngles arm_gazebo/pos "{x: 1.5, y: 1.7, z: 0.8}" 

sleep 2s

rostopic pub  -1 /updateJointAngles arm_gazebo/pos "{x: 1.5, y: 1.7, z: 0.3}" 

sleep 2s

rostopic pub -1 /updateGripper arm_gazebo/UpdateGripper "release: true"

sleep 2s

rostopic pub  -1 /updateJointAngles arm_gazebo/pos "{x: 1.5, y: 1.7, z: 0.6}" 

sleep 2s

rostopic pub  -1 /updateJointAngles arm_gazebo/pos "{x: 1.0, y: 2.1, z: 1.3}" 