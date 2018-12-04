#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash
cd ~
sudo ./setup_robot.sh

roslaunch robots_exercise_3 movement.launch visualise:=$1 rviz:=$2 
