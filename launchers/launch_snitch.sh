#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash
cd ~/catkin_ws/src/robots_ex_3/launchers
sudo ./setup_robot.sh

roslaunch robots_exercise_3 robot.launch visualise:=$1 rviz:=$2
