#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash
cd ~
roslaunch robots_exercise_3 face.launch visualise:=$1
