#!/bin/bash
workspace=$(pwd)

source ~/.bashrc

gnome-terminal -t "master1" -x sudo bash -c "cd ${workspace}; source ./devel/setup.bash && roslaunch arm_control arx.launch; exec bash;"


