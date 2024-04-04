#!/bin/bash
workspace=$(pwd)

source ~/.bashrc

gnome-terminal -t "master1" -x  bash -c "cd ${workspace}/master1; catkin_make;"

gnome-terminal -t "follow1" -x  bash -c "cd ${workspace}/follow1; catkin_make; "

gnome-terminal -t "master2" -x  bash -c "cd ${workspace}/master2; catkin_make; "

gnome-terminal -t "follow2" -x  bash -c "cd ${workspace}/follow2; catkin_make; "

