#!/bin/bash
# Activate the ROS environment and then launch pycharm

PYCHARM_DIR=pycharm-community-2018
source /home/robond/catkin_ws/devel/setup.bash && /opt/${PYCHARM_DIR}/bin/pycharm.sh &