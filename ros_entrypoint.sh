#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"
source "/catkin_ws/devel/setup.bash"

roslaunch ual_backend_mavros docker_ual_sim_server.launch