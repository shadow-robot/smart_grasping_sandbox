#!/bin/bash
set -e

# setup ros environment
source "/workspace/devel/setup.bash"
source "/usr/share/gazebo-7/setup.sh"

roslaunch smart_grasping_sandbox smart_grasping_sandbox.launch gui:=false &

sleep 60

cd ~/gzweb
GAZEBO_MODEL_PATH=/workspace/src:/workspace/src/universal_robot:${GAZEBO_MODEL_PATH} ./start_gzweb.sh

# wait for any keu
read -n 1 -s
