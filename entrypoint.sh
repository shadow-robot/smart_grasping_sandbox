#!/bin/bash
set -e

# setup ros environment
source "/workspace/devel/setup.bash"
roslaunch smart_grasping_sandbox/smart_grasping_sandbox.launch
