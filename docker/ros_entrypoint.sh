#!/bin/bash
set -e

# setup ros2 environment
source "/opt/workspace/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "$@"

