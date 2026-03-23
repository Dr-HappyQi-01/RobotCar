#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_SETUP="${WORKSPACE_DIR}/devel/setup.bash"

source /opt/ros/melodic/setup.bash

if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "Cannot find workspace setup: $WORKSPACE_SETUP"
    exit 1
fi

source "$WORKSPACE_SETUP"

exec rosrun robot_monitor robot_monitor
