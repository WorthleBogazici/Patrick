#!/bin/bash

RS="/opt/ros/$ROS_DISTRO/setup.bash"
WS="/patrick_ws/install/setup.bash"

source "$RS"

if [ -f "$WS" ]; then
  source "$WS"
fi

echo ""
echo "Sourced:"
echo "  * $RS"
echo "  * $WS"
echo ""

exec "$@"
