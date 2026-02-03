#! /usr/bin/env bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/haptic_interface_ROS2/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.35/
source haptic_interface_ROS2/install/setup.bash
source kuka_lbr_control/install/setup.bash
source zed_ws/install/setup.bash
source clarius_ws/install/setup.bash