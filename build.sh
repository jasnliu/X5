#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/jazzy/setup.bash
colcon build --base-paths vendor/openarmx_description --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
xacro vendor/openarmx_description/urdf/robot/v10.urdf.xacro bimanual:=true ros2_control:=false > model/openarmx.urdf.tmp
/usr/bin/python3 -c 'import xml.etree.ElementTree as E; assert E.parse("model/openarmx.urdf.tmp").getroot().find("ros2_control") is None'
mv model/openarmx.urdf.tmp model/openarmx.urdf
/usr/bin/python3 -m unittest discover -s tests -v
