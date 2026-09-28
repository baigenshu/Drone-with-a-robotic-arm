#!/usr/bin/env bash
# Connect MAVROS to the running PX4 SITL (offboard link 14580 <-> local 14540).
# Run this in terminal 2.
set -e
source /opt/ros/noetic/setup.bash
exec roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580
