#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Classic (GUI) with the Typhoon H480 aerial manipulator.
# Run this in terminal 1.
set -e
cd "$HOME/PX4-Autopilot"
exec make px4_sitl gazebo-classic_typhoon_h480
