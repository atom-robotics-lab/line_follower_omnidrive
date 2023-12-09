#! /bin/bash

source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/line_follower_omnidrive/models/:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/line_follower_omnidrive/models/:${CATKIN_ENV_HOOK_WORKSPACE}/../src/line_follower_omnidrive/worlds/:${GAZEBO_RESOURCE_PATH}"