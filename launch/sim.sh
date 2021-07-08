#!/bin/bash
echo 0000 | sudo -S pkill -9 gzserver
echo 0000 | sudo -S pkill -9 gzclient
sleep 1
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/fly_curve/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/fly_curve/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib

echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_PLUGIN_PATH

roslaunch fly_curve gazebo_sim.launch pause:=false
#sleep 3
