# RotalProject

bashrc
``` 
source /opt/ros/humble/setup.bash
source $HOME/ggory_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=:/opt/ros/humble/share/turtlebot3_gazebo/models:/home/home/colcon_ws/src/kimm_multi_floor_gazebo/worlds

export MY_ROBOT=mpo_700
export MAP_NAME=neo_workshop

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI=$HOME/test.xml
export ROS_LOCALHOST_ONLY=1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Launch
```
ros2 launch kjj_simulation simulation.launch.py

ros2 launch kjj_simulation kjj_navigation.launch.py

ros2 launch nav2_collision_monitor multi_collision_monitor_node.launch.py

ros2 launch kjj_nav2_bringup rviz_launch.py
```
