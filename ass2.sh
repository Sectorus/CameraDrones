#!/bin/bash

roslaunch dla2_path_planner octomap_mapping_a2.launch &
rosrun dla2_path_planner dla2_path_planner_ros_node --runtime 0.5 --planner RRTStar -o WeightedLengthAndClearanceCombo -f planner_trajectory.txt --info 2 &
rosrun dla2_path_planner dla2_path_planner_trajectory_visualization &
rviz &
rostopic pub /path_planner/current_position geometry_msgs/Point "x: 0.0 y: 0.0 z: 0.3" --once &
rostopic pub /path_planner/goal_position geometry_msgs/Point "x: 10.0 y: -27.0 z: 15.0" --once &


