#!/bin/bash
# Stop the ROS side (bridge, spawner, services, controllers) but keep the CARLA server running.
for p in carla_ros_bridge carla_spawn_objects set_initial_pose trajectory_service road_reference \
         stanley_controller lqr_controller; do
    pkill -f "$p"
done
sleep 3
ps -eo pid,args | grep -E "carla_ros_bridge|spawn_objects|trajectory_service|road_reference|_controller" | grep -v grep || echo "ROS side stopped"
