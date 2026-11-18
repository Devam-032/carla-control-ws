#!/bin/bash
# Kill every sim-related process. Run as a file (bash kill_stack.sh) so pkill never
# matches the caller's own command line.
for p in CarlaUE4 carla_ros_bridge carla_manual_control carla_spawn_objects set_initial_pose \
         trajectory_service road_reference stanley_controller lqr_controller x11grab; do
    pkill -9 -f "$p"
done
sleep 2
ps -eo pid,args | grep -E "CarlaUE4|carla_ros_bridge|manual_control|spawn_objects|trajectory_service|road_reference|_controller" | grep -v grep || echo "all clear"
