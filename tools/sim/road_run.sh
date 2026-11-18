#!/bin/bash
# usage: road_run.sh stanley|lqr [wall_seconds] [v_ref]
# Same start pose + same junction choice for both controllers.
T=$(cd "$(dirname "$0")" && pwd)
L=$T/logs; mkdir -p $L
source $T/env.sh
C=$1; DUR=${2:-45}; V=${3:-8}
case $C in
    stanley) NODE=/stanley_controller ;;
    lqr)     NODE=/roar_lqr_controller ;;
    *) echo "usage: $0 stanley|lqr [seconds] [v_ref]"; exit 1 ;;
esac

brake() { timeout 10 ros2 topic pub --once /carla/ego_vehicle/vehicle_control_cmd carla_msgs/msg/CarlaEgoVehicleControl "{throttle: 0.0, brake: 1.0, steer: 0.0}" >/dev/null; }

# Never leave a previous controller alive: it would also obey 'go' and share the log file
pkill -f "lib/carla_control/${C}_controller"

# Reset to the start pose (Town05 spawn 38, ROS frame) and let the car settle
brake
timeout 15 ros2 topic pub --once /carla/ego_vehicle/control/set_transform geometry_msgs/msg/Pose \
    "{position: {x: -188.1, y: -55.3, z: 0.8}, orientation: {z: 0.7071068, w: 0.7071068}}" >/dev/null
brake; brake

# Controller idles until 'go'
ros2 run carla_control ${C}_controller --ros-args -p reference_source:=road -p wait_for_command:=true \
    > $L/road_$C.log 2>&1 &
CTRL=$!
sleep 4

timeout 10 ros2 topic pub --once $NODE/command std_msgs/msg/String "{data: 'go v=$V'}" >/dev/null
echo "GO sent to $NODE"

status="time limit"
for i in $(seq $DUR); do
    sleep 1
    pgrep -f CarlaUE4-Linux-Shipping >/dev/null || { status="CARLA DIED at ${i}s"; break; }
done

timeout 10 ros2 topic pub --once $NODE/command std_msgs/msg/String "{data: 'stop'}" >/dev/null
brake
kill $CTRL 2>/dev/null; pkill -f "lib/carla_control/${C}_controller"; wait $CTRL 2>/dev/null
echo "$C finished: $status"
grep -aE "GO:|initialized|STOP|Bad|Error|Traceback" $L/road_$C.log | head -5
