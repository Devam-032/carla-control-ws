#!/bin/bash
# Start the sim stack one stage at a time, verifying each stage before moving on.
#
# Stability notes (CARLA 0.9.14 on this machine):
#  - The chase camera streamed to carla_manual_control made the server segfault
#    within ~1-2 min, so the ego has no cameras and manual control is not launched.
#  - Town05 is loaded before the bridge starts so the bridge never reloads the map.
#  - Lockstep: the sim advances one tick per control command.
T=$(cd "$(dirname "$0")" && pwd)
L=$T/logs; mkdir -p $L

cd ~/carla && setsid nohup ./CarlaUE4.sh -prefernvidia -quality-level=Low -nosound > $L/carla.log 2>&1 &
up=0
for i in $(seq 40); do
    python3 -c "import carla;c=carla.Client('localhost',2000);c.set_timeout(3);c.get_server_version()" 2>/dev/null && { up=1; break; }
    sleep 2
done
[ $up = 1 ] || { echo "CARLA FAILED to start"; tail -5 $L/carla.log; exit 1; }
echo "1/4 CARLA up"
python3 -c "import carla;c=carla.Client('localhost',2000);c.set_timeout(60);c.load_world('Town05');print('   map:',c.get_world().get_map().name)" || { echo "MAP LOAD FAILED"; exit 1; }
sleep 3

setsid nohup bash -c "
source $T/env.sh
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:=Carla/Maps/Town05 \
    fixed_delta_seconds:=0.03333 synchronous_mode:=True \
    synchronous_mode_wait_for_vehicle_control_command:=True timeout:=30 &
sleep 8
ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py \
    objects_definition_file:=$T/objects_light.json &
wait" > $L/bridge.log 2>&1 &
for i in $(seq 40); do grep -q "All objects spawned" $L/bridge.log && break; sleep 2; done
grep -q "All objects spawned" $L/bridge.log || { echo "BRIDGE FAILED"; tail -5 $L/bridge.log; exit 1; }
echo "2/4 bridge up, ego spawned"

setsid nohup bash -c "source $T/env.sh; exec ros2 run carla_control trajectory_service" > $L/traj.log 2>&1 &
sleep 4
echo "3/4 trajectory service started"

setsid nohup bash -c "source $T/env.sh; exec ros2 run carla_control road_reference" > $L/road_ref.log 2>&1 &
for i in $(seq 20); do grep -q "Loaded CARLA map" $L/road_ref.log && break; sleep 1; done
grep -q "Loaded CARLA map" $L/road_ref.log || { echo "ROAD_REFERENCE FAILED"; tail -5 $L/road_ref.log; exit 1; }
echo "4/4 road_reference up"
