#!/usr/bin/env python3
"""
Rolling road-following reference for the Stanley / LQR controllers.

Reads the ego pose, walks the CARLA map's lane waypoints ahead of the car
and publishes them as a nav_msgs/Path (ROS map frame) a few times per second.
The controllers track this live path instead of a fixed synthetic trajectory,
so the steering follows whatever road the car is actually on.

Junction behaviour is set by the 'junction_choice' parameter
(straight | left | right | random) and can be changed while running:

    ros2 param set /road_reference junction_choice left
"""

import math
import random

import carla
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaWorldInfo
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class RoadReference(Node):

    def __init__(self):
        super().__init__('road_reference')

        # ======================================================
        # ROS PARAMETERS
        # ======================================================
        self.declare_parameter("horizon", 40.0)          # m ahead of the car
        self.declare_parameter("lookback", 5.0)          # m behind (keeps nearest-point search stable)
        self.declare_parameter("waypoint_spacing", 0.5)
        self.declare_parameter("rate", 5.0)              # Hz
        self.declare_parameter("junction_choice", "straight")

        # The map is built client-side from the bridge's latched OpenDRIVE,
        # so this node never opens its own connection to the simulator.
        # (A second carla.Client whose world stream goes unread in sync mode
        # eventually gets dropped, and CARLA 0.9.14 segfaults on that.)
        self.map = None

        # Junction decisions are remembered per junction so the
        # rolling re-plan doesn't flip between branches mid-turn
        self.junction_branch = {}
        self.ego_pose = None
        self.last_ahead = []

        # ======================================================
        # ROS INTERFACES
        # ======================================================
        self.path_pub = self.create_publisher(Path, "/carla/ego_vehicle/road_reference", 10)

        self.create_subscription(
            Odometry,
            "/carla/ego_vehicle/odometry",
            self.odom_cb,
            10
        )

        self.create_subscription(
            CarlaWorldInfo,
            "/carla/world_info",
            self.world_info_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(1.0 / self.get_parameter("rate").value, self.publish_path)

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "junction_choice":
                if p.value not in ("straight", "left", "right", "random"):
                    return SetParametersResult(successful=False,
                                               reason="use straight|left|right|random")
                # New choice applies to junctions not yet entered
                self.junction_branch.clear()
                self.get_logger().info(f"junction_choice -> {p.value}")
        return SetParametersResult(successful=True)

    def world_info_cb(self, msg):
        if not msg.opendrive:
            return
        if self.map is None or self.map.name != msg.map_name:
            self.map = carla.Map(msg.map_name, msg.opendrive)
            self.junction_branch.clear()
            self.last_ahead = []
            self.get_logger().info(f"Loaded CARLA map {msg.map_name}")

    # ======================================================
    # ODOM CALLBACK (ROS frame)
    # ======================================================
    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.ego_pose = (pos.x, pos.y, pos.z, yaw)

    # ======================================================
    # BRANCH SELECTION
    # ======================================================
    def pick_branch(self, wp, candidates):
        if len(candidates) == 1:
            return candidates[0]

        junction = candidates[0].get_junction() if candidates[0].is_junction else None
        key = (junction.id if junction else None, wp.road_id, wp.lane_id)
        if key in self.junction_branch:
            for c in candidates:
                if (c.road_id, c.lane_id) == self.junction_branch[key]:
                    return c

        choice = self.get_parameter("junction_choice").value
        y0 = wp.transform.rotation.yaw

        # Heading change over each branch; CARLA yaw is clockwise,
        # so a right turn is a positive change
        def turn(c):
            far = c.next(15.0)
            end = far[0] if far else c
            return wrap_deg(end.transform.rotation.yaw - y0)

        turns = [turn(c) for c in candidates]
        if choice == "left":
            idx = min(range(len(candidates)), key=lambda i: turns[i])
        elif choice == "right":
            idx = max(range(len(candidates)), key=lambda i: turns[i])
        elif choice == "random":
            idx = random.randrange(len(candidates))
        else:
            idx = min(range(len(candidates)), key=lambda i: abs(turns[i]))

        self.junction_branch[key] = (candidates[idx].road_id, candidates[idx].lane_id)
        return candidates[idx]

    # ======================================================
    # PATH GENERATION
    # ======================================================
    def publish_path(self):
        if self.ego_pose is None or self.map is None:
            return

        x, y, z, _ = self.ego_pose
        # ROS (right-handed) -> CARLA (left-handed): flip y
        ego_loc = carla.Location(x=x, y=-y, z=z)

        # Junction lanes overlap, so snapping to the nearest lane can jump
        # onto a crossing branch. Continue along the last path while on it.
        wp = None
        if self.last_ahead:
            near = min(self.last_ahead, key=lambda w: w.transform.location.distance(ego_loc))
            if near.transform.location.distance(ego_loc) < 2.0:
                wp = near
        if wp is None:
            wp = self.map.get_waypoint(
                ego_loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
        if wp is None:
            return

        ds = self.get_parameter("waypoint_spacing").value
        horizon = self.get_parameter("horizon").value
        lookback = self.get_parameter("lookback").value

        behind = []
        w = wp
        for _ in range(int(lookback / ds)):
            prev = w.previous(ds)
            if not prev:
                break
            w = prev[0]
            behind.append(w)

        ahead = [wp]
        w = wp
        for _ in range(int(horizon / ds)):
            nxt = w.next(ds)
            if not nxt:
                break
            w = self.pick_branch(w, nxt)
            ahead.append(w)
        self.last_ahead = ahead

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        for w in list(reversed(behind)) + ahead:
            t = w.transform
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = t.location.x
            ps.pose.position.y = -t.location.y
            ps.pose.position.z = t.location.z
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, -math.radians(t.rotation.yaw))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)

        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = RoadReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
