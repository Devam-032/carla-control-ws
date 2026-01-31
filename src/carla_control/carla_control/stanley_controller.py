#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
import time

from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_control_interfaces.srv import GenerateTrajectory
from tf_transformations import euler_from_quaternion


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class StanleyController(Node):

    def __init__(self):
        super().__init__('stanley_controller')

        # ======================================================
        # ROS PARAMETERS
        # ======================================================
        self.declare_parameter("trajectory_type", "straight")
        self.declare_parameter("waypoint_spacing", 0.2)
        self.declare_parameter("total_length", 100.0)
        self.declare_parameter("turn_radius", 5.0)
        self.declare_parameter("v_ref", 10.0)

        self.trajectory_type = self.get_parameter("trajectory_type").value
        self.waypoint_spacing = self.get_parameter("waypoint_spacing").value
        self.total_length = self.get_parameter("total_length").value
        self.turn_radius = self.get_parameter("turn_radius").value
        self.v_ref = self.get_parameter("v_ref").value

        # ======================================================
        # STANLEY GAINS
        # ======================================================
        self.k_ct = 1.8
        self.k_heading = 2.0
        self.k_heading_damp = 0.0
        self.v_min = 0.5

        self.dt = 0.02

        self.prev_steer = 0.0
        self.steer_alpha = 1.0

        self.kp_speed = 0.5

        # ======================================================
        # STATE
        # ======================================================
        self.ego_pose = None
        self.ego_speed = 0.0

        self.traj_local = None
        self.traj_world = None

        self.traj_received = False
        self.initialized = False

        self.goal_reached = False
        self.goal_tol = 0.5

        self.request_shutdown = False
        self.control_timer = None

        # ======================================================
        # LOGGING
        # ======================================================
        log_dir = os.path.expanduser("~/stanley_logs")
        os.makedirs(log_dir, exist_ok=True)

        self.ego_log = open(os.path.join(log_dir, "ego_path.csv"), "w", newline="")
        self.ego_writer = csv.writer(self.ego_log)

        # CSV HEADER (self-describing)
        self.ego_writer.writerow([
            "time",
            "x",
            "y",
            "yaw",
            "ego_speed",
            "cte",
            "heading_error",
            "steer",
            "k_ct",
            "k_heading"
        ])

        self.ref_log = open(os.path.join(log_dir, "reference_trajectory.csv"), "w", newline="")
        self.ref_writer = csv.writer(self.ref_log)
        self.ref_writer.writerow(["x", "y", "yaw"])

        # ======================================================
        # ROS INTERFACES
        # ======================================================
        self.ctrl_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            "/carla/ego_vehicle/vehicle_control_cmd",
            10
        )

        self.create_subscription(
            Odometry,
            "/carla/ego_vehicle/odometry",
            self.odom_cb,
            10
        )

        self.traj_client = self.create_client(
            GenerateTrajectory,
            "/generate_trajectory"
        )

        self.traj_timer = self.create_timer(0.5, self.request_trajectory)

        self.get_logger().info(
            f"Stanley controller started | k_ct={self.k_ct}, k_heading={self.k_heading}"
        )

    # ======================================================
    # TRAJECTORY REQUEST
    # ======================================================
    def request_trajectory(self):

        if self.traj_received:
            return

        if not self.traj_client.service_is_ready():
            return

        req = GenerateTrajectory.Request()
        req.trajectory_type = self.trajectory_type
        req.waypoint_spacing = self.waypoint_spacing
        req.total_length = self.total_length
        req.turn_radius = self.turn_radius

        future = self.traj_client.call_async(req)
        future.add_done_callback(self.on_traj_response)

    def on_traj_response(self, future):

        resp = future.result()
        if resp is None or len(resp.x) == 0:
            return

        self.traj_local = np.column_stack((resp.x, resp.y, resp.yaw))
        self.traj_received = True

        self.get_logger().info(f"Trajectory received ({len(self.traj_local)} points)")

    # ======================================================
    # BUILD WORLD TRAJECTORY
    # ======================================================
    def build_world_trajectory(self):

        x0, y0, yaw0 = self.ego_pose

        R = np.array([
            [np.cos(yaw0), -np.sin(yaw0)],
            [np.sin(yaw0),  np.cos(yaw0)]
        ])

        xy_world = (R @ self.traj_local[:, :2].T).T
        xy_world[:, 0] += x0
        xy_world[:, 1] += y0

        yaw_world = wrap_angle(self.traj_local[:, 2] + yaw0)
        self.traj_world = np.column_stack((xy_world, yaw_world))

        for p in self.traj_world:
            self.ref_writer.writerow(p)

        self.get_logger().info("Trajectory transformed to WORLD frame")

    # ======================================================
    # ODOM CALLBACK
    # ======================================================
    def odom_cb(self, msg):

        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.ego_pose = (pos.x, pos.y, yaw)
        self.ego_speed = np.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

        if self.traj_received and not self.initialized:
            self.build_world_trajectory()
            self.control_timer = self.create_timer(self.dt, self.control_loop)
            self.initialized = True
            self.get_logger().info("Controller initialized")

    # ======================================================
    # CONTROL LOOP
    # ======================================================
    def control_loop(self):

        if self.goal_reached or self.ego_pose is None:
            return

        x, y, yaw = self.ego_pose

        d = np.hypot(
            self.traj_world[:, 0] - x,
            self.traj_world[:, 1] - y
        )
        idx = np.argmin(d)

        gx, gy, _ = self.traj_world[-1]
        if idx >= len(self.traj_world) - 2 and np.hypot(gx - x, gy - y) < self.goal_tol:
            self.goal_reached = True

            cmd = CarlaEgoVehicleControl()
            cmd.throttle = 0.0
            cmd.brake = 1.0
            cmd.steer = 0.0
            self.ctrl_pub.publish(cmd)

            self.control_timer.cancel()
            self.request_shutdown = True
            return

        xr, yr, yawr = self.traj_world[idx]

        theta_e = wrap_angle(yawr - yaw)
        theta_e /= (1.0 + self.k_heading_damp * self.ego_speed)

        n = np.array([-np.sin(yawr), np.cos(yawr)])
        e_ct = n @ np.array([x - xr, y - yr])

        v_eff = max(self.ego_speed, self.v_min)

        steer_cmd = (
            -self.k_heading * theta_e +
            np.arctan(self.k_ct * e_ct / v_eff)
        )
        steer_cmd = np.clip(steer_cmd, -1.0, 1.0)

        steer = (
            self.steer_alpha * steer_cmd +
            (1.0 - self.steer_alpha) * self.prev_steer
        )
        self.prev_steer = steer

        accel = self.kp_speed * (self.v_ref - self.ego_speed)

        cmd = CarlaEgoVehicleControl()
        cmd.steer = float(steer)
        cmd.throttle = float(np.clip(accel, 0.0, 1.0))
        cmd.brake = 0.0

        self.ctrl_pub.publish(cmd)

        # CSV LOG
        self.ego_writer.writerow([
            time.time(),
            x, y, yaw,
            self.ego_speed,
            e_ct,
            theta_e,
            steer,
            self.k_ct,
            self.k_heading
        ])


def main():
    rclpy.init()
    node = StanleyController()

    try:
        while rclpy.ok() and not node.request_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
