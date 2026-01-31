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
from control.matlab import dare


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ==================================================
# Local curvature from x–y geometry (RIGHT-HANDED)
# ==================================================
def local_curvature_xy(p_prev, p_curr, p_next):
    x1, y1 = p_prev
    x2, y2 = p_curr
    x3, y3 = p_next

    a = np.hypot(x2 - x1, y2 - y1)
    b = np.hypot(x3 - x2, y3 - y2)
    c = np.hypot(x3 - x1, y3 - y1)

    if a * b * c < 1e-6:
        return 0.0

    area2 = (
        (x2 - x1) * (y3 - y1) -
        (y2 - y1) * (x3 - x1)
    )

    return 2.0 * area2 / (a * b * c)


class RoarLQRController(Node):

    def __init__(self):
        super().__init__('roar_lqr_controller')

        # ==================================================
        # Parameters
        # ==================================================
        self.declare_parameter("trajectory_type", "straight")
        self.declare_parameter("waypoint_spacing", 0.2)
        self.declare_parameter("total_length", 100.0)
        self.declare_parameter("turn_radius", 10.0)
        self.declare_parameter("v_ref", 4.0)

        self.trajectory_type = self.get_parameter("trajectory_type").value
        self.waypoint_spacing = self.get_parameter("waypoint_spacing").value
        self.total_length = self.get_parameter("total_length").value
        self.turn_radius = self.get_parameter("turn_radius").value
        self.v_ref = self.get_parameter("v_ref").value

        # ==================================================
        # Vehicle parameters
        # ==================================================
        self.m  = 1845.0
        self.Iz = 1388.0
        self.lf = 1.657
        self.lr = 1.348
        self.Cf = 80000.0
        self.Cr = 80000.0

        # ==================================================
        # Timing
        # ==================================================
        self.dt = 0.03333

        # ==================================================
        # LQR cost
        # ==================================================
        self.Q = np.diag([15.0, 0.0, 280.0, 0])
        self.R = np.array([[1e2]])

        self.K = None
        self.last_speed = None

        # ==================================================
        # Low-speed gating
        # ==================================================
        self.v_min_lqr = 3.0
        self.lqr_active = False

        # ==================================================
        # Goal / shutdown logic
        # ==================================================
        self.goal_tol = 1.5
        self.control_active = True

        # ==================================================
        # State
        # ==================================================
        self.ego_pose = None
        self.vx = 0.0
        self.vy = 0.0
        self.r  = 0.0

        self.traj_local = None
        self.traj_world = None
        self.traj_received = False
        self.initialized = False

        # ==================================================
        # Logging
        # ==================================================
        log_dir = os.path.expanduser("~/roar_lqr_logs")
        os.makedirs(log_dir, exist_ok=True)

        self.log = open(os.path.join(log_dir, "ego_path.csv"), "w", newline="")
        self.writer = csv.writer(self.log)
        self.writer.writerow([
            "time", "x", "y", "ref_x", "ref_y",
            "ey", "epsi", "vy", "r", "steer",
            "delta_ff", "delta_fb"
        ])

        # ==================================================
        # ROS interfaces
        # ==================================================
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

        self.create_timer(0.5, self.request_trajectory)

        self.get_logger().info("ROAR Dynamic Bicycle LQR started")

    # ==================================================
    # Trajectory
    # ==================================================
    def request_trajectory(self):
        if self.traj_received or not self.traj_client.service_is_ready():
            return

        req = GenerateTrajectory.Request()
        req.trajectory_type = self.trajectory_type
        req.waypoint_spacing = self.waypoint_spacing
        req.total_length = self.total_length
        req.turn_radius = self.turn_radius

        future = self.traj_client.call_async(req)
        future.add_done_callback(self.on_traj)

    def on_traj(self, future):
        resp = future.result()
        if resp is None:
            return

        self.traj_local = np.column_stack((resp.x, resp.y, resp.yaw, resp.kappa))
        self.traj_received = True
        self.get_logger().info(f"Trajectory received ({len(self.traj_local)} points)")

    def build_world_traj(self):
        x0, y0, yaw0 = self.ego_pose
        R = np.array([
            [np.cos(yaw0), -np.sin(yaw0)],
            [np.sin(yaw0),  np.cos(yaw0)]
        ])
        xy = (R @ self.traj_local[:, :2].T).T
        xy[:, 0] += x0
        xy[:, 1] += y0
        yaw = wrap_angle(self.traj_local[:, 2] + yaw0)
        self.traj_world = np.column_stack((xy, yaw))

    # ==================================================
    # Odometry
    # ==================================================
    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.ego_pose = (pos.x, pos.y, yaw)
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.r  = msg.twist.twist.angular.z

        if self.traj_received and not self.initialized:
            self.build_world_traj()
            self.control_timer = self.create_timer(self.dt, self.control_loop)
            self.initialized = True

    # ==================================================
    # LQR gain
    # ==================================================
    def compute_lqr(self, vx):
        if vx < self.v_min_lqr:
            return

        if self.last_speed is not None and abs(vx - self.last_speed) < 0.5:
            return

        m, Iz, lf, lr, Cf, Cr = self.m, self.Iz, self.lf, self.lr, self.Cf, self.Cr

        A = np.array([
            [0, 1, vx, 0],
            [0, -(2*(Cf+Cr))/(m*vx), 0,
             -(vx + (2*(Cf*lf-Cr*lr))/(m*vx))],
            [0, 0, 0, 1],
            [0, -(2*(Cf*lf-Cr*lr))/(Iz*vx), 0,
             -(2*(Cf*lf**2+Cr*lr**2))/(Iz*vx)]
        ])

        B = np.array([
            [0],
            [2*Cf/m],
            [0],
            [2*Cf*lf/Iz]
        ])

        Ad = np.eye(4) + self.dt * A
        Bd = self.dt * B

        P, _, _ = dare(Ad, Bd, self.Q, self.R)
        self.K = np.linalg.inv(self.R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)

        self.last_speed = vx
        self.lqr_active = True

    # ==================================================
    # Control loop
    # ==================================================
    def control_loop(self):
        if self.ego_pose is None or not self.control_active:
            return

        self.compute_lqr(self.vx)

        x, y, yaw = self.ego_pose

        # Shutdown condition
        if self.check_goal_reached(x, y):
            cmd = CarlaEgoVehicleControl()
            cmd.steer = 0.0
            cmd.throttle = 0.0
            cmd.brake = 1.0
            self.ctrl_pub.publish(cmd)
            self.control_active = False
            return

        d = np.hypot(self.traj_world[:, 0] - x,
                     self.traj_world[:, 1] - y)
        i = np.argmin(d)
        xr, yr, yawr = self.traj_world[i]

        epsi = wrap_angle(yaw - yawr)
        ey = -np.sin(yawr)*(x-xr) + np.cos(yawr)*(y-yr)

        i_prev = max(i - 1, 0)
        i_next = min(i + 1, len(self.traj_world) - 1)

        # --------------------------------------------------
        # Curvature: geometry → CARLA (LEFT-HANDED)
        # --------------------------------------------------
        kappa_geom = local_curvature_xy(
            self.traj_world[i_prev, :2],
            self.traj_world[i,      :2],
            self.traj_world[i_next, :2]
        )

        kappa = -kappa_geom   # <<< CARLA correction

        if self.vx >= self.v_min_lqr:
            denom = 1.0 - (self.m * self.vx**2) / (2.0 * self.Cf * (self.lf + self.lr))
            delta_ff = (self.lf + self.lr) * kappa / denom
        else:
            delta_ff = 0.0

        if not self.lqr_active:
            delta_fb = 0.0
            steer = 0.0
        else:
            x_state = np.array([[ey],
                                [self.vy],
                                [epsi],
                                [self.r]])
            delta_fb = -float(self.K @ x_state)
            steer = (delta_ff*0.2 - delta_fb)
            steer = np.clip(steer, -1.0, 1.0)

        throttle = 0.6 if self.vx < 0.3 else np.clip(0.4 * (self.v_ref - self.vx), 0.0, 0.8)

        cmd = CarlaEgoVehicleControl()
        cmd.steer = steer
        cmd.throttle = throttle
        cmd.brake = 0.0
        self.ctrl_pub.publish(cmd)

        self.writer.writerow([
            time.time(), x, y, xr, yr,
            ey, epsi, self.vy, self.r,
            steer, delta_ff, delta_fb
        ])

    def check_goal_reached(self, x, y):
        xf, yf, yawf = self.traj_world[-1]
        dist = np.hypot(x - xf, y - yf)
        if dist > self.goal_tol:
            return False
        path_dir = np.array([np.cos(yawf), np.sin(yawf)])
        veh_vec  = np.array([x - xf, y - yf])
        return np.dot(veh_vec, path_dir) > 0.0


def main():
    rclpy.init()
    node = RoarLQRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
