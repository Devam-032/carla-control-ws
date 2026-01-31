#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from carla_control_interfaces.srv import GenerateTrajectory


class TrajectoryService(Node):

    def __init__(self):
        super().__init__('traj_service')

        self.srv = self.create_service(
            GenerateTrajectory,
            '/generate_trajectory',
            self.handle_request
        )

        self.get_logger().info("Trajectory generation service ready")

    # ==========================================================
    # SERVICE CALLBACK
    # ==========================================================

    def handle_request(self, request, response):

        ds = request.waypoint_spacing
        total_length = request.total_length
        R = request.turn_radius
        traj_type = request.trajectory_type.lower()

        # ----------------------------
        # Validation
        # ----------------------------
        if ds <= 0.0 or total_length <= 0.0:
            self.get_logger().error("Invalid trajectory parameters")
            return response

        # ----------------------------
        # Generate trajectory
        # ----------------------------
        if traj_type == "straight":
            x, y, yaw = self.straight_trajectory(ds, total_length)

        elif traj_type == "right_turn":
            x, y, yaw = self.right_turn_trajectory(ds, total_length, R)

        elif traj_type == "s_curve":
            x, y, yaw = self.s_curve_trajectory(ds, total_length, R)

        else:
            self.get_logger().warn(
                f"Unknown trajectory_type '{traj_type}', using straight"
            )
            x, y, yaw = self.straight_trajectory(ds, total_length)

        # ----------------------------
        # Feedforward curvature
        # κ = d(yaw) / ds
        # ----------------------------
        kappa = self.compute_curvature(yaw, ds)

        # ----------------------------
        # Populate response
        # ----------------------------
        response.x = x.tolist()
        response.y = y.tolist()
        response.yaw = yaw.tolist()
        response.kappa = kappa.tolist()

        self.get_logger().info(
            f"Generated '{traj_type}' trajectory | "
            f"{len(x)} points | κ ∈ [{np.min(kappa):.3f}, {np.max(kappa):.3f}]"
        )

        return response

    # ==========================================================
    # CURVATURE COMPUTATION
    # ==========================================================

    def compute_curvature(self, yaw, ds):
        """
        Discrete curvature computation:
        κ = d(yaw)/ds
        """
        yaw = np.unwrap(yaw)          # critical for continuity
        kappa = np.gradient(yaw, ds)
        return kappa

    # ==========================================================
    # TRAJECTORY DEFINITIONS
    # ==========================================================

    def straight_trajectory(self, ds, L):
        s = np.arange(0.0, L, ds)
        x = s
        y = np.zeros_like(s)
        yaw = np.zeros_like(s)
        return x, y, yaw

    def right_turn_trajectory(self, ds, L, R):
        turn_angle = np.pi / 2
        L_turn = R * turn_angle

        if L <= L_turn:
            L = L_turn + 1.0

        L_straight = (L - L_turn) / 2

        # ---- Straight in ----
        s1 = np.arange(0.0, L_straight, ds)
        x1 = s1
        y1 = np.zeros_like(s1)
        yaw1 = np.zeros_like(s1)

        # ---- Circular arc ----
        theta = np.arange(0.0, turn_angle, ds / R)
        xc = L_straight
        yc = -R

        x2 = xc + R * np.sin(theta)
        y2 = yc + R * np.cos(theta)
        yaw2 = -theta

        # ---- Straight out ----
        s3 = np.arange(0.0, L_straight, ds)
        x3 = np.full_like(s3, x2[-1])
        y3 = y2[-1] - s3
        yaw3 = np.full_like(s3, -np.pi / 2)

        return (
            np.concatenate([x1, x2, x3]),
            np.concatenate([y1, y2, y3]),
            np.concatenate([yaw1, yaw2, yaw3]),
        )

    def s_curve_trajectory(self, ds, L, R):
        s = np.arange(0.0, L, ds)
        x = s
        y = R * np.sin(2.0 * np.pi * s / L)

        dy_ds = np.gradient(y, ds)
        yaw = np.arctan2(dy_ds, np.ones_like(dy_ds))

        return x, y, yaw


# ==========================================================
# MAIN
# ==========================================================

def main():
    rclpy.init()
    node = TrajectoryService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
