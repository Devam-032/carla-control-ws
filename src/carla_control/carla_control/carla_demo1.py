#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl

import pygame
import time


class PygameTeleopCARLA(Node):
    def __init__(self):
        super().__init__('pygame_teleop_carla')

        self.pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            10
        )

        # Control parameters (MUST be floats)
        self.max_throttle = 0.6
        self.max_steer = 0.5

        # Persistent reverse state
        self.reverse = False

        # Init pygame
        pygame.init()
        pygame.display.set_caption("CARLA ROS2 Teleop")
        self.screen = pygame.display.set_mode((300, 200))

        self.get_logger().info(
            "Pygame CARLA Teleop started\n"
            "W : throttle\n"
            "S : brake\n"
            "A : steer left\n"
            "D : steer right\n"
            "Q : toggle reverse\n"
            "ESC / Window close : stop & quit"
        )

    def run(self):
        rate_hz = 100
        dt = 1.0 / rate_hz

        try:
            while rclpy.ok():
                pygame.event.pump()
                keys = pygame.key.get_pressed()

                msg = CarlaEgoVehicleControl()
                msg.reverse = self.reverse
                msg.hand_brake = False

                # Throttle / Brake
                if keys[pygame.K_w]:
                    msg.throttle = self.max_throttle
                    msg.brake = 0.0
                elif keys[pygame.K_s]:
                    msg.throttle = 0.0
                    msg.brake = 1.0
                else:
                    msg.throttle = 0.0
                    msg.brake = 0.0

                # Steering
                if keys[pygame.K_a]:
                    msg.steer = -self.max_steer
                elif keys[pygame.K_d]:
                    msg.steer = self.max_steer
                else:
                    msg.steer = 0.0

                # Handle discrete key events (toggle reverse, quit)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt

                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            raise KeyboardInterrupt

                        if event.key == pygame.K_q:
                            self.reverse = not self.reverse
                            self.get_logger().info(
                                f"Reverse {'ON' if self.reverse else 'OFF'}"
                            )

                self.pub.publish(msg)
                time.sleep(dt)

        finally:
            self.stop_vehicle()
            pygame.quit()

    def stop_vehicle(self):
        stop = CarlaEgoVehicleControl()
        stop.throttle = 0.0
        stop.steer = 0.0
        stop.brake = 1.0
        stop.reverse = False
        stop.hand_brake = False
        self.pub.publish(stop)
        self.get_logger().info("Vehicle stopped safely.")


def main():
    rclpy.init()
    node = PygameTeleopCARLA()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
