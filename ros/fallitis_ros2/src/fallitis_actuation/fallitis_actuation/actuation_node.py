from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .config import ActuationConfig


class ActuationNode(Node):
    def __init__(self) -> None:
        super().__init__("fallitis_actuation")
        self.config = ActuationConfig.from_node(self)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.lop_pub = self.create_publisher(Empty, "/robot/lack_of_progress", 10)
        self.status_pub = self.create_publisher(String, "/actuation/status", 10)

        self.create_subscription(Twist, "/teleop/cmd_vel_input", self.on_teleop_cmd, 10)
        self.create_subscription(Empty, "/teleop/lack_of_progress", self.on_lop_request, 10)

        self.current_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.current_sim_time_s = 0.0
        self.current_step_index = 0
        self.last_cmd_step = 0
        self.lop_count = 0
        self.last_lop_time_s = 0.0
        self.step_mode_enabled = False
        self.command_timeout_steps = 1
        self.status_every_steps = 1

        self.command_timer = self.create_timer(0.05, self.tick)
        self.status_timer = self.create_timer(self.config.status_period_s, self.publish_status)

    def on_teleop_cmd(self, msg: Twist) -> None:
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if self.config.mirror_reverse_steering and linear < 0.0 and angular != 0.0:
            angular = -angular
        self.current_cmd = Twist()
        self.current_cmd.linear.x = linear
        self.current_cmd.angular.z = angular
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_step = self.current_step_index
        self.cmd_pub.publish(self.current_cmd)

    def on_lop_request(self, _: Empty) -> None:
        self.lop_pub.publish(Empty())
        self.lop_count += 1
        if self.step_mode_enabled:
            self.last_lop_time_s = self.current_sim_time_s
        else:
            self.last_lop_time_s = self.get_clock().now().nanoseconds / 1e9

    def tick(self) -> None:
        if self.step_mode_enabled:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age <= self.config.command_timeout_s:
            return
        if self.current_cmd.linear.x == 0.0 and self.current_cmd.angular.z == 0.0:
            return
        self.current_cmd = Twist()
        self.cmd_pub.publish(self.current_cmd)

    def enable_step_mode(self, time_step_s: float) -> None:
        self.step_mode_enabled = True
        self.command_timeout_steps = max(1, round(self.config.command_timeout_s / max(time_step_s, 1e-6)))
        self.status_every_steps = max(1, round(self.config.status_period_s / max(time_step_s, 1e-6)))
        self.command_timer.cancel()
        self.status_timer.cancel()

    def step_update(self, *, sim_time_s: float, step_index: int) -> None:
        self.current_sim_time_s = float(sim_time_s)
        self.current_step_index = int(step_index)
        if self.step_mode_enabled:
            age_steps = self.current_step_index - self.last_cmd_step
            if age_steps > self.command_timeout_steps:
                if self.current_cmd.linear.x != 0.0 or self.current_cmd.angular.z != 0.0:
                    self.current_cmd = Twist()
                    self.cmd_pub.publish(self.current_cmd)
        if step_index % self.status_every_steps == 0:
            self.publish_status()

    def publish_status(self) -> None:
        payload = {
            "linear_x": float(self.current_cmd.linear.x),
            "angular_z": float(self.current_cmd.angular.z),
            "mirror_reverse_steering": self.config.mirror_reverse_steering,
            "lop_count": self.lop_count,
            "last_lop_time_s": self.last_lop_time_s,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActuationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
