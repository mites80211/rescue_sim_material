from __future__ import annotations

import rclpy
from rclpy.executors import ShutdownException, SingleThreadedExecutor, TimeoutException
from rclpy.node import Node

from fallitis_actuation.actuation_node import ActuationNode
from fallitis_mapping.mapping_node import MappingNode
from fallitis_perception.perception_node import PerceptionNode
from fallitis_robot.webots_robot_node import WebotsRobotNode

from .config import SyncControllerConfig


class SyncController(Node):
    def __init__(self) -> None:
        super().__init__("fallitis_sync_controller")
        self.config = SyncControllerConfig.from_node(self)
        self._executor = SingleThreadedExecutor()
        self.robot = WebotsRobotNode()
        self.actuation = ActuationNode()
        self.perception = PerceptionNode()
        self.mapping = MappingNode()
        self.nodes = [
            self.robot,
            self.actuation,
            self.perception,
            self.mapping,
        ]
        for node in self.nodes:
            self._executor.add_node(node)
        self.actuation.enable_step_mode(self.robot.time_step_s)
        self.perception.enable_step_mode(self.robot.time_step_s)
        self.mapping.enable_step_mode(self.robot.time_step_s)
        self.get_logger().info("Step-synchronous controller initialised.")

    def drain_ready_callbacks(self) -> bool:
        processed = 0
        while rclpy.ok():
            try:
                handler, _, _ = self._executor.wait_for_ready_callbacks(timeout_sec=0.0)
            except TimeoutException:
                return True
            except ShutdownException:
                return False
            handler()
            if handler.exception() is not None:
                raise handler.exception()
            handler.result()
            processed += 1
            if processed >= self.config.max_ready_callbacks_per_drain:
                self.get_logger().warning(
                    "Reached callback drain limit; advancing the simulator to avoid callback starvation."
                )
                return True
        return False

    def run(self) -> None:
        try:
            while rclpy.ok():
                if not self.drain_ready_callbacks():
                    break
                self.actuation.step_update(
                    sim_time_s=self.robot.current_sim_time_s(),
                    step_index=self.robot.step_index,
                )
                if not self.drain_ready_callbacks():
                    break
                self.robot.stop_if_stale()
                self.robot.apply_drive_command()
                if not self.robot.step_simulation():
                    break
                self.robot.publish_step_data()
                if not self.drain_ready_callbacks():
                    break
                self.mapping.step_update(self.robot.step_index)
                self.perception.step_update(self.robot.step_index)
        finally:
            self.robot.commanded_linear = 0.0
            self.robot.commanded_angular = 0.0
            self.robot.apply_drive_command()
            for node in reversed(self.nodes):
                self._executor.remove_node(node)
                node.destroy_node()
            self.destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    controller = SyncController()
    try:
        controller.run()
    finally:
        rclpy.shutdown()
