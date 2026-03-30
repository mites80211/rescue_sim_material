from __future__ import annotations

from dataclasses import dataclass

from fallitis_common.params import declare_required_parameters, require_int
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True, slots=True)
class SyncControllerConfig:
    max_ready_callbacks_per_drain: int

    @classmethod
    def from_node(cls, node: Node) -> "SyncControllerConfig":
        declare_required_parameters(
            node,
            [
                ("max_ready_callbacks_per_drain", Parameter.Type.INTEGER),
            ],
        )
        return cls(
            max_ready_callbacks_per_drain=require_int(node, "max_ready_callbacks_per_drain"),
        )
