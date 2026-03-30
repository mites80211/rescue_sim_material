from __future__ import annotations

from dataclasses import dataclass

from fallitis_common.params import declare_required_parameters, require_bool, require_float
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True, slots=True)
class ActuationConfig:
    command_timeout_s: float
    mirror_reverse_steering: bool
    status_period_s: float

    @classmethod
    def from_node(cls, node: Node) -> "ActuationConfig":
        declare_required_parameters(
            node,
            [
                ("command_timeout_s", Parameter.Type.DOUBLE),
                ("mirror_reverse_steering", Parameter.Type.BOOL),
                ("status_period_s", Parameter.Type.DOUBLE),
            ],
        )
        return cls(
            command_timeout_s=require_float(node, "command_timeout_s"),
            mirror_reverse_steering=require_bool(node, "mirror_reverse_steering"),
            status_period_s=require_float(node, "status_period_s"),
        )
