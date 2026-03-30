from __future__ import annotations

from dataclasses import dataclass

from fallitis_common.params import declare_required_parameters, require_float
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True, slots=True)
class WebConfig:
    manual_linear_speed_m_s: float
    manual_angular_speed_rad_s: float

    @classmethod
    def from_node(cls, node: Node) -> "WebConfig":
        declare_required_parameters(
            node,
            [
                ("manual_linear_speed_m_s", Parameter.Type.DOUBLE),
                ("manual_angular_speed_rad_s", Parameter.Type.DOUBLE),
            ],
        )
        return cls(
            manual_linear_speed_m_s=require_float(node, "manual_linear_speed_m_s"),
            manual_angular_speed_rad_s=require_float(node, "manual_angular_speed_rad_s"),
        )
