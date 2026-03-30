from __future__ import annotations

from dataclasses import dataclass

from fallitis_common.params import declare_required_parameters, require_bool, require_float, require_int, require_str
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True, slots=True)
class PerceptionConfig:
    annotation_period_s: float
    model_conf: float
    model_device: str
    model_imgsz: int
    yolo_api_enabled: bool
    yolo_api_url: str
    yolo_api_timeout_s: float
    yolo_api_jpeg_quality: int

    @classmethod
    def from_node(cls, node: Node) -> "PerceptionConfig":
        declare_required_parameters(
            node,
            [
                ("annotation_period_s", Parameter.Type.DOUBLE),
                ("model_conf", Parameter.Type.DOUBLE),
                ("model_device", Parameter.Type.STRING),
                ("model_imgsz", Parameter.Type.INTEGER),
                ("yolo_api_enabled", Parameter.Type.BOOL),
                ("yolo_api_url", Parameter.Type.STRING),
                ("yolo_api_timeout_s", Parameter.Type.DOUBLE),
                ("yolo_api_jpeg_quality", Parameter.Type.INTEGER),
            ],
        )
        return cls(
            annotation_period_s=require_float(node, "annotation_period_s"),
            model_conf=require_float(node, "model_conf"),
            model_device=require_str(node, "model_device"),
            model_imgsz=require_int(node, "model_imgsz"),
            yolo_api_enabled=require_bool(node, "yolo_api_enabled"),
            yolo_api_url=require_str(node, "yolo_api_url"),
            yolo_api_timeout_s=require_float(node, "yolo_api_timeout_s"),
            yolo_api_jpeg_quality=require_int(node, "yolo_api_jpeg_quality"),
        )
