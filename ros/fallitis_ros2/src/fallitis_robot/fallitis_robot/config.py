from __future__ import annotations

from dataclasses import dataclass

from fallitis_common.params import declare_required_parameters, require_bool, require_float, require_int, require_str
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True, slots=True)
class RobotConfig:
    time_step_ms: int
    max_motor_velocity: float
    wheel_radius: float
    axle_length: float
    command_timeout_s: float
    camera_publish_period_s: float
    scan_publish_period_s: float
    odom_publish_period_s: float
    wheel_state_publish_period_s: float
    default_control_mode: str
    object_recognition_enabled: bool
    object_recognition_api_url: str
    object_recognition_api_timeout_s: float
    object_recognition_api_jpeg_quality: int
    object_recognition_conf: float
    object_recognition_imgsz: int
    object_recognition_device: str
    object_recognition_period_s: float
    left_motor_name: str
    right_motor_name: str
    gps_name: str
    imu_name: str
    lidar_name: str
    left_camera_name: str
    right_camera_name: str
    emitter_name: str

    @classmethod
    def from_node(cls, node: Node) -> "RobotConfig":
        declare_required_parameters(
            node,
            [
                ("time_step_ms", Parameter.Type.INTEGER),
                ("max_motor_velocity", Parameter.Type.DOUBLE),
                ("wheel_radius", Parameter.Type.DOUBLE),
                ("axle_length", Parameter.Type.DOUBLE),
                ("command_timeout_s", Parameter.Type.DOUBLE),
                ("camera_publish_period_s", Parameter.Type.DOUBLE),
                ("scan_publish_period_s", Parameter.Type.DOUBLE),
                ("odom_publish_period_s", Parameter.Type.DOUBLE),
                ("wheel_state_publish_period_s", Parameter.Type.DOUBLE),
                ("default_control_mode", Parameter.Type.STRING),
                ("object_recognition_enabled", Parameter.Type.BOOL),
                ("object_recognition_api_url", Parameter.Type.STRING),
                ("object_recognition_api_timeout_s", Parameter.Type.DOUBLE),
                ("object_recognition_api_jpeg_quality", Parameter.Type.INTEGER),
                ("object_recognition_conf", Parameter.Type.DOUBLE),
                ("object_recognition_imgsz", Parameter.Type.INTEGER),
                ("object_recognition_device", Parameter.Type.STRING),
                ("object_recognition_period_s", Parameter.Type.DOUBLE),
                ("left_motor_name", Parameter.Type.STRING),
                ("right_motor_name", Parameter.Type.STRING),
                ("gps_name", Parameter.Type.STRING),
                ("imu_name", Parameter.Type.STRING),
                ("lidar_name", Parameter.Type.STRING),
                ("left_camera_name", Parameter.Type.STRING),
                ("right_camera_name", Parameter.Type.STRING),
                ("emitter_name", Parameter.Type.STRING),
            ],
        )
        return cls(
            time_step_ms=require_int(node, "time_step_ms"),
            max_motor_velocity=require_float(node, "max_motor_velocity"),
            wheel_radius=require_float(node, "wheel_radius"),
            axle_length=require_float(node, "axle_length"),
            command_timeout_s=require_float(node, "command_timeout_s"),
            camera_publish_period_s=require_float(node, "camera_publish_period_s"),
            scan_publish_period_s=require_float(node, "scan_publish_period_s"),
            odom_publish_period_s=require_float(node, "odom_publish_period_s"),
            wheel_state_publish_period_s=require_float(node, "wheel_state_publish_period_s"),
            default_control_mode=require_str(node, "default_control_mode"),
            object_recognition_enabled=require_bool(node, "object_recognition_enabled"),
            object_recognition_api_url=require_str(node, "object_recognition_api_url"),
            object_recognition_api_timeout_s=require_float(node, "object_recognition_api_timeout_s"),
            object_recognition_api_jpeg_quality=require_int(node, "object_recognition_api_jpeg_quality"),
            object_recognition_conf=require_float(node, "object_recognition_conf"),
            object_recognition_imgsz=require_int(node, "object_recognition_imgsz"),
            object_recognition_device=require_str(node, "object_recognition_device"),
            object_recognition_period_s=require_float(node, "object_recognition_period_s"),
            left_motor_name=require_str(node, "left_motor_name"),
            right_motor_name=require_str(node, "right_motor_name"),
            gps_name=require_str(node, "gps_name"),
            imu_name=require_str(node, "imu_name"),
            lidar_name=require_str(node, "lidar_name"),
            left_camera_name=require_str(node, "left_camera_name"),
            right_camera_name=require_str(node, "right_camera_name"),
            emitter_name=require_str(node, "emitter_name"),
        )
