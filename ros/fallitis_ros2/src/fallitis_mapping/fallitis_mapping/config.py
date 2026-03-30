from __future__ import annotations

from dataclasses import dataclass

from fallitis_common.params import declare_required_parameters, require_float, require_int
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True, slots=True)
class MappingConfig:
    map_width_cells: int
    map_height_cells: int
    map_spacing: int
    lidar_max_range_m: float
    map_yaw_offset_rad: float
    wheel_radius: float
    axle_length: float
    expected_dt_s: float
    gps_filter_window: int
    gps_filter_window_min: int
    gps_filter_window_max: int
    trail_length: int
    spawn_anchor_sample_count: int
    spawn_snap_step_m: float
    robot_reset_jump_threshold_m: float
    lidar_close_range_m: float
    lidar_stationary_range_eps_m: float
    lidar_downsample: int
    linear_stationary_threshold_m_s: float
    linear_motion_threshold_m_s: float
    angular_stationary_threshold_rad_s: float
    angular_motion_threshold_rad_s: float
    imu_yaw_rate_stationary_threshold_rad_s: float
    lidar_static_confidence_threshold: float
    predictive_index_threshold: float
    pose_history_size: int
    pose_sync_tolerance_s: float
    debug_publish_period_s: float

    @classmethod
    def from_node(cls, node: Node) -> "MappingConfig":
        declare_required_parameters(
            node,
            [
                ("map_width_cells", Parameter.Type.INTEGER),
                ("map_height_cells", Parameter.Type.INTEGER),
                ("map_spacing", Parameter.Type.INTEGER),
                ("lidar_max_range_m", Parameter.Type.DOUBLE),
                ("map_yaw_offset_rad", Parameter.Type.DOUBLE),
                ("wheel_radius", Parameter.Type.DOUBLE),
                ("axle_length", Parameter.Type.DOUBLE),
                ("expected_dt_s", Parameter.Type.DOUBLE),
                ("gps_filter_window", Parameter.Type.INTEGER),
                ("gps_filter_window_min", Parameter.Type.INTEGER),
                ("gps_filter_window_max", Parameter.Type.INTEGER),
                ("trail_length", Parameter.Type.INTEGER),
                ("spawn_anchor_sample_count", Parameter.Type.INTEGER),
                ("spawn_snap_step_m", Parameter.Type.DOUBLE),
                ("robot_reset_jump_threshold_m", Parameter.Type.DOUBLE),
                ("lidar_close_range_m", Parameter.Type.DOUBLE),
                ("lidar_stationary_range_eps_m", Parameter.Type.DOUBLE),
                ("lidar_downsample", Parameter.Type.INTEGER),
                ("linear_stationary_threshold_m_s", Parameter.Type.DOUBLE),
                ("linear_motion_threshold_m_s", Parameter.Type.DOUBLE),
                ("angular_stationary_threshold_rad_s", Parameter.Type.DOUBLE),
                ("angular_motion_threshold_rad_s", Parameter.Type.DOUBLE),
                ("imu_yaw_rate_stationary_threshold_rad_s", Parameter.Type.DOUBLE),
                ("lidar_static_confidence_threshold", Parameter.Type.DOUBLE),
                ("predictive_index_threshold", Parameter.Type.DOUBLE),
                ("pose_history_size", Parameter.Type.INTEGER),
                ("pose_sync_tolerance_s", Parameter.Type.DOUBLE),
                ("debug_publish_period_s", Parameter.Type.DOUBLE),
            ],
        )
        return cls(
            map_width_cells=require_int(node, "map_width_cells"),
            map_height_cells=require_int(node, "map_height_cells"),
            map_spacing=require_int(node, "map_spacing"),
            lidar_max_range_m=require_float(node, "lidar_max_range_m"),
            map_yaw_offset_rad=require_float(node, "map_yaw_offset_rad"),
            wheel_radius=require_float(node, "wheel_radius"),
            axle_length=require_float(node, "axle_length"),
            expected_dt_s=require_float(node, "expected_dt_s"),
            gps_filter_window=require_int(node, "gps_filter_window"),
            gps_filter_window_min=require_int(node, "gps_filter_window_min"),
            gps_filter_window_max=require_int(node, "gps_filter_window_max"),
            trail_length=require_int(node, "trail_length"),
            spawn_anchor_sample_count=require_int(node, "spawn_anchor_sample_count"),
            spawn_snap_step_m=require_float(node, "spawn_snap_step_m"),
            robot_reset_jump_threshold_m=require_float(node, "robot_reset_jump_threshold_m"),
            lidar_close_range_m=require_float(node, "lidar_close_range_m"),
            lidar_stationary_range_eps_m=require_float(node, "lidar_stationary_range_eps_m"),
            lidar_downsample=require_int(node, "lidar_downsample"),
            linear_stationary_threshold_m_s=require_float(node, "linear_stationary_threshold_m_s"),
            linear_motion_threshold_m_s=require_float(node, "linear_motion_threshold_m_s"),
            angular_stationary_threshold_rad_s=require_float(node, "angular_stationary_threshold_rad_s"),
            angular_motion_threshold_rad_s=require_float(node, "angular_motion_threshold_rad_s"),
            imu_yaw_rate_stationary_threshold_rad_s=require_float(node, "imu_yaw_rate_stationary_threshold_rad_s"),
            lidar_static_confidence_threshold=require_float(node, "lidar_static_confidence_threshold"),
            predictive_index_threshold=require_float(node, "predictive_index_threshold"),
            pose_history_size=require_int(node, "pose_history_size"),
            pose_sync_tolerance_s=require_float(node, "pose_sync_tolerance_s"),
            debug_publish_period_s=require_float(node, "debug_publish_period_s"),
        )
