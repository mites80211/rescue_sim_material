from __future__ import annotations

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Empty

from .config import RobotConfig
from .devices import CameraDevice, EmitterDevice, GpsDevice, InertialUnitDevice, LidarDevice, MotorDevice, RobotDriver
from .geometry import clamp, quaternion_from_yaw


class WebotsRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("fallitis_webots_robot")
        self.config = RobotConfig.from_node(self)

        self.robot = RobotDriver(time_step=self.config.time_step_ms)
        self.left_motor = MotorDevice(self.robot, self.config.left_motor_name)
        self.right_motor = MotorDevice(self.robot, self.config.right_motor_name)
        self.gps = GpsDevice(self.robot, self.config.gps_name)
        self.imu = InertialUnitDevice(self.robot, self.config.imu_name)
        self.lidar = LidarDevice(self.robot, self.config.lidar_name)
        self.left_camera = CameraDevice(self.robot, self.config.left_camera_name)
        self.right_camera = CameraDevice(self.robot, self.config.right_camera_name)
        self.emitter = EmitterDevice(self.robot, self.config.emitter_name)

        self.raw_odom_pub = self.create_publisher(Odometry, "/robot/odometry_raw", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/robot/lidar/scan", 10)
        self.lidar_range_image_pub = self.create_publisher(Image, "/robot/lidar/range_image", 5)
        self.left_image_pub = self.create_publisher(Image, "/robot/camera/left/image_raw", 5)
        self.right_image_pub = self.create_publisher(Image, "/robot/camera/right/image_raw", 5)
        self.wheel_state_pub = self.create_publisher(JointState, "/robot/wheel_states", 10)
        self.imu_rpy_pub = self.create_publisher(Vector3Stamped, "/robot/imu/rpy", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Empty, "/robot/lack_of_progress", self.on_lack_of_progress, 10)

        self.time_step_s = self.config.time_step_ms / 1000.0
        self.step_index = 0
        self.command_timeout_steps = self.period_to_steps(self.config.command_timeout_s)
        self.odom_publish_every_steps = self.period_to_steps(self.config.odom_publish_period_s)
        self.scan_publish_every_steps = self.period_to_steps(self.config.scan_publish_period_s)
        self.camera_publish_every_steps = self.period_to_steps(self.config.camera_publish_period_s)
        self.wheel_state_publish_every_steps = self.period_to_steps(self.config.wheel_state_publish_period_s)
        self.commanded_linear = 0.0
        self.commanded_angular = 0.0
        self.last_cmd_step = 0

        self.get_logger().info("Webots robot bridge initialised.")

    def period_to_steps(self, period_s: float) -> int:
        return max(1, round(float(period_s) / max(self.time_step_s, 1e-6)))

    def current_sim_time_s(self) -> float:
        return self.step_index * self.time_step_s

    def current_stamp(self) -> TimeMsg:
        sim_time = self.current_sim_time_s()
        seconds = int(sim_time)
        nanoseconds = int(round((sim_time - seconds) * 1e9))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        return TimeMsg(sec=seconds, nanosec=nanoseconds)

    def should_publish(self, every_steps: int) -> bool:
        return self.step_index > 0 and (
            self.step_index == 1 or self.step_index % max(1, every_steps) == 0
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.commanded_linear = float(msg.linear.x)
        self.commanded_angular = float(msg.angular.z)
        self.last_cmd_step = self.step_index

    def on_lack_of_progress(self, _: Empty) -> None:
        self.emitter.send_letter("L")
        self.get_logger().info("LoP request sent.")

    def stop_if_stale(self) -> None:
        age_steps = self.step_index - self.last_cmd_step
        if age_steps > self.command_timeout_steps:
            self.commanded_linear = 0.0
            self.commanded_angular = 0.0

    def apply_drive_command(self) -> None:
        linear = self.commanded_linear
        angular = self.commanded_angular
        half_axle = self.config.axle_length * 0.5
        left = (linear - angular * half_axle) / self.config.wheel_radius
        right = (linear + angular * half_axle) / self.config.wheel_radius
        left = clamp(left, -self.config.max_motor_velocity, self.config.max_motor_velocity)
        right = clamp(right, -self.config.max_motor_velocity, self.config.max_motor_velocity)
        self.left_motor.set_velocity(left)
        self.right_motor.set_velocity(right)

    def current_pose(self) -> tuple[float, float, float, float, float]:
        position = self.gps.xyz()
        rpy = self.imu.rpy()
        x = float(position[0])
        y = -float(position[2])
        roll = float(rpy[0])
        pitch = float(rpy[1])
        yaw = -float(rpy[2])
        return x, y, roll, pitch, yaw

    def publish_odometry(self, stamp) -> None:
        if not self.should_publish(self.odom_publish_every_steps):
            return

        x, y, roll, pitch, yaw = self.current_pose()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        left_velocity = self.left_motor.get_velocity()
        right_velocity = self.right_motor.get_velocity()
        odom.twist.twist.linear.x = self.config.wheel_radius * (left_velocity + right_velocity) * 0.5
        odom.twist.twist.angular.z = self.config.wheel_radius * (right_velocity - left_velocity) / self.config.axle_length
        self.raw_odom_pub.publish(odom)

        imu_msg = Vector3Stamped()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "base_link"
        imu_msg.vector.x = roll
        imu_msg.vector.y = pitch
        imu_msg.vector.z = yaw
        self.imu_rpy_pub.publish(imu_msg)

    def publish_wheel_state(self, stamp) -> None:
        if not self.should_publish(self.wheel_state_publish_every_steps):
            return
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ["left_wheel", "right_wheel"]
        msg.position = []
        msg.velocity = [self.left_motor.get_velocity(), self.right_motor.get_velocity()]
        msg.effort = []
        self.wheel_state_pub.publish(msg)

    def publish_scan(self, stamp) -> None:
        if not self.should_publish(self.scan_publish_every_steps):
            return

        image = self.lidar.range_image()
        ranges = np.where(np.isfinite(image), image, np.inf)
        ranges = np.min(ranges, axis=0).astype(np.float32)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_link"
        scan.angle_min = -self.lidar.hor_fov * 0.5
        scan.angle_max = self.lidar.hor_fov * 0.5
        scan.angle_increment = self.lidar.hor_fov / max(1, self.lidar.hor_res - 1)
        scan.range_min = 0.0
        scan.range_max = self.lidar.max_range
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)
        self.lidar_range_image_pub.publish(self.numpy_to_float_image(image, scan.header.stamp, "lidar"))

    def publish_cameras(self, stamp) -> None:
        if not self.should_publish(self.camera_publish_every_steps):
            return
        self.left_image_pub.publish(self.numpy_to_image(self.left_camera.image(), stamp, "camera_left"))
        self.right_image_pub.publish(self.numpy_to_image(self.right_camera.image(), stamp, "camera_right"))

    def step_simulation(self) -> bool:
        if self.robot.step() == -1:
            self.get_logger().info("Webots requested controller shutdown.")
            return False
        self.step_index += 1
        return True

    def publish_step_data(self) -> None:
        stamp = self.current_stamp()
        self.publish_odometry(stamp)
        self.publish_scan(stamp)
        self.publish_cameras(stamp)
        self.publish_wheel_state(stamp)

    @staticmethod
    def numpy_to_image(frame: np.ndarray, stamp, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgra8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        return msg

    @staticmethod
    def numpy_to_float_image(frame: np.ndarray, stamp, frame_id: str) -> Image:
        data = frame.astype(np.float32, copy=False)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(data.shape[0])
        msg.width = int(data.shape[1])
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = int(data.shape[1] * 4)
        msg.data = data.tobytes()
        return msg

    def spin(self) -> None:
        try:
            while rclpy.ok():
                if not self.step_simulation():
                    break
                rclpy.spin_once(self, timeout_sec=0.0)
                self.stop_if_stale()
                self.apply_drive_command()
                self.publish_step_data()
        finally:
            self.left_motor.set_velocity(0.0)
            self.right_motor.set_velocity(0.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebotsRobotNode()
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
