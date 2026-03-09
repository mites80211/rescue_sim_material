import math

from devices import Motor


class DifferentialController:

    def __init__(self, localization, left_wheel_motor: Motor, right_wheel_motor: Motor,
                 axle_length, wheel_radius, max_velocity, drive_angle_tolerance=float('+inf'), turn_gain=1):
        self.left_wheel_motor = left_wheel_motor
        self.right_wheel_motor = right_wheel_motor
        self.localization = localization
        self.axle_length = axle_length
        self.half_axle_length = axle_length / 2
        self.wheel_radius = wheel_radius
        self.max_velocity = max_velocity
        self.angle_tolerance = drive_angle_tolerance
        self.turn_gain = turn_gain

    def get_velocities(self):
        return self.left_wheel_motor.get_velocity(), self.right_wheel_motor.get_velocity()

    def set_velocities(self, lwm_velocity, rwm_velocity):
        self.left_wheel_motor.set_velocity(lwm_velocity)
        self.right_wheel_motor.set_velocity(rwm_velocity)

    def stop(self):
        self.set_velocities(0, 0)

    def reverse(self):
        self.set_velocities(-self.max_velocity, -self.max_velocity)

    def turn(self, angle):
        remaining_angle = angle - self.localization.get_yaw()
        remaining_angle = (remaining_angle + 3.14) % 6.28 - 3.14
        angular_velocity = remaining_angle * self.turn_gain
        angular_velocity = angular_velocity * self.half_axle_length / self.wheel_radius
        angular_velocity = min(angular_velocity, self.max_velocity)
        angular_velocity = max(angular_velocity, -self.max_velocity)
        self.set_velocities(-angular_velocity, angular_velocity)
        return abs(remaining_angle)

    def drive(self, target):
        target_angle = math.atan2(self.localization.get_x() - target[0], self.localization.get_z() - target[1])
        remaining_angle = target_angle - self.localization.get_yaw()
        remaining_angle = (remaining_angle + 3.14) % 6.28 - 3.14

        if abs(remaining_angle) > self.angle_tolerance:
            angular_velocity = remaining_angle * self.turn_gain
            angular_velocity = angular_velocity * self.half_axle_length / self.wheel_radius
            angular_velocity = min(angular_velocity, self.max_velocity)
            angular_velocity = max(angular_velocity, -self.max_velocity)
            self.set_velocities(-angular_velocity, angular_velocity)
        else:
            angular_velocity = remaining_angle * self.turn_gain
            angular_velocity = angular_velocity * self.axle_length
            wl = (1 - angular_velocity) / self.wheel_radius
            wr = (1 + angular_velocity) / self.wheel_radius
            mx = max(abs(wl), abs(wr))
            wl = wl / mx * self.max_velocity
            wr = wr / mx * self.max_velocity
            self.set_velocities(wl, wr)
        return abs(remaining_angle)
