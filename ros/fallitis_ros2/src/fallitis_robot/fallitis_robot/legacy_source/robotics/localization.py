from devices import GPS, InertialUnit


class GPSInertialLocalization:
    def __init__(self, gps: GPS, inertial_unit: InertialUnit):
        self.gps = gps
        self.inertial_unit = inertial_unit

    def get_xyz(self):
        return self.gps.get_xyz()

    def get_xz(self):
        return self.get_x(), self.get_z()

    def get_x(self):
        return self.gps.get_xyz()[0]

    def get_y(self):
        return self.gps.get_xyz()[1]

    def get_z(self):
        return self.gps.get_xyz()[2]

    def get_rpy(self):
        return self.inertial_unit.get_rpy()

    def get_roll(self):
        return self.inertial_unit.get_rpy()[0]

    def get_pitch(self):
        return self.inertial_unit.get_rpy()[1]

    def get_yaw(self):
        return self.inertial_unit.get_rpy()[2]
