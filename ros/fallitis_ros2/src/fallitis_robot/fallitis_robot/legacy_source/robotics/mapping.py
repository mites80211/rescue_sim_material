import numpy as np


class OccupancyGrid:
    unknown, free, occupied = range(3)

    def __init__(self, shape, spacing, origin):
        self.shape = shape
        self.spacing = spacing
        shape = [x // 2 for x in shape]
        origin = [round(x * self.spacing) for x in origin]
        self.offset = (shape[0] - origin[0], shape[1] + origin[1])
        self.occupancy = np.full(self.shape, self.unknown)

    def world_to_grid(self, x, y):
        return round(x * self.spacing + self.offset[0]), round(-y * self.spacing + self.offset[1])

    def grid_to_world(self, x, y):
        return (x - self.offset[0]) / self.spacing, -(y - self.offset[1]) / self.spacing

    def is_unknown(self, x, y):
        return self.occupancy[x, y] == self.unknown

    def is_free(self, x, y):
        return self.occupancy[x, y] == self.free

    def is_occupied(self, x, y):
        return self.occupancy[x, y] == self.occupied

    def update_region(self, grid, center):
        h, w = grid.shape[0] // 2, grid.shape[1] // 2
        r = self.occupancy[center[0] - h: center[0] + h, center[1] - w: center[1] + w]
        np.putmask(r, grid > r, grid)
        return r
