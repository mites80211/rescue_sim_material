import math
import time as ptime
from collections import deque

from ultralytics import YOLO
import numpy as np
import cv2

import devices
from robotics.controllers import DifferentialController
from robotics.localization import GPSInertialLocalization
from robotics.mapping import OccupancyGrid
from robotics.pathfinding import construct_path, astar, bfs
import constants as const
import erebus
from utils import plot, get_kernel, rotate_point, LidarDataProcessor, BonusMap, sector_crown_mask, bresenham

model = YOLO(const.MODEL_PATH)
start_time = ptime.time()
robot = devices.Robot(time_step=32)
supervisor = erebus.Supervisor(
    emitter=devices.Emitter(robot, 'emitter'),
    receiver=devices.Receiver(robot, 'receiver')
)
localization = GPSInertialLocalization(
    gps=devices.GPS(robot, 'global positioning system'),
    inertial_unit=devices.InertialUnit(robot, 'inertial unit')
)
controller = DifferentialController(
    localization=localization,
    left_wheel_motor=devices.Motor(robot, 'left wheel motor'),
    right_wheel_motor=devices.Motor(robot, 'right wheel motor'),
    axle_length=0.05, wheel_radius=0.02, max_velocity=6.28,
    drive_angle_tolerance=math.pi / 6, turn_gain=10
)
lidar = devices.Lidar(robot, 'lidar', max_range=0.24)
distance_sensors = [
    devices.DistanceSensor(robot, 'left dist sensor', offset=(-0.024, 0, -0.024)),
    devices.DistanceSensor(robot, 'right dist sensor', offset=(0.024, 0, -0.024)),
    devices.DistanceSensor(robot, 'front dist sensor', offset=(0, 0, -0.032))
]
color_sensor = devices.Camera(robot, 'color sensor')
left_camera = devices.Camera(robot, 'left camera', offset=(-0.02, 0, 0), time_step=64)
right_camera = devices.Camera(robot, 'right camera', offset=(0.02, 0, 0), time_step=64)

should_detect_victims = True


def raw_step():
    global should_detect_victims
    should_detect_victims = not should_detect_victims
    return robot.step()


if raw_step() == -1:
    exit()

game_score = 0
game_time = -1


def update_game_information(score, time):
    global game_score, game_time

    game_score = score
    game_time = time


supervisor.add_listener(supervisor.Events.game_information, update_game_information)


def delay(amount):
    for _ in range(math.ceil(amount / robot.time_step)):
        if raw_step() == -1:
            exit()


def get_ground_color():
    img = color_sensor.get_image()
    img[:, :, 2] = np.zeros([img.shape[0], img.shape[1]])
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[0][0]


def euclidean_distance(a, b):
    return np.sqrt(np.square(a[0] - b[0]) + np.square(a[1] - b[1]))


def distance_from(goal):
    return math.sqrt((goal[0] - localization.get_x()) ** 2 + (goal[1] - localization.get_z()) ** 2)


origin = localization.get_x(), localization.get_z()
occupancy_grid = OccupancyGrid(shape=(2000, 2000), spacing=400, origin=origin)
lidar_processor = LidarDataProcessor(lidar, occupancy_grid)
bonus_grid = BonusMap(occupancy_grid, origin)


def get_grid_position():
    return occupancy_grid.world_to_grid(localization.get_x(), localization.get_z())


outer_radius = round(0.048 * occupancy_grid.spacing)
outer_kernel = get_kernel(outer_radius)
outer_grid = np.full(occupancy_grid.shape, True, dtype='uint8')
inner_radius = round(0.0333 * occupancy_grid.spacing)
inner_kernel = get_kernel(inner_radius)
inner_grid = np.full(occupancy_grid.shape, True, dtype='uint8')

stuck_mask = get_kernel(round(0.012 * occupancy_grid.spacing))

visited_mask = get_kernel(round(0.008 * occupancy_grid.spacing))
visited_grid = np.full(occupancy_grid.shape, False, dtype='uint8')

hole_size = round(0.12 * occupancy_grid.spacing)
hole_mask = np.ones((hole_size, hole_size), dtype='uint8')

path_position = get_grid_position()
continue_on_left = False
should_stop_traversing = False


def reset_path_position():
    global path_position, continue_on_left, should_stop_traversing

    path_position = get_grid_position()
    continue_on_left = False
    should_stop_traversing = True


supervisor.add_listener(supervisor.Events.lack_of_progress, reset_path_position)

detected_signs = []


def call_sign(results):
    boxes = results[0].boxes
    for box, cls in zip(boxes.xywhn, boxes.cls):
        controller.stop()

        position = rotate_point(right_camera.offset, yaw=localization.get_yaw()) + localization.get_xyz()
        start = occupancy_grid.world_to_grid(position[0], position[2])

        offset = 0.5 - box.data[0].item()
        direction = rotate_point((0.12 * np.sign(right_camera.offset[0]), 0, 0), yaw=localization.get_yaw() + offset)
        end = position + direction
        end = occupancy_grid.world_to_grid(end[0], end[2])

        line = zip(*bresenham(start, end))

        position = None
        for point in line:
            if occupancy_grid.is_occupied(*point):
                position = point
                break
        if position is not None:
            if distance_from(occupancy_grid.grid_to_world(*position)) < 0.082:
                old_score = game_score
                controller.stop()
                delay(1500)
                sign_type = const.CLS_TO_LETTER[cls.data.item()]
                sign_position = occupancy_grid.grid_to_world(*position)
                supervisor.score_game_element(sign_type, *sign_position)
                for _ in range(5):
                    if not step():
                        break
                if game_score > old_score:
                    detected_signs.append((get_grid_position(), position))
                    bonus_grid.append_sign(sign_position, sign_type)
                print(f'time: {game_time:<5} type: {sign_type} position: '
                      f'{sign_position[0]:<+10.3f} {sign_position[1]:<+10.3f}')


def right_hand(start, forward, length=100):
    path = [start]
    forward = -forward[1], forward[0]
    i = 0
    while len(path) < length and i < length*4:
        i += 1
        r = path[-1][0] + forward[1], path[-1][1] - forward[0]
        f = path[-1][0] + forward[0], path[-1][1] + forward[1]
        if not occupancy_grid.is_occupied(*r):
            forward = forward[1], -forward[0]
            path.append(r)
        elif not occupancy_grid.is_occupied(*f):
            path.append(f)
        else:
            forward = -forward[1], forward[0]  # turn left
    return path


def left_hand(start, forward, length=100):
    path = [start]
    forward = forward[1], -forward[0]
    i = 0
    while len(path) < length and i < length*4:
        i += 1
        l = path[-1][0] - forward[1], path[-1][1] + forward[0]
        f = path[-1][0] + forward[0], path[-1][1] + forward[1]
        if not occupancy_grid.is_occupied(*l):
            forward = -forward[1], forward[0]
            path.append(l)
        elif not occupancy_grid.is_occupied(*f):
            path.append(f)
        else:
            forward = forward[1], -forward[0]  # turn right
    return path


def sign_bfs(start, goal):
    rows, cols = inner_grid.shape
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    queue = deque()
    visited = set()
    parents = {}

    queue.append(start)
    visited.add(start)

    while queue:
        current_node = queue.popleft()
        if current_node == goal:
            return construct_path(parents, current_node)
        for movement in movements:
            new_row = current_node[0] + movement[0]
            new_col = current_node[1] + movement[1]
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbor = (new_row, new_col)
                if neighbor not in visited and inner_grid[neighbor] and occupancy_grid.is_free(*neighbor):
                    queue.append(neighbor)
                    visited.add(neighbor)
                    parents[neighbor] = current_node
    return None


def check_sign(pos, forward):
    edge_1 = occupancy_grid.grid_to_world(*left_hand(forward, np.array(pos) - forward, 6)[-1])
    edge_2 = occupancy_grid.grid_to_world(*right_hand(forward, np.array(pos) - forward, 6)[-1])
    m = np.arctan2(edge_2[1] - edge_1[1], edge_2[0] - edge_1[0])
    xs, ys = bresenham(pos, pos + rotate_point((0, 0, 0.08 * occupancy_grid.spacing), yaw=m).astype(int)[::2])
    xs, ys = xs[2:], ys[2:]
    last = []
    for x, y, i in zip(xs, ys, range(len(xs))):
        if occupancy_grid.occupancy[x, y] == occupancy_grid.occupied:
            break
        if occupancy_grid.occupancy[x, y] == occupancy_grid.free:
            last = (x, y)
        if i > 0.05 * occupancy_grid.spacing and last:
            break
    if last:
        lw = np.array(pos) - round(0.2 * occupancy_grid.spacing)
        up = np.array(pos) + round(0.2 * occupancy_grid.spacing)
        grid = occupancy_grid.occupancy[lw[0]:up[0], lw[1]:up[1]]
        walkable = inner_grid[lw[0]:up[0], lw[1]:up[1]]
        visible = sector_crown_mask(up-lw, (up-lw)//2, 0, round(0.09 * occupancy_grid.spacing),
                                    (m-np.pi/2.5, m+np.pi/2.5))

        rows, cols = grid.shape
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        queue = deque()
        visited = set()
        parents = {}
        start = np.array(last) - lw
        queue.append(tuple(start))
        visited.add(tuple(start))
        point = []
        while queue:
            current_node = queue.popleft()
            if walkable[*current_node] and visible[*current_node]:
                point = construct_path(parents, current_node)
                break
            for movement in movements:
                new_row = current_node[0] + movement[0]
                new_col = current_node[1] + movement[1]
                if 0 <= new_row < rows and 0 <= new_col < cols:
                    neighbor = (new_row, new_col)
                    if neighbor not in visited and grid[neighbor] != occupancy_grid.occupied \
                            and grid[neighbor] != occupancy_grid.unknown:
                        queue.append(neighbor)
                        visited.add(neighbor)
                        parents[neighbor] = current_node
        if point:
            path = sign_bfs(get_grid_position(), tuple(point[-1] + lw))
            if path is None or len(path) > round(0.18 * occupancy_grid.spacing):
                return
            while True:
                path = bfs(inner_grid, get_grid_position(), tuple(point[-1] + lw))
                if path is None or path_traversal(path, should_detect_signs=False):
                    break

            direction = -np.arctan2(*(pos - np.array(get_grid_position())))
            while controller.turn((direction + math.pi / 2) % (2 * np.pi)) > math.pi / 180:
                if raw_step() == -1:
                    exit()
            image = np.repeat(np.repeat(cv2.cvtColor(right_camera.get_image(), cv2.COLOR_BGRA2BGR),
                                        10, axis=1), 16, axis=0)
            results = model(image, device=const.MODEL_DEVICE, conf=const.MODEL_CONF, verbose=False,
                            classes=[0, 2, 3, 4, 5, 6, 7, 8])
            call_sign(results)
            reset_path_position()


checked_pos = []


def detect_signs():
    if not should_detect_victims:
        return
    unchecked_positions = []
    cameras = [left_camera] if continue_on_left else [right_camera]
    for camera in cameras:
        image = np.repeat(np.repeat(cv2.cvtColor(camera.get_image(), cv2.COLOR_BGRA2BGR), 10, axis=1), 16, axis=0)
        results = model(
            image, device=const.MODEL_DEVICE, conf=const.MODEL_CONF,
            verbose=False, classes=[0, 2, 3, 4, 5, 6, 7, 8]
        )
        for box in results[0].boxes.xywhn:
            position = rotate_point(camera.offset, yaw=localization.get_yaw()) + localization.get_xyz()
            start = occupancy_grid.world_to_grid(position[0], position[2])

            offset = 0.5 - box.data[0].item()
            direction = rotate_point((0.12 * np.sign(camera.offset[0]), 0, 0), yaw=localization.get_yaw() + offset)
            end = position + direction
            end = occupancy_grid.world_to_grid(end[0], end[2])

            last = (0, 0)
            line = zip(*bresenham(start, end))
            for point in line:
                if occupancy_grid.is_occupied(*point):
                    unchecked_positions.append((point, last))
                    break
                last = point
    for position in unchecked_positions:
        distances = [euclidean_distance(position[0], x) for x in checked_pos]
        if np.all([x > 2 for x in distances]):
            check_sign(position[0], position[1])
            checked_pos.append(position[0])
        else:
            print('invalid check position')


def update_outer_grid(grid_position, active_area):
    area = np.pad(active_area, (outer_radius, outer_radius)).astype('uint8')
    height, width = area.shape[0] / 2, area.shape[1] / 2
    lower = grid_position[0] - math.floor(height), grid_position[1] - math.floor(width)
    upper = grid_position[0] + math.ceil(height), grid_position[1] + math.ceil(width)
    region = outer_grid[lower[0]:upper[0], lower[1]:upper[1]]
    region[cv2.dilate(area, outer_kernel).astype(bool)] = False


def update_inner_grid(grid_position, active_area):
    area = np.pad(active_area, (inner_radius, inner_radius)).astype('uint8')
    height, width = area.shape[0] / 2, area.shape[1] / 2
    lower = grid_position[0] - math.floor(height), grid_position[1] - math.floor(width)
    upper = grid_position[0] + math.ceil(height), grid_position[1] + math.ceil(width)
    region = inner_grid[lower[0]:upper[0], lower[1]:upper[1]]
    region[cv2.dilate(area, inner_kernel).astype(bool)] = False


def update_visited_grid(grid_position):
    height, width = visited_mask.shape[0] / 2, visited_mask.shape[1] / 2
    lower = grid_position[0] - math.floor(height), grid_position[1] - math.floor(width)
    upper = grid_position[0] + math.ceil(height), grid_position[1] + math.ceil(width)
    region = visited_grid[lower[0]:upper[0], lower[1]:upper[1]]
    region[visited_mask.astype(bool)] = True


def step(should_update_visited_grid=True):
    supervisor.request_game_information()
    if raw_step() == -1:
        return False
    supervisor.handle_received_data()
    grid_position = get_grid_position()
    lidar_scan = lidar_processor.get_lidar_scan(localization.get_yaw())
    active_area = occupancy_grid.update_region(lidar_scan, grid_position)

    update_outer_grid(grid_position, active_area == occupancy_grid.occupied)
    update_inner_grid(grid_position, active_area == occupancy_grid.occupied)
    if should_update_visited_grid:
        update_visited_grid(path_position)

    for sensor in distance_sensors:
        if sensor.get_distance() > 0.1:
            position = rotate_point(sensor.offset, yaw=localization.get_yaw()) + localization.get_xyz()
            position = origin + (np.array([position[0], position[2]]) - origin + 0.06) // 0.12 * 0.12
            update_outer_grid(occupancy_grid.world_to_grid(*position), hole_mask)
            update_inner_grid(occupancy_grid.world_to_grid(*position), hole_mask)
            bonus_grid.mark_tile(position, bonus_grid.hole)

            controller.reverse()
            for _ in range(8):
                if raw_step() == -1:
                    return False
            controller.stop()

    ground_type = bonus_grid.get_ground_type(get_ground_color())
    if ground_type != bonus_grid.free:
        position = np.array([localization.get_x(), localization.get_z()])
        position = origin + (position - origin + 0.06) // 0.12 * 0.12
        if distance_from(position) < 0.05:
            bonus_grid.mark_tile(position, ground_type)
    return True


def search_frontier_bfs(start):
    rows, cols = outer_grid.shape
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    queue = deque()
    visited = set()
    parents = {}

    queue.append(start)
    visited.add(start)

    path = []

    if not inner_grid[start]:
        while queue and not path:
            current_node = queue.popleft()
            for movement in movements:
                new_row = current_node[0] + movement[0]
                new_col = current_node[1] + movement[1]
                if 0 <= new_row < rows and 0 <= new_col < cols:
                    neighbor = (new_row, new_col)
                    if neighbor not in visited and occupancy_grid.is_free(*neighbor):
                        queue.append(neighbor)
                        visited.add(neighbor)
                        parents[neighbor] = current_node

                        if inner_grid[neighbor]:
                            path = construct_path(parents, neighbor)
                            start = neighbor
                            break

        queue.clear()
        visited.clear()
        parents.clear()

        queue.append(start)
        visited.add(start)

    while queue:
        current_node = queue.popleft()
        for movement in movements:
            new_row = current_node[0] + movement[0]
            new_col = current_node[1] + movement[1]
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbor = (new_row, new_col)
                if neighbor not in visited and occupancy_grid.is_free(*neighbor):
                    if inner_grid[neighbor]:
                        queue.append(neighbor)
                    visited.add(neighbor)
                    parents[neighbor] = current_node

                    if not outer_grid[neighbor] and outer_grid[current_node] and not visited_grid[current_node]:
                        return construct_path(parents, current_node)
                    if not outer_grid[current_node] and outer_grid[neighbor] and not visited_grid[neighbor]:
                        return construct_path(parents, neighbor)

    queue.clear()
    visited.clear()
    parents.clear()

    queue.append(start)
    visited.add(start)

    while queue:
        current_node = queue.popleft()
        for movement in movements:
            new_row = current_node[0] + movement[0]
            new_col = current_node[1] + movement[1]
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbor = (new_row, new_col)
                if neighbor not in visited:
                    if occupancy_grid.is_free(*neighbor) and inner_grid[neighbor]:
                        queue.append(neighbor)
                    visited.add(neighbor)
                    parents[neighbor] = current_node

                    if occupancy_grid.is_unknown(*neighbor):
                        return construct_path(parents, current_node)
    return None


def path_correction_bfs(start, max_parents=5):
    rows, cols = outer_grid.shape
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    queue = deque()
    visited = set()
    parents = {}

    queue.append(start)
    visited.add(start)
    parents[start] = 0

    if outer_grid[start]:
        while queue:
            current_node = queue.popleft()
            for movement in movements:
                new_row = current_node[0] + movement[0]
                new_col = current_node[1] + movement[1]
                if 0 <= new_row < rows and 0 <= new_col < cols:
                    neighbor = (new_row, new_col)
                    if neighbor not in visited and occupancy_grid.is_free(*neighbor):
                        visited.add(neighbor)
                        parents[neighbor] = parents[current_node] + 1
                        if parents[neighbor] < max_parents:
                            queue.append(neighbor)

                        if not outer_grid[neighbor]:
                            return neighbor, (-movement[0], -movement[1])
        return None, None

    while queue:
        current_node = queue.popleft()
        for movement in movements:
            new_row = current_node[0] + movement[0]
            new_col = current_node[1] + movement[1]
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbor = (new_row, new_col)
                if neighbor not in visited and occupancy_grid.is_free(*neighbor):
                    visited.add(neighbor)
                    parents[neighbor] = parents[current_node] + 1
                    if parents[neighbor] < max_parents:
                        queue.append(neighbor)

                    if outer_grid[neighbor]:
                        return current_node, movement
    return None, None


def traverse_outer_grid(start, movement, length):
    global continue_on_left

    path = []
    position = start
    counter = 0

    if continue_on_left:
        movement = -movement[1], movement[0]
        while len(path) < length and counter < length * 4:
            counter += 1
            if not outer_grid[position[0] + movement[1], position[1] - movement[0]]:
                position = position[0] + movement[1], position[1] - movement[0]
                movement = movement[1], -movement[0]
                path.append(position)
            elif not outer_grid[position[0] + movement[0], position[1] + movement[1]]:
                position = position[0] + movement[0], position[1] + movement[1]
                path.append(position)
            else:
                movement = -movement[1], movement[0]
        return path

    movement = movement[1], -movement[0]
    while len(path) < length and counter < length * 4:
        counter += 1
        if not outer_grid[position[0] - movement[1], position[1] + movement[0]]:
            position = position[0] - movement[1], position[1] + movement[0]
            movement = -movement[1], movement[0]
            path.append(position)
        elif not outer_grid[position[0] + movement[0], position[1] + movement[1]]:
            position = position[0] + movement[0], position[1] + movement[1]
            path.append(position)
        else:
            movement = movement[1], -movement[0]
    return path


def path_traversal(path, should_detect_signs=True):
    global path_position, should_stop_traversing

    should_stop_traversing = False
    for path_position in path:
        counter = 0
        target = occupancy_grid.grid_to_world(*path_position)
        while distance_from(target) > 0.01:
            if not step(should_update_visited_grid=should_detect_signs):
                exit()
            if not inner_grid[path_position]:
                return False
            if should_stop_traversing:
                should_stop_traversing = False
                return False
            if should_detect_signs:
                detect_signs()
            remaining_angle = controller.drive(target)
            if abs(remaining_angle) <= controller.angle_tolerance:
                counter += 1
                if counter == 50:
                    centre = occupancy_grid.world_to_grid(*target)
                    height, width = stuck_mask.shape[0] / 2, stuck_mask.shape[1] / 2
                    lower = centre[0] - math.floor(height), centre[1] - math.floor(width)
                    upper = centre[0] + math.ceil(height), centre[1] + math.ceil(width)
                    region = inner_grid[lower[0]:upper[0], lower[1]:upper[1]]
                    region[stuck_mask.astype(bool)] = False
                    region = outer_grid[lower[0]:upper[0], lower[1]:upper[1]]
                    region[stuck_mask.astype(bool)] = False
                    controller.reverse()
                    for _ in range(8):
                        if raw_step() == -1:
                            exit()
                    controller.stop()
                    return False
    return True


def exit_maze():
    global path_position

    supervisor.submit_map_bonus(bonus_grid.get_map())
    bonus_grid.pretty_print()
    for _ in range(2):
        if not step():
            exit()

    grid_origin = occupancy_grid.world_to_grid(*origin)
    if not inner_grid[grid_origin]:
        start = grid_origin
        rows, cols = outer_grid.shape
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        queue = deque()
        visited = set()
        parents = {}

        queue.append(start)
        visited.add(start)

        path = []

        while queue and not path:
            current_node = queue.popleft()
            for movement in movements:
                new_row = current_node[0] + movement[0]
                new_col = current_node[1] + movement[1]
                if 0 <= new_row < rows and 0 <= new_col < cols:
                    neighbor = (new_row, new_col)
                    if neighbor not in visited and occupancy_grid.is_free(*neighbor):
                        queue.append(neighbor)
                        visited.add(neighbor)
                        parents[neighbor] = current_node

                        if inner_grid[neighbor]:
                            path = construct_path(parents, neighbor)
                            grid_origin = neighbor
                            break

    while True:
        path_position = get_grid_position()

        if not inner_grid[path_position]:
            start = path_position
            rows, cols = outer_grid.shape
            movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            queue = deque()
            visited = set()
            parents = {}

            queue.append(start)
            visited.add(start)

            path = []

            while queue and not path:
                current_node = queue.popleft()
                for movement in movements:
                    new_row = current_node[0] + movement[0]
                    new_col = current_node[1] + movement[1]
                    if 0 <= new_row < rows and 0 <= new_col < cols:
                        neighbor = (new_row, new_col)
                        if neighbor not in visited and occupancy_grid.is_free(*neighbor):
                            queue.append(neighbor)
                            visited.add(neighbor)
                            parents[neighbor] = current_node

                            if inner_grid[neighbor]:
                                path = construct_path(parents, neighbor)
                                path_position = neighbor
                                break
        path = astar(inner_grid, path_position, grid_origin)
        if path is None:
            exit()
        if path_traversal(path, should_detect_signs=False):
            break
    supervisor.call_end_of_play()
    while raw_step() != -1:
        pass
    exit()


def main():
    global path_position, continue_on_left

    while step():
        if 0 <= game_time < 60 or ptime.time() - start_time > 540:
            exit_maze()

        detect_signs()

        path_position, movement = path_correction_bfs(path_position)
        frontier_flag = path_position is None

        if not frontier_flag:
            path = traverse_outer_grid(path_position, movement, round(0.2 * occupancy_grid.spacing))
            if all([visited_grid[position] for position in path]):
                path_traversal(traverse_outer_grid(path_position, movement, round(0.06 * occupancy_grid.spacing)))
                frontier_flag = True

        if frontier_flag:
            while True:
                path_position = get_grid_position()
                path = search_frontier_bfs(path_position)
                if path is None:
                    exit_maze()
                if path_traversal(path, should_detect_signs=False):
                    break

            continue_on_left = False
            path_position, movement = path_correction_bfs(path_position)
            frontier_flag = path_position is None

            if not frontier_flag:
                path = traverse_outer_grid(path_position, movement, round(0.15 * occupancy_grid.spacing))
                continue_on_left = all([visited_grid[position] for position in path])
        else:
            target = occupancy_grid.grid_to_world(*path_position)

            if continue_on_left:
                counter = 0
                movement = -movement[1], movement[0]
                while distance_from(target) < 0.01:
                    counter += 1
                    if counter >= 25:
                        break
                    if not outer_grid[path_position[0] + movement[1], path_position[1] - movement[0]]:
                        path_position = path_position[0] + movement[1], path_position[1] - movement[0]
                        movement = movement[1], -movement[0]
                    elif not outer_grid[path_position[0] + movement[0], path_position[1] + movement[1]]:
                        path_position = path_position[0] + movement[0], path_position[1] + movement[1]
                    else:
                        movement = -movement[1], movement[0]
                    target = occupancy_grid.grid_to_world(*path_position)
            else:
                counter = 0
                movement = movement[1], -movement[0]
                while distance_from(target) < 0.01:
                    counter += 1
                    if counter >= 25:
                        break
                    if not outer_grid[path_position[0] - movement[1], path_position[1] + movement[0]]:
                        path_position = path_position[0] - movement[1], path_position[1] + movement[0]
                        movement = -movement[1], movement[0]
                    elif not outer_grid[path_position[0] + movement[0], path_position[1] + movement[1]]:
                        path_position = path_position[0] + movement[0], path_position[1] + movement[1]
                    else:
                        movement = movement[1], -movement[0]
                    target = occupancy_grid.grid_to_world(*path_position)

            controller.drive(target)

        if path_position is None:
            reset_path_position()


if __name__ == '__main__':
    main()
