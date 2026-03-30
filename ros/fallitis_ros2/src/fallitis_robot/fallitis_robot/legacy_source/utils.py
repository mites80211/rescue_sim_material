import atexit
import copy
import html
import math
import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import cv2

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


def plot(*args):
    if plt is None:
        return
    for arg in args:
        plt.matshow(arg)
    plt.show()


def get_kernel(radius):
    diameter = radius * 2
    radius = min(radius, diameter - radius)
    y, x = np.ogrid[:diameter + 1, :diameter + 1]
    a = np.sqrt((x - radius) ** 2 + (y - radius) ** 2) <= radius
    return np.array(a, dtype='uint8')


def rotate_point(point=(0, 0, -1), roll=0, pitch=0, yaw=0):
    rx = np.array([[1, 0, 0], [0, np.cos(pitch), -np.sin(pitch)], [0, np.sin(pitch), np.cos(pitch)]])
    ry = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    rz = np.array([[np.cos(roll), -np.sin(roll), 0], [np.sin(roll), np.cos(roll), 0], [0, 0, 1]])
    return ry @ rx @ rz @ point


class LidarDataProcessor:

    def __init__(self, lidar, occupancy_grid):
        self.lidar = lidar
        self.occupancy_grid = occupancy_grid

        self.first_ray = 0.0
        self.first_lyr = lidar.ver_fov / 2.0
        self.hor_angle = lidar.hor_fov / (lidar.hor_res - 1)
        self.ver_angle = lidar.ver_fov / (lidar.ver_res - 1)
        self.max_radius = round(lidar.max_range * occupancy_grid.spacing)
        self.polar_shape = lidar.hor_res, self.max_radius
        self.polar_X = np.arange(self.polar_shape[0])
        self.polar_Y = np.arange(self.polar_shape[1])
        self.theta = np.sqrt(1 - np.square(np.sin(
            self.first_lyr - self.ver_angle * np.arange(lidar.ver_res)[:, np.newaxis])))
        self.flags = cv2.WARP_INVERSE_MAP + cv2.WARP_POLAR_LINEAR + cv2.WARP_FILL_OUTLIERS
        self.warp_shape = self.polar_shape[1] * 2, self.polar_shape[1] * 2
        self.warp_center = self.polar_shape[1], self.polar_shape[1]

    def get_lidar_scan(self, yaw):
        image = self.lidar.get_range_image() * self.theta
        image = np.nan_to_num(image, nan=self.lidar.max_range, posinf=self.lidar.max_range, neginf=0.0)
        image = np.rint(image * self.occupancy_grid.spacing).astype(int)
        polar = np.full(self.polar_shape, self.occupancy_grid.unknown)
        for layer in image:
            mask = (layer > 0) & (layer < self.polar_shape[1])
            polar[self.polar_X[mask], layer[mask]] = self.occupancy_grid.occupied
            polar[self.polar_Y < layer[:, None]] = self.occupancy_grid.free
        mask = (image <= 0) | (image >= self.polar_shape[1])
        polar[self.polar_X[np.all(mask, axis=0)], :] = self.occupancy_grid.free
        shift = round(math.degrees(yaw) * self.polar_shape[0] / 360)
        polar = np.roll(polar, -shift, axis=0)
        return cv2.warpPolar(polar, self.warp_shape, self.warp_center, self.max_radius, self.flags)


class OccupancyGridViewer:

    def __init__(
            self,
            occupancy_grid,
            origin_position=None,
            window_name='Occupancy Grid',
            refresh_period=0.15,
            crop_margin=60,
            max_size=900,
            host='127.0.0.1',
            port=8765
    ):
        requested = os.getenv('FALLITIS_SHOW_OCCUPANCY')
        self.enabled = requested is None or requested.lower() not in {'0', 'false', 'no'}

        self.occupancy_grid = occupancy_grid
        self.origin_position = origin_position
        self.window_name = window_name
        self.refresh_period = refresh_period
        self.crop_margin = crop_margin
        self.max_size = max_size
        self.host = os.getenv('FALLITIS_OCCUPANCY_HOST', host)
        self.port = int(os.getenv('FALLITIS_OCCUPANCY_PORT', str(port)))
        self.last_update = 0.0
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.server = None
        self.server_thread = None
        self.server_error = None

        if self.enabled:
            self._start_server()
        atexit.register(self.close)

    def _start_server(self):
        viewer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/' or self.path.startswith('/?'):
                    body = viewer._build_page().encode('utf-8')
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith('/frame'):
                    with viewer.frame_lock:
                        frame = viewer.latest_frame
                    if frame is None:
                        body = b'waiting for first frame'
                        self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                        self.send_header('Content-Type', 'text/plain; charset=utf-8')
                        self.send_header('Cache-Control', 'no-store')
                        self.send_header('Content-Length', str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    return

                self.send_error(HTTPStatus.NOT_FOUND)

            def log_message(self, format, *args):
                return

        try:
            self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:
            self.server_error = exc
            self.enabled = False
            print(f'occupancy viewer disabled: {exc}')
            return

        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        print(f'occupancy viewer: http://{self.host}:{self.port}')

    def _build_page(self):
        title = html.escape(self.window_name)
        return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #101417;
      --panel: #172026;
      --text: #edf2f7;
      --muted: #91a0ad;
      --accent: #ffb703;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      min-height: 100vh;
      background:
        radial-gradient(circle at top, rgba(255, 183, 3, 0.18), transparent 30%),
        linear-gradient(180deg, #0e1418, var(--bg));
      color: var(--text);
      font-family: "Segoe UI", "Helvetica Neue", sans-serif;
    }}
    main {{
      width: min(1100px, calc(100vw - 32px));
      margin: 24px auto;
      padding: 20px;
      background: rgba(23, 32, 38, 0.88);
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 18px;
      box-shadow: 0 16px 40px rgba(0, 0, 0, 0.28);
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: clamp(1.2rem, 2vw, 1.6rem);
    }}
    p {{
      margin: 0 0 16px;
      color: var(--muted);
    }}
    .legend {{
      display: flex;
      gap: 16px;
      flex-wrap: wrap;
      margin-bottom: 14px;
      font-size: 0.95rem;
    }}
    .legend span {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .swatch {{
      width: 12px;
      height: 12px;
      border-radius: 999px;
      display: inline-block;
    }}
    .frame {{
      display: grid;
      place-items: center;
      min-height: 320px;
      padding: 12px;
      background: #0a0f13;
      border-radius: 14px;
      overflow: hidden;
    }}
    img {{
      max-width: 100%;
      height: auto;
      image-rendering: pixelated;
      border-radius: 10px;
      background: #1f2b33;
    }}
    code {{
      color: var(--accent);
    }}
  </style>
</head>
<body>
  <main>
    <h1>{title}</h1>
    <p>Aggiornamento automatico dal loop del controller.</p>
    <div class="legend">
      <span><i class="swatch" style="background:#ebebeb"></i>free</span>
      <span><i class="swatch" style="background:#606060"></i>unknown</span>
      <span><i class="swatch" style="background:#1e1e1e"></i>occupied</span>
      <span><i class="swatch" style="background:#00b400"></i>start</span>
      <span><i class="swatch" style="background:#ffb400"></i>target</span>
      <span><i class="swatch" style="background:#ff3b30"></i>robot</span>
    </div>
    <div class="frame">
      <img id="frame" alt="Live occupancy grid">
    </div>
    <p id="status">Connessione a <code>/frame</code>...</p>
  </main>
  <script>
    const frame = document.getElementById('frame');
    const status = document.getElementById('status');
    function refresh() {{
      const ts = Date.now();
      const next = new Image();
      next.onload = () => {{
        frame.src = next.src;
        status.textContent = 'Ultimo frame: ' + new Date().toLocaleTimeString();
      }};
      next.onerror = () => {{
        status.textContent = 'In attesa del primo frame...';
      }};
      next.src = '/frame?ts=' + ts;
    }}
    refresh();
    setInterval(refresh, {max(100, int(self.refresh_period * 1000))});
  </script>
</body>
</html>
"""

    def _grid_to_image(self, point, lower, shape):
        if point is None:
            return None
        x = int(point[0] - lower[0])
        y = int(point[1] - lower[1])
        if x < 0 or y < 0 or x >= shape[0] or y >= shape[1]:
            return None
        return x, shape[1] - 1 - y

    def _get_bounds(self, robot_position):
        occupancy = self.occupancy_grid.occupancy
        known = occupancy != self.occupancy_grid.unknown

        if np.any(known):
            xs, ys = np.where(known)
            lower = np.array([xs.min(), ys.min()]) - self.crop_margin
            upper = np.array([xs.max(), ys.max()]) + self.crop_margin + 1
        elif robot_position is not None:
            position = np.array(robot_position)
            lower = position - self.crop_margin
            upper = position + self.crop_margin + 1
        else:
            lower = np.zeros(2, dtype=int)
            upper = np.array(occupancy.shape)

        lower = np.maximum(lower, 0)
        upper = np.minimum(upper, np.array(occupancy.shape))
        return lower.astype(int), upper.astype(int)

    def render(self, robot_position=None, path_position=None):
        lower, upper = self._get_bounds(robot_position)
        cropped = self.occupancy_grid.occupancy[lower[0]:upper[0], lower[1]:upper[1]]

        image = np.full((*cropped.shape, 3), 96, dtype=np.uint8)
        image[cropped == self.occupancy_grid.free] = (235, 235, 235)
        image[cropped == self.occupancy_grid.occupied] = (30, 30, 30)
        image = np.ascontiguousarray(np.flip(np.transpose(image, (1, 0, 2)), axis=0))

        markers = [
            (self.origin_position, (0, 180, 0)),
            (path_position, (0, 180, 255)),
            (robot_position, (0, 0, 255)),
        ]
        for point, color in markers:
            image_point = self._grid_to_image(point, lower, cropped.shape)
            if image_point is not None:
                cv2.circle(image, image_point, 3, color, -1, lineType=cv2.LINE_AA)

        height, width = image.shape[:2]
        scale = min(self.max_size / max(height, 1), self.max_size / max(width, 1))
        if scale != 1:
            image = cv2.resize(
                image,
                (max(1, round(width * scale)), max(1, round(height * scale))),
                interpolation=cv2.INTER_NEAREST
            )

        cv2.rectangle(image, (0, 0), (260, 30), (32, 32, 32), -1)
        cv2.putText(
            image,
            'green=start orange=target red=robot',
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )
        return image

    def update(self, robot_position=None, path_position=None):
        if not self.enabled:
            return
        if time.time() - self.last_update < self.refresh_period:
            return

        success, buffer = cv2.imencode('.png', self.render(robot_position, path_position))
        if not success:
            self.enabled = False
            return

        with self.frame_lock:
            self.latest_frame = buffer.tobytes()
        self.last_update = time.time()

    def close(self):
        if self.server is None:
            return
        self.server.shutdown()
        self.server.server_close()
        self.server = None
        if self.server_thread is not None:
            self.server_thread.join(timeout=1.0)
            self.server_thread = None


class ANSI:
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    COLOR_DEFAULT = '\033[39m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    INVISIBLE = '\033[08m'
    REVERSE = '\033[07m'
    BG_BLACK = '\033[40m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_MAGENTA = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_WHITE = '\033[47m'
    BG_DEFAULT = '\033[49m'
    RESET = '\033[0m'


class BonusMap:
    free = 0
    wall = 1
    hole = 2
    swamp = 3
    checkpoint = 4
    start = 5
    blue = 6
    purple = 7
    red = 8
    green = 9

    def __init__(self, occupancy_grid, origin):
        self.occupancy_grid = occupancy_grid
        self.shape = (
            math.floor(occupancy_grid.shape[0] / (0.12 * occupancy_grid.spacing)) * 4 + 1,
            math.floor(occupancy_grid.shape[1] / (0.12 * occupancy_grid.spacing)) * 4 + 1
        )
        self.array = [[0] * self.shape[1] for _ in range(self.shape[0])]
        self.origin = origin
        self.sign_list = []
        self.mark_tile(origin, self.start)

    def get_ground_type(self, color):
        h, s, v = color
        if 62 < h < 68:
            return self.green
        elif 75 < h < 80:
            return self.swamp
        elif 85 < h < 95:
            if v < 80:
                return self.red
            elif v > 240:
                return self.checkpoint
        elif 110 < h < 120:
            if v > 230:
                return self.blue
            elif v < 230:
                return self.purple
        return self.free

    def mark_tile(self, centre, value):
        centre = self.occupancy_grid.world_to_grid(*centre)
        centre = [x // (0.12 * self.occupancy_grid.spacing) for x in centre]
        centre = [math.floor(x) * 4 + 2 for x in centre]
        for ox, oy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            self.array[centre[0] + ox][centre[1] + oy] = value

    def append_sign(self, sign_position, sign_type):
        self.sign_list.append((sign_position, sign_type))

    def get_map(self):
        array = copy.deepcopy(self.array)

        occupied = self.occupancy_grid.occupancy == self.occupancy_grid.occupied
        origin = self.occupancy_grid.world_to_grid(*self.origin)
        d = round(0.12 * self.occupancy_grid.spacing / 4)
        offset = [(x - d // 2) % d for x in origin]
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                x, y = i * d + offset[0], j * d + offset[1]
                if array[i][j] == self.free and occupied[x:x + d, y:y + d].sum() > 13 and not (i % 2 and j % 2):
                    array[i][j] = self.wall

        # for sign_position, sign_type in self.sign_list:
        #     grid_sign_position = self.occupancy_grid.world_to_grid(*sign_position)
        #     grid_sign_position = [x / (0.06 * self.occupancy_grid.spacing) for x in grid_sign_position]
        #     grid_sign_position = [math.floor(x) * 2 + 1 for x in grid_sign_position]
        #
        #     half_tile_center = [(x - 1) / 2 for x in grid_sign_position]
        #     half_tile_center = [x * (0.06 * self.occupancy_grid.spacing) for x in half_tile_center]
        #     half_tile_center = self.occupancy_grid.grid_to_world(*half_tile_center)
        #
        #     difference = sign_position[0] - half_tile_center[0], sign_position[1] - half_tile_center[1]
        #     if difference[0] < 0:
        #         if difference[1] < 0:
        #             movements = [(1, 0), (0, -1)]
        #         else:
        #             movements = [(1, 0), (0, 1)]
        #     else:
        #         if difference[1] < 0:
        #             movements = [(-1, 0), (0, -1)]
        #         else:
        #             movements = [(-1, 0), (0, 1)]
        #     if abs(difference[0]) < abs(difference[1]):
        #         movements.reverse()
        #     # movements.extend([(1, 0), (-1, 0), (0, 1), (0, -1)])
        #
        #     for movement in movements:
        #         i, j = grid_sign_position[0] + movement[0], grid_sign_position[1] + movement[1]
        #         if type(array[i][j]) != str and array[i][j] != self.wall:
        #             continue
        #         if type(array[i][j]) == str:
        #             array[i][j] += sign_type
        #         else:
        #             array[i][j] = sign_type
        #         break

        array = np.flip(np.array(array).T, axis=0)
        return array.astype(str)

    def pretty_print(self):
        d = self.get_map()
        for m in d:
            for mm in m:
                if mm == '0':  # Normal
                    print(f'0', end='')
                elif mm == '1':  # Walls
                    print(f'{ANSI.BG_WHITE}{ANSI.BLACK}1{ANSI.RESET}', end='')
                elif mm == '2':  # Holes
                    print(f'{ANSI.BG_WHITE}{ANSI.BOLD}2{ANSI.RESET}', end='')
                elif mm == '3':  # Swamps
                    print(f'{ANSI.YELLOW}3{ANSI.RESET}', end='')
                elif mm == '4':  # Checkpoints
                    print(f'{ANSI.UNDERLINE}4{ANSI.RESET}', end='')
                elif mm == '5':  # Stating tile
                    print(f'{ANSI.GREEN}5{ANSI.RESET}', end='')
                elif mm == '6':  # 1to2
                    print(f'{ANSI.BLUE}6{ANSI.RESET}', end='')
                elif mm == '7':  # 1to3
                    print(f'{ANSI.MAGENTA}7{ANSI.RESET}', end='')
                elif mm == '8':  # 2to3
                    print(f'{ANSI.RED}8{ANSI.RESET}', end='')
                else:  # Victims
                    print(f'{ANSI.BG_WHITE}{ANSI.CYAN}{mm}{ANSI.RESET}', end='')
            print('')


def bresenham(start, end):
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    xs, ys = [], []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        xs.append(coord[0])
        ys.append(coord[1])
        error -= abs(dy)
        if error < 0:
            xs.append(coord[0])
            ys.append(coord[1] + 1)
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        xs.reverse()
        ys.reverse()
    return xs, ys


def sector_crown_mask(shape, centre, inner_radius, outer_radius, angle_range):
    x, y = np.ogrid[:shape[0], :shape[1]]
    cx, cy = centre
    tmin, tmax = angle_range
    # ensure stop angle > start angle
    if tmax < tmin:
        tmax += 2 * np.pi
    # convert cartesian to polar coordinates
    r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy)
    theta = np.arctan2(x - cx, y - cy) - tmin
    theta %= (2 * np.pi)  # wrap angles between 0 and tau
    circular_mask = r2 <= outer_radius ** 2
    circular_mask &= r2 >= inner_radius ** 2
    angular_mask = theta <= (tmax - tmin)
    return circular_mask * angular_mask
