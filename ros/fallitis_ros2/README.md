# Fall-Itis ROS 2 Workspace

ROS 2 Jazzy workspace that extracts the minimum useful pieces from the original
`Fall-Itis` controller into separate, maintainable nodes:

- `fallitis_robot`: Webots bridge for robot I/O and manual driving
- `fallitis_perception`: occupancy mapping and optional remote YOLO camera annotation
- `fallitis_web`: web GUI with live cameras, teleop and zoomable map
- `fallitis_bringup`: launch files for the support stack

The original autonomous exploration logic is intentionally not ported yet.

## Packages

### `fallitis_robot`

Runs inside the Webots controller process. It talks directly to the simulated
devices and exposes standard ROS topics:

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/robot/odometry` (`nav_msgs/Odometry`)
- `/robot/lidar/scan` (`sensor_msgs/LaserScan`)
- `/robot/camera/left/image_raw` (`sensor_msgs/Image`)
- `/robot/camera/right/image_raw` (`sensor_msgs/Image`)

### `fallitis_perception`

Consumes odometry, lidar and cameras, then publishes:

- `/perception/map` (`nav_msgs/OccupancyGrid`)
- `/perception/camera/left/annotated` (`sensor_msgs/Image`)
- `/perception/camera/right/annotated` (`sensor_msgs/Image`)
- `/perception/status` (`std_msgs/String`)

YOLO inference is optional and is expected to run on the host machine through
the local HTTP API script, not inside the VM. If the API is not reachable, the
node still runs and keeps the camera feeds annotated with status text.

### `fallitis_web`

Provides a browser UI with:

- `W`, `A`, `S`, `D` teleoperation
- `Space` to save a snapshot bundle
- live left/right camera streams
- zoomable and pannable occupancy map

The server listens on `0.0.0.0:8080` by default so it can be SSH-forwarded out
of the VM.

## Expected Runtime Layout

This workspace is meant to live in `~/fallitis_ros2` inside the VM, next to:

- `~/ros2_jazzy`
- `~/rescue_sim_material/Fall-Itis`

The detector is expected to be reachable from the VM at:

- `http://10.0.2.2:8000/predict` by default

`10.0.2.2` is the QEMU guest-side address for the host machine.

## Build

```bash
source ~/ros2_jazzy/install/setup.bash
cd ~/fallitis_ros2
./scripts/build_workspace.sh
source install/setup.bash
```

Install the web dependencies once:

```bash
cd ~/fallitis_ros2
./scripts/install_python_deps.sh
```

No model file is required in the VM.

## Run

Quick start:

```bash
cd /home/mites/Dev/Personal/rescue_sim_attempt && ./run_yolo_api.sh --host 0.0.0.0 --port 8000 --model ./yolo.pt
```

```bash
ssh -L 8080:127.0.0.1:8080 -p 2226 sim@127.0.0.1
```

Inside the VM:

```bash
cd ~/fallitis_ros2 && ./scripts/start_support_stack.sh
```

Start the Webots controller bridge from the controller environment:

```bash
cd ~/fallitis_ros2
./scripts/start_controller.sh
```

Start perception + web stack in another shell:

```bash
cd ~/fallitis_ros2
./scripts/start_support_stack.sh
```

If your host YOLO API is on a different port:

```bash
cd ~/fallitis_ros2
YOLO_API_URL=http://10.0.2.2:8001/predict ./scripts/start_support_stack.sh
```

Forward the GUI to the host:

```bash
ssh -L 8080:127.0.0.1:8080 -p 2226 sim@127.0.0.1
```

Then open `http://127.0.0.1:8080`.

## Host YOLO API

Run the existing API on the host machine, outside the VM:

```bash
cd /home/mites/Dev/Personal/rescue_sim_attempt
./run_yolo_api.sh --host 0.0.0.0 --port 8000 --model ./yolo.pt
```

If you want multiple host-side detector instances, use separate ports such as
`8000`, `8001`, `8002`, then point `YOLO_API_URL` at the desired one. There is
no need to forward a whole port range into the VM for this: the guest can
already reach the host directly on `10.0.2.2`.

Note: on this VM the overlay packages are reliably discovered through the
provided wrapper scripts. If you run `ros2 launch` manually, export:

```bash
export AMENT_PREFIX_PATH="$HOME/fallitis_ros2/install:${AMENT_PREFIX_PATH:-}"
```
