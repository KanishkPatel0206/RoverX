# 🤖 RoverX

A ROS2 Humble differential drive rover with Gazebo Ignition Fortress simulation, lidar, YOLOv8 object detection, and onboard camera recording.

---

## 📸 Demo

<!-- Add a screenshot or GIF of the rover in Gazebo here -->
![RoverX in Gazebo]()

---

## 🎥 Videos

<!-- Add demo video links here -->

| Teleop Demo | YOLO Detection | Lidar Visualization |
|:-----------:|:--------------:|:-------------------:|
| [▶ Watch]() | [▶ Watch]() | [▶ Watch]() |

---

## 📦 Package Structure

```
my_robot_description/
├── launch/
│   └── simulation.launch.py     # Main simulation launch file
├── my_robot_description/
│   ├── __init__.py
│   ├── teleop_node.py           # WASD keyboard teleoperation
│   ├── cam.py                   # Camera recorder (video + snapshots + rosbag)
│   └── yolo.py                  # YOLOv8 object detection node
├── urdf/
│   └── robot.urdf               # Robot model (base, wheels, caster, lidar, camera)
├── resource/
│   └── my_robot_description
├── test/
├── package.xml
├── setup.cfg
└── setup.py
```

---

## ⚙️ Features

- **Differential Drive** — Two driven wheels + front caster ball, controlled via `/cmd_vel`
- **Gazebo Ignition Fortress** — Full physics simulation with `libignition-gazebo-diff-drive-system`
- **Lidar** — 360° GPU lidar (`gpu_lidar`) visualized in RViz, published on `/scan`
- **Camera** — 640×480 @ 30fps forward-facing camera, bridged via `ros_gz_image`
- **WASD Teleop** — Terminal keyboard control with hold-to-move, release-to-stop
- **Camera Recorder** — Saves MP4 video, JPEG snapshots every 10s, and a full ROS2 bag
- **YOLOv8 Detection** — Real-time object detection on `/camera/image_raw`, publishes annotated frames to `/detection/image_raw` and JSON metadata to `/detection/objects`

---

## 🖼️ Screenshots

<!-- Add RViz, YOLO, or Gazebo screenshots here -->

| RViz Lidar View | YOLO Detection Output |
|:---------------:|:---------------------:|
| ![RViz Lidar]() | ![YOLO Detection]() |

---

## 🛠️ Requirements

- **ROS2 Humble** (Ubuntu 22.04)
- **Ignition Gazebo Fortress**
- **WSL2** (tested) — uses `ogre` render engine for WSL2 GPU compatibility

### Python Dependencies

```bash
pip install "numpy<2"
pip install "opencv-python<4.9"
pip install ultralytics
```

> ⚠️ `numpy<2` is required — ROS Humble's `cv_bridge` was compiled against NumPy 1.x and breaks with NumPy 2.x.

### ROS2 Packages

```bash
sudo apt install ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-ros-gz-image \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2
```

---

## 🚀 Build & Run

### 1. Clone & Build

```bash
git clone https://github.com/KanishkPatel0206/RoverX.git
cd RoverX
# Place inside your ROS2 workspace src/ folder, then:
colcon build --packages-select my_robot_description
source install/setup.bash
```

### 2. Launch Simulation

```bash
ros2 launch my_robot_description simulation.launch.py
```

### 3. Teleop (keyboard control)

```bash
ros2 run my_robot_description teleop_wasd
```

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `SPACE` | Hard Stop |
| `CTRL-C` | Quit |

### 4. Camera Recorder

```bash
ros2 run my_robot_description cam
```

Saves to `output/`:
- `video/video_<timestamp>.mp4`
- `snapshots/snapshot_<timestamp>.jpg` (every 10s)
- `rosbag_<timestamp>/` (camera + lidar + odom + tf)

### 5. YOLO Object Detection

```bash
ros2 run my_robot_description yolo
```

| Topic | Type | Description |
|-------|------|-------------|
| `/detection/image_raw` | `sensor_msgs/Image` | Annotated frame with bounding boxes |
| `/detection/objects` | `std_msgs/String` | JSON detection metadata |

Custom model or threshold:
```bash
ros2 run my_robot_description yolo --ros-args -p model:=yolov8s.pt -p conf_threshold:=0.4
```

---

## 🗺️ Topic Map

```
Gazebo ──► /scan             ──► RViz (Lidar)
       ──► /camera/image_raw ──► cam node   ──► output/video.mp4
                             ──► yolo node  ──► /detection/image_raw
                                            ──► /detection/objects
       ──► /odom
       ──► /tf

teleop ──► /cmd_vel ──► Gazebo (DiffDrive plugin)
```

---

## 🐛 Known Issues / Notes

- **WSL2**: `ogre2` is not supported; render engine is set to `ogre` in the URDF
- **NumPy 2.x**: Must use `numpy<2` — `cv_bridge` ABI incompatibility
- **opencv-python**: Must use `<4.9` to match NumPy 1.x requirement
- **gpu_lidar**: CPU `lidar` type is not implemented in Fortress; `gpu_lidar` + `ogre` is the only working combo on WSL2

---

## 📄 License

TODO

---

## 🙋 Author

**Kanishk Patel** — [@KanishkPatel0206](https://github.com/KanishkPatel0206)
