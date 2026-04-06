<div align="center">

<!-- Animated title using SVG -->
<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=36&duration=3000&pause=1000&color=00D4FF&center=true&vCenter=true&width=600&lines=🤖+RoverX;ROS+2+Differential+Drive+Rover;Gazebo+%7C+YOLOv8+%7C+LiDAR" alt="Typing SVG" />

<br/>

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition_Fortress-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-purple?style=for-the-badge)
![WSL2](https://img.shields.io/badge/WSL2-Compatible-green?style=for-the-badge&logo=linux&logoColor=white)

<br/>

*A fully simulated differential drive rover with 360° LiDAR, real-time YOLOv8 object detection, and onboard camera recording — running on ROS 2 Humble + Ignition Gazebo Fortress.*

</div>

---

## 📸 Preview

<div align="center">

<!-- Replace the src below with a real screenshot or GIF once available -->
<img src="assets/roverx_preview.gif" alt="RoverX in Gazebo Fortress" width="80%" />

> *Rover navigating in Gazebo Fortress with LiDAR scan and YOLOv8 overlay*

</div>

---

## 🎬 Demo Videos

<div align="center">

> 🎥 Drop `.mp4` files in the `assets/` folder and link them below, or embed YouTube thumbnails using the template in the comment.

| Teleop Demo | YOLO Detection | LiDAR Visualization |
|:-----------:|:--------------:|:--------------------:|
| [![Teleop](https://img.shields.io/badge/▶_Watch-Teleop_Demo-blue?style=for-the-badge)]() | [![YOLO](https://img.shields.io/badge/▶_Watch-YOLO_Demo-purple?style=for-the-badge)]() | [![LiDAR](https://img.shields.io/badge/▶_Watch-LiDAR_Demo-orange?style=for-the-badge)]() |
| WASD keyboard control | Real-time bounding boxes | RViz 360° scan |

<!-- 
  ── YouTube embed template ──────────────────────────────────────────
  Replace YOUR_VIDEO_ID with the actual YouTube video ID.

  [![Watch on YouTube](https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

  ── Local video / GitHub release ────────────────────────────────────
  Upload the .mp4 to a GitHub Release, then link the download URL:

  [![Teleop Demo](assets/teleop_thumbnail.png)](https://github.com/KanishkPatel0206/RoverX/releases/download/v1.0/teleop_demo.mp4)
  ─────────────────────────────────────────────────────────────────── 
-->

</div>

---

## ✨ Features

<div align="center">

| Module | Description |
|--------|-------------|
| 🚗 **Differential Drive** | Two driven wheels + front caster ball, controlled via `/cmd_vel` |
| 🌐 **Gazebo Ignition Fortress** | Full physics simulation with `libignition-gazebo-diff-drive-system` |
| 📡 **360° LiDAR** | GPU LiDAR (`gpu_lidar`) → `/scan` topic, visualized in RViz |
| 📷 **Onboard Camera** | 640×480 @ 30fps, bridged via `ros_gz_image` |
| ⌨️ **WASD Teleop** | Terminal keyboard control — hold to move, release to stop |
| 🎥 **Camera Recorder** | Saves MP4 video, JPEG snapshots every 10s, and a full ROS 2 bag |
| 🧠 **YOLOv8 Detection** | Real-time object detection on `/camera/image_raw` with annotated output |

</div>

---

## 🗂️ Package Structure

```
RoverX/
├── launch/
│   ├── simulation.launch.py        # Main simulation launch
│   └── obstacle_avoidance.launch.py
├── my_robot_description/
│   ├── __init__.py
│   ├── teleop_node.py              # WASD keyboard teleoperation
│   ├── cam.py                      # Camera recorder (MP4 + snapshots + rosbag)
│   └── yolo.py                     # YOLOv8 object detection node
├── urdf/
│   └── robot.urdf                  # Robot model (base, wheels, caster, LiDAR, camera)
├── resource/
│   └── my_robot_description
├── package.xml
└── setup.py
```

---

## 🛠️ Requirements

**System:**
- Ubuntu 22.04 (bare metal or WSL2)
- ROS 2 Humble
- Ignition Gazebo Fortress

**Python:**

```bash
pip install "numpy<2"           # cv_bridge ABI requires NumPy 1.x
pip install "opencv-python<4.9"
pip install ultralytics
```

> ⚠️ `numpy<2` is **required** — ROS 2 Humble's `cv_bridge` was compiled against NumPy 1.x and breaks silently with NumPy 2.x.

**ROS 2 Packages:**

```bash
sudo apt install ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-ros-gz-image \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2
```

---

## 🚀 Build & Run

### 1 · Clone & Build

```bash
git clone https://github.com/KanishkPatel0206/RoverX.git
# Move into your ROS 2 workspace src/ folder, then:
colcon build --packages-select my_robot_description
source install/setup.bash
```

### 2 · Launch Simulation

```bash
ros2 launch my_robot_description simulation.launch.py
```

### 3 · Teleop (keyboard)

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

### 4 · Camera Recorder

```bash
ros2 run my_robot_description cam
```

Output saved to `output/`:

```
output/
├── video/
│   └── video_<timestamp>.mp4
├── snapshots/
│   └── snapshot_<timestamp>.jpg    # every 10 seconds
└── rosbag_<timestamp>/             # camera + LiDAR + odom + tf
```

### 5 · YOLO Object Detection

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
Gazebo ──► /scan               ──► RViz (LiDAR)
       ──► /camera/image_raw   ──► cam node   ──► output/video.mp4
                               ──► yolo node  ──► /detection/image_raw
                                              ──► /detection/objects
       ──► /odom
       ──► /tf

teleop ──► /cmd_vel ──► Gazebo (DiffDrive plugin)
```

---

## 🖼️ Screenshots

<div align="center">

| RViz LiDAR View | YOLO Detection Output |
|:---------------:|:---------------------:|
| <img src="assets/rviz_lidar.png" alt="RViz LiDAR" width="350"/> | <img src="assets/yolo_detection.png" alt="YOLO Detection" width="350"/> |

<!-- Add screenshots to the assets/ folder and update paths above -->

</div>

---

## 🐛 Known Issues

| Issue | Fix |
|-------|-----|
| WSL2: `ogre2` render crash | Render engine set to `ogre` in URDF |
| `cv_bridge` crash with NumPy 2.x | Use `numpy<2` |
| OpenCV import error | Use `opencv-python<4.9` |
| `gpu_lidar` not found | CPU `lidar` type unsupported in Fortress; use `gpu_lidar` + `ogre` |

---

## 📄 License

MIT — see [LICENSE](LICENSE)

---

<div align="center">

**Made with ❤️ by [Kanishk Patel](https://github.com/KanishkPatel0206)**

[![GitHub](https://img.shields.io/badge/GitHub-KanishkPatel0206-black?style=flat-square&logo=github)](https://github.com/KanishkPatel0206)
[![Portfolio](https://img.shields.io/badge/Portfolio-kanishkpatel0206.github.io-blue?style=flat-square)](https://kanishkpatel0206.github.io/Kanishk)

![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=KanishkPatel0206.RoverX)

</div>
