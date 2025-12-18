# Neato Tag

## Intro

We created **Neato Tag** to explore our interests in machine vision, localization, and multi-robot systems. This project combines these areas by simulating a game of tag between two Neatos, one controlled by algorithms and the other controlled by a user.

The chaser Neato uses machine vision to follow the escaper Neato and attempt to bump into it. The escaper Neato is controlled by a user using teleop controls. If the chaser Neato’s bump sensors are triggered when the escaper Neato covers the chaser’s camera view, the controls will switch. The new chaser Neato then gives the new escaper a few seconds to move away before the chase begins again.

See our website for more details on what we did and how:  
https://kelsedilla.github.io/Neato_tag/

Our project contains two parts:
- The main program that runs the Neato Tag game using machine vision
- A particle filter system that we were not able to get fully working

Instructions to run both are provided below.

---

## Requirements

### System Requirements
- ROS 2 Humble
- Ubuntu 22.04 (Jammy Jellyfish)
- Python 3.8+

### ROS 2 Packages
- nav2_amcl
- nav2_map_server
- slam_toolbox

### Python Packages
- OpenCV
- NumPy

### Hardware
Two Neato vacuum robots, each equipped with:
- LIDAR scanner
- Camera
- Pink sticky notes

---

## Running the Project

First, ensure your Neatos are equipped with the hardware listed above and that you have the necessary system requirements installed.

### Clone the Repository

Navigate into your `ros2_ws/src` folder:

```
git clone git@github.com:kelsedilla/Neato_tag.git
```

### Build and Source the Package

From the `ros2_ws` directory:

```
colcon build --symlink-install --packages-select neato_tag
source ~/ros2_ws/install/setup.bash
```

### Install Required ROS 2 Packages

```
sudo apt install -y ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-slam-toolbox
```

### Run Neato Tag (Machine Vision)

Install OpenCV:

```
pip install opencv-python
```

Connect to the Neatos (replace IPs):

Neato 1:
```
ros2 launch neato_node2 bringup_multi.py host:=<NEATO_1_IP> robot_name:=neato1 udp_video_port:=5002 udp_sensor_port:=7777
```

Neato 2:
```
ros2 launch neato_node2 bringup_multi.py host:=<NEATO_2_IP> robot_name:=neato2 udp_video_port:=5003 udp_sensor_port:=7778
```

Run teleop:
```
ros2 run neato_tag teleop_single_swap
```

Run color tracking:
```
ros2 run neato_tag color_tracking_swap
```

---

## Neato Tag Logic

### Chaser Logic
- Camera Input: Subscribe to chaser Neato’s camera feed
- Image Processing: Convert ROS image messages to OpenCV images
- Color Detection: Detect runner Neato using color-based detection to return a bounding box
- Motion Calculation:
  - If runner detected: Compute heading angle from bounding box center and drive toward it
  - If runner not detected: Rotate in place in the last known direction of search
- Publish Command: Publish linear and angular velocity to chaser Neato

### Upon Bump (Tag)
- Immediate Stop: Publish stop command to chaser Neato
- Role Swap:
  - Publish swap signal to runner over `/swap_role`
  - Swap names of chaser and runner Neato within chaser code
- Re-setup: Reinitialize subscriptions and publishers for the new chaser
- Wait Period: Pause chaser movement for a short duration

### Runner Logic
- Keyboard Input: Read user key presses and map to linear and angular velocity commands
- Publish Command: Publish linear and angular velocity to runner Neato

### Upon Swap Signal (`/swap_roles`)
- Role Swap: Swap names of chaser and runner Neato within runner code
- Re-setup: Reinitialize velocity publisher for the new runner

---

## Color Detection Logic
![Color detection example](docs/images/color_detection_example.png)
1. Get image from camera
2. Mask out all colors except those near a specified neon pink color
3. Apply Canny edge detection and morphological closing
4. Retrieve contours and extract rectangles using `cv2.findContours`
5. Remove small rectangles and merge remaining ones into a bounding box
6. Compute the angle of the bounding box center using the camera’s FOV
