# Piper ROS – ROS 2 Control Stack for AgileX Piper Arm (Jetson + Humble)

This repository contains the complete ROS 2 control system for the **AgileX Piper robotic arm**, check the official repositiry:
https://github.com/agilexrobotics/piper_ros/tree/humble


Also this repository include:

- Joint-level control over **CAN**
- **Cartesian control** (numerical IK / analytical IK)
- **Joystick teleoperation** (GameSir / Xbox / Logitech)
- Additional tools such as depth camera processing and game controller utilities
- URDF/Xacro robot description integration

Tested on:
- **NVIDIA Jetson Orin / Xavier NX**
- **ROS 2 Humble**
- Linux with CAN support (`can0`)

---

## Repository Structure

```
piper_ros/
 ├── src/
 │   ├── piper/                       # Main robot control
 │   │   ├── control_manager
 │   │   ├── delta_manager
 │   │   ├── piper_single_ctrl
 │   │   ├── joystick_controller.py
 │   │   ├── piper_conrol_gamesir.py
 │   │   ├── game_controller_manager.py
 │   │   └── scripts/depth_camera.py
 │   │
 │   ├── piper_teleoperation/         # NEW: Joystick teleoperation
 │   │   ├── launch/joy_controller.launch.py
 │   │   ├── scripts/joystick_controller_node.py
 │   │   ├── scripts/ant.py
 │   │   ├── CMakeLists.txt
 │   │   └── package.xml
 │   │
 │   ├── object_recognition/          # (Not yet documented) 
 │   │
 │   ├── piper_description/           # URDF + Xacro + RViz files
 │   │
 │   └── piper_kinematics/            # Cartesian IK controller
 │       └── cartesian_controller
 │
 └── README.md
```

---

# Installation

## Install dependencies

You can dowloand the dependencies from the official repository mention before or install the dependecies and requerimentes following
this commands

### System packages
- ROS 2 Humble
- python3-can
- can-utils
- libopencv-dev
- ros-humble-joy
- ros-humble-ros2-control
- ros-humble-ros2-controllers
- ros-humble-vision-msgs

### Python packages
- numpy
- scipy
- opencv-python
- python-can

```bash
bash dependencies.sh
pip3 install -r requirements.txt
```
# Run the code 
First find all the can ports connected to the jetson and activate 

## 1. Enable CAN interface
```bash
bash find_all_can_port.sh
bash can_activate.sh can_piper 1000000 "1-2.4:1.0"      # it can change
```

## 2. Enable CAN interface
Run the 2 following launches files 
```bash
#  First run this line, rviz will be open, but there will be some errors, it'll be solve with the next line

ros2 launch piper piper_connect.launch.py can_port:=can_piper

#  Run this line, it charge all the urdfs of the robot

ros2 launch piper controller_general.launch.py

```

## 3. Control wit the gamepad
To control with the gamepad, first install the dependencies, then connect the gamepad and run the next code.
 
```bash
ros2 run game_controller_manager.py
```
There are 2 modes of control the arm with the game pad 

### Cartesian controller
You can move the arm in x and z coordinates 


### Joint controller
You can move all the joints of the arm 


## 4. Foxglove connection 


---

# Joint Control (CAN Mode)

### Launch file: `piper_single_ctrl.launch.py`

Loads the URDF (GUI disabled) and starts the joint-based controller.

Run:
```bash
ros2 launch piper piper_single_ctrl.launch.py
```

### Parameters

| Parameter | Default | Description |
|----------|----------|-------------|
| `can_port` | `can0` | CAN bus interface |
| `auto_enable` | `true` | Auto-enable motors |
| `gripper_exist` | `true` | Enable gripper |
| `gripper_val_mutiple` | `2` | PWM scaling multiplier |

---

#  Cartesian Control (IK)

### Launch file: `cartesian_controller.launch.py`

Starts:
- `control_manager`
- `delta_manager`
- `cartesian_controller`

Run:
```bash
ros2 launch piper_kinematics cartesian_controller.launch.py
```

### Parameters

| Parameter | Default |
|----------|---------|
| `use_analytic_ik` | false |
| `dh_type` | modified |
| `max_iterations` | 80 |
| `position_tolerance` | 1e-3 |
| `orientation_tolerance` | 1e-2 |
| `damping_factor` | 0.1 |
| `publish_rate` | 30.0 |

---

# Joystick Teleoperation

Package: `piper_teleoperation`

### Launch:
```bash
ros2 launch piper_teleoperation joy_controller.launch.py
```

### Scripts:

- `joystick_controller_node.py` – reads joystick and publishes commands  
- `piper_conrol_gamesir.py` – GameSir button mapping  
- `game_controller_manager.py` – gamepad state handling  
- `ant.py` – helper utility  

Supported controllers:
- GameSir T4 / T4 Mini  
- Xbox Controller  
- Logitech F710  

---

# 📷 Depth Camera Node

`depth_camera.py` processes depth images or point clouds.

Run:
```bash
ros2 run piper depth_camera
```

---

# 🧪 Quick Tests

Move a single joint:
```bash
ros2 topic pub /joint_ctrl_single std_msgs/Float64MultiArray "{data: [0.5, 0, 0, 0, 0, 0]}"
```

Send Cartesian velocity:
```bash
ros2 topic pub /cartesian_delta geometry_msgs/msg/Twist "{linear: {x:0.1, y:0, z:0}}"
```

---

# 📝 To‑Do

- Document `object_recognition/`
- MoveIt 2 integration guide
- Gripper calibration tutorial
- Example: teleoperation + vision pipeline

---

# 👤 Author
Developed as part of **ROBOTUM** for AgileX Piper robotic arm research.

