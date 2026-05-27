<div align="center">

<h1>🐕 Spot Fetch and Carry 📦</h1>
<h3>Autonomous Manipulation and Navigation using Behavior Trees</h3>

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Boston Dynamics](https://img.shields.io/badge/Boston_Dynamics-000000?style=for-the-badge&logo=robotics&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLO-00FFFF?style=for-the-badge&logo=opencv&logoColor=black)

*Orchestrating autonomous fetch-and-carry missions for the Boston Dynamics Spot robot utilizing ROS 2, py_trees, and computer vision.*

[📌 Features](#-features) • [🧠 Architecture](#-system-architecture) • [📁 Structure](#-project-structure) • [🚀 Usage](#-usage-guide)

</div>

---

## 🏛️ Project Overview
This repository contains a complete ROS 2 package designed to govern the autonomous behavior of the Boston Dynamics **Spot** robot for fetch-and-carry tasks. The core decision-making and mission execution are driven by a robust **Behavior Tree** architecture using the `py_trees` library. The agent is capable of safely navigating environments, detecting target objects via integrated perception models, and coordinating Spot's robotic arm to retrieve and transport items.

---

## 📌 Features
- ✅ **Behavior Tree Orchestration**: Complex mission logic managed via `py_trees`, combining sequences, selectors, and parallel execution for reactive robotics.
- ✅ **Parallel Perception & Action**: Continuous object detection runs concurrently with robot mobility (MovingScan) and manipulation scanning (ManipulatorScan) using `SuccessOnAll` parallel policies.
- ✅ **Advanced Safety Mechanisms**: Custom pre-tick handlers constantly monitor a `status_blackboard`. If a critical failure is detected, an emergency protocol forces an immediate, safe physical shutdown.
- ✅ **Object Detection Integration**: Seamless integration with YOLO models (`yolo26n.pt`) and custom interfaces (`Detection.srv`) for recognizing target items like tennis balls.
- ✅ **Graceful Teardown**: Aggressive exception catching and system signal (SIGINT) interception to guarantee the robot always powers down safely upon interruption.

---

## 📁 Project Structure
```text
├── 📁 bt_interfaces/               # Custom ROS 2 interfaces
│   ├── 📁 srv/
│   │   └── Detection.srv           # Object detection service definitions
│   ├── CMakeLists.txt
│   └── package.xml
├── 📁 spot_bt/                     # Main Behavior Tree workspace
│   ├── 📁 spot_bt/
│   │   ├── 📁 actions/             # BT Action Nodes (arm, body, perception, general)
│   │   ├── 📁 conditions/          # BT Condition Nodes for state checking
│   │   ├── 📁 composites/          # Custom composite nodes (Selectors, Sequences)
│   │   ├── 📁 ros_nodes/           # ROS 2 node wrappers (Init, Detection, Navigation)
│   │   ├── 📁 assets/              # PyTorch models (.pt) and JSON configurations
│   │   ├── 📁 utils/               # Blackboards, handlers, constants, and tree builders
│   │   └── script.py               # Main entry point for the behavior tree
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── LICENSE
└── README.md
```

---

## 🧠 System Architecture

The behavior of the robot is entirely decoupled from the low-level hardware control through the use of **ROS 2 nodes** and a centralized **Blackboard** data-sharing system.

* **Initialization Node**: When the system starts, `InitializationNode` handles the physical lease, power-on, and un-stowing of the Spot robot.
* **Behavior Tree Root (Parallel)**: The main execution loop ticks a root `Parallel` composite. This simultaneously manages continuous environmental scanning (`MovingScan`, `ManipulatorScan`), real-time `ObjectDetection`, and the sequential `mission_subtree`.
* **Blackboard Architecture**: Sensor data, mission parameters, and critical failure states are shared across the tree branches using `py_trees.blackboard`. This allows condition nodes to safely abort actions if the environment changes or if hardware errors arise.

---

## 🔧 Installation & Setup

### Prerequisites
- **ROS 2** (Humble/Iron depending on your setup)
- **Python 3.10+**
- **py_trees** and **py_trees_ros**
- **Boston Dynamics Spot SDK**

### Build the Workspace
Clone the repository into your ROS 2 workspace and build the packages:

```bash
# Navigate to your workspace src folder
cd ~/ros2_ws/src
git clone [https://github.com/chiar15/spot-fetch-and-carry.git](https://github.com/chiar15/spot-fetch-and-carry.git)

# Navigate back to the workspace root and build
cd ~/ros2_ws
colcon build --packages-select bt_interfaces spot_bt

# Source the overlay
source install/setup.bash
```

---

## 🚀 Usage Guide

The main entry point for the mission is located in `script.py`. It handles all setups, starts the ROS 2 executor, and ticks the Behavior Tree.

```bash
# Run the main behavior tree mission
ros2 run spot_bt script --robot spot
```

**Arguments:**
* `--robot`: Specifies the robot namespace (e.g., `'spot'`). If omitted, no namespace is used.

### Safe Shutdown
The system is built with safety in mind. At any point during the mission, pressing `CTRL+C` will be intercepted by the signal handler. This will instantly halt the behavior tree, trigger the `safe_shutdown()` routine to safely stow the robot, and clear the blackboard to prevent data corruption.

---

## 🔗 Related Resources
* [Boston Dynamics Spot SDK Documentation](https://dev.bostondynamics.com/)
* [ROS 2 Documentation](https://docs.ros.org/)
* [py_trees Official Documentation](https://py-trees.readthedocs.io/)
