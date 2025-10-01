# sitl_manus

A ROS package for hand tracking and gesture recognition using Manus gloves and VIVE trackers for robotic surgery applications.


## Publications

by  Borgioli, L., Oh, K-H., Valle, V., Ducas, A., Halloum, M., Mendoza Medina, D. F., Sharifi, A., López, P. A., Cassiani, J., Žefran, M., *et al.* (ICRA 2025)

[![arXiv](https://img.shields.io/badge/arXiv-Paper-red?logo=arxiv)](https://arxiv.org/pdf/2403.13941)  
## Overview

This package provides real-time hand tracking capabilities by integrating:
- **Manus gloves** for finger joint tracking and gesture recognition
- **VIVE trackers** for 6DOF hand position and orientation tracking
- **Point cloud workspace** visualization for surgical robot interaction
- **ROS integration** for seamless robotic system communication

## Features

- **Multi-hand tracking**: Support for both left and right hand tracking
- **Gesture recognition**: Machine learning-based gesture classification (fist, pinch, etc.)
- **Workspace mapping**: Real-time mapping of hand workspace to robot workspace
- **Clutch functionality**: Hand tracking enable/disable control
- **Point cloud visualization**: 3D workspace representation in RVIZ

## Package Structure

```
├── scripts/                    # Main Python scripts
│   ├── gesture_recog*.py      # Gesture recognition nodes
│   ├── gloves_reading*.py     # Manus glove data processing
│   ├── node_tracker*.py       # VIVE tracker processing
│   ├── pcl_workspace*.py      # Point cloud workspace generation
│   ├── processing_gloves*.py  # Glove data preprocessing
│   └── recording*.py          # Data recording utilities
├── src/                       # Source code
│   ├── nodes/                 # Additional ROS nodes
│   └── tracker/               # VIVE tracker utilities
├── launch/                    # Launch files
├── msg/                       # Custom message definitions
├── rviz/                      # RVIZ configuration
└── README.md
```

## Dependencies

- ROS (tested with ROS Melodic/Noetic)
- Python 3
- OpenVR/SteamVR for VIVE tracker support
- Manus Core SDK
- Required Python packages:
  - `numpy`
  - `pandas`
  - `scipy`
  - `pyquaternion`
  - `scikit-learn`
  - `joblib`

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <repository_url> sitl_manus
```

2. Install dependencies:
```bash
rosdep install --from-paths . --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Basic Launch
Launch the complete hand tracking system:
```bash
roslaunch sitl_manus glove.launch
```

### Individual Components

1. **Hand tracking only**:
```bash
rosrun sitl_manus node_tracker.py          # Left hand
rosrun sitl_manus node_tracker_right.py    # Right hand
```

2. **Gesture recognition**:
```bash
rosrun sitl_manus gesture_recog.py         # Left hand gestures
rosrun sitl_manus gesture_recog_right.py   # Right hand gestures
```

3. **Workspace visualization**:
```bash
rosrun sitl_manus pcl_workspace_v2.py      # Left hand workspace
rosrun sitl_manus pcl_workspace_v2_right.py # Right hand workspace
```

### Key Controls
- **'s'**: Enable hand tracking
- **'e'**: Disable hand tracking

## Topics

### Published Topics
- `/tracker_current_pos_tf` - Current tracker pose
- `/manus/pcl` - Point cloud from left hand
- `/manus/pcl_right` - Point cloud from right hand  
- `/glove/left/fist` - Fist gesture detection
- `/cube_point_cloud` - Workspace point cloud

### Subscribed Topics
- `/manus/cp` - Left hand glove data
- `/manus/cp_right` - Right hand glove data
- `/glove/left/on_off` - Hand tracking enable/disable

## Configuration

### Tracker Setup
1. Ensure VIVE trackers are properly calibrated
2. Update tracker device IDs in the node tracker scripts:
   - `tracker_1` for left hand
   - `tracker_2` for right hand

### Workspace Calibration
Modify workspace boundaries in [`pcl_workspace_v2.py`](scripts/pcl_workspace_v2.py):
```python
size_hand_workspace = 0.25  # Adjust workspace size
```

## Visualization

Launch RVIZ with the provided configuration:
```bash
rviz -d rviz/glove.rviz
```

This will show:
- Hand tracker poses
- Workspace point clouds
- Robot coordinate frames

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
If you find this repository useful please cite our work!

> **Sensory Glove-Based Surgical Robot User Interface**  
> Borgioli, L., Oh, K-H., Valle, V., Ducas, A., Halloum, M., Mendoza Medina, D. F., Sharifi, A., López, P. A., Cassiani, J., Žefran, M., *et al.* (ICRA 2025)

## Contact

For questions or support, please contact the SITL lab.
