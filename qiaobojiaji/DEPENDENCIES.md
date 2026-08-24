# Dependencies

## Included in this folder

- Python dependency manifests: `requirements.txt`, `requirements-vision.txt`
- YOLO model: `platform_a/models/heel_seg.pt`
- Hand-eye, TCP, gripper and task correction JSON files
- Platform-A source and runtime scripts

## Must be installed on the target machine

1. Ubuntu 22.04 or WSL2
2. ROS 2 Humble, including `rclpy`, `sensor_msgs`, `cv_bridge` and standard ROS libraries
3. Intel RealSense ROS 2 driver with aligned depth enabled
4. FR5 `fairino` Python SDK matching the robot controller
5. Python 3.10+
6. Access to the AG95 serial device and permission for its `/dev/serial/by-id/...` path

## Install Python packages

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-vision.txt
```

`fairino` is vendor-specific and is intentionally not redistributed in this package. Install it from the FR5 SDK supplied with the target robot.

ROS and the RealSense driver are system packages and are intentionally not copied into this project folder.
