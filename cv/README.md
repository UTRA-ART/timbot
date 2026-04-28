# Computer Vision Packages

This directory contains ROS2 packages related to computer vision and image processing.

## Purpose

Computer vision packages handle camera interfaces, image processing, object detection, and visual perception tasks.

In timbot orchestrator configs (sim/comp), the lane detection stage key is `lane_detection`.

## Creating Packages

To create a new computer vision package:

```bash
cd cv
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy sensor_msgs cv_bridge
```

## Common Package Types

- Camera drivers
- Image processing pipelines
- Object detection and tracking
- AprilTag/ArUco marker detection
- Visual servoing
