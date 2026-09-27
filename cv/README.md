# Computer Vision Folder Overview

The `cv` folder contains packages responsible for the robot’s perception capabilities using camera and vision sensors. These packages process visual data to extract information about the environment, such as lane markings and obstacles.

## Structure

- **lane_detection/**: Implements algorithms for detecting lane markings and other relevant features from camera images. This package subscribes to camera topics, processes images in real time, and publishes lane information for use by navigation and control systems.

<<<<<<< Updated upstream
In timbot orchestrator configs (sim/comp), the lane detection stage key is `lane_detection`.

## Creating Packages
=======
## Integration Role
>>>>>>> Stashed changes

- The computer vision packages provide processed perception data to the navigation and control subsystems.
- Lane detection outputs are used by the navigation stack for path planning and by control algorithms for lane keeping.
- The vision system is designed to be modular, allowing for the addition of further perception capabilities as needed.
- Vision outputs may be visualized in RViz or logged for offline analysis.

## How It Works

1. **Image Acquisition**: The package subscribes to camera topics and receives image frames in real time.
2. **Feature Extraction**: Lane detection algorithms process the images to identify lane boundaries and other relevant features.
3. **Data Publishing**: Detected features are published on ROS2 topics for consumption by navigation and control nodes.

## Usage

- Launch the vision packages individually or as part of a system-wide launch file.
- Configure camera parameters and algorithm settings via configuration files.

## Relationships

- Provides perception data to the `nav` folder for path planning and control.
- May use robot model information from the `description` folder for camera calibration.
- Can be integrated with simulation environments in `gazebo_worlds` for testing.
- Image processing pipelines
- Object detection and tracking
- AprilTag/ArUco marker detection
- Visual servoing
