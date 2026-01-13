# Lane Detection Debugging Scripts

This directory contains standalone testing and debugging scripts for the `lane_detection` package. These scripts allow you to test individual components without requiring the full ROS system or GPU hardware.

## Scripts Overview

| Script | Purpose | ROS Required | GPU Required |
|--------|---------|--------------|--------------|
| `test_threshold_standalone.py` | Test classical threshold-based detection | No | No |
| `test_model_standalone.py` | Test YOLO/UNet deep learning models | No | Optional (`--cpu`) |
| `test_line_fitting_standalone.py` | Test spline curve fitting | No | No |
| `fake_image_publisher.py` | Publish fake images for ROS testing | Yes | No |
| `visualize_output.py` | Visualize lane detection ROS outputs | Yes | No |

## Quick Start

### 1. Test Threshold Detection (No ROS, No GPU)

```bash
cd /path/to/lane_detection/debugging_scripts

# Run with synthetic images
python3 test_threshold_standalone.py

# Run with a real image
python3 test_threshold_standalone.py --image /path/to/image.jpg

# Run with a directory of images
python3 test_threshold_standalone.py --dir /path/to/images/

# Save outputs without display
python3 test_threshold_standalone.py --save --output-dir ./results --no-display
```

### 2. Test Deep Learning Models (CPU Mode)

```bash
# Test YOLOv8 on CPU (no GPU required)
python3 test_model_standalone.py --model yolo --cpu

# Test UNet on CPU
python3 test_model_standalone.py --model unet --cpu

# Test with specific image
python3 test_model_standalone.py --model yolo --cpu --image /path/to/image.jpg

# Specify custom model weights
python3 test_model_standalone.py --model yolo --cpu --weights /path/to/model.pt
```

### 3. Test Line Fitting

```bash
# Run with synthetic lane masks
python3 test_line_fitting_standalone.py

# Test with curved lanes
python3 test_line_fitting_standalone.py --curved

# Test with a mask image
python3 test_line_fitting_standalone.py --mask /path/to/mask.png
```

### 4. Full ROS Pipeline Testing

```bash
# Terminal 1: Start fake image publisher
ros2 run lane_detection fake_image_publisher --ros-args -p mode:=synthetic

# Terminal 2: Start lane detection node
ros2 launch lane_detection launch.py

# Terminal 3: Visualize outputs
ros2 run lane_detection visualize_output
```

## CPU-Only Testing

To run deep learning models without a GPU:

1. **Use the `--cpu` flag:**
   ```bash
   python3 test_model_standalone.py --model yolo --cpu
   ```

2. **Set environment variable (alternative):**
   ```bash
   CUDA_VISIBLE_DEVICES="" python3 test_model_standalone.py --model yolo
   ```

**Note:** CPU inference will be significantly slower than GPU (typically 5-10x), but allows testing on machines without NVIDIA GPUs.

## Fake Image Publisher Modes

The `fake_image_publisher.py` script supports multiple modes:

| Mode | Description |
|------|-------------|
| `synthetic` | Generates synthetic road images with lane markings |
| `file` | Loads images from a directory |
| `video` | Reads frames from a video file |
| `solid` | Publishes solid colored images (baseline testing) |

Examples:
```bash
# Synthetic mode with orange barrel (tests barrel filtering)
ros2 run lane_detection fake_image_publisher --ros-args \
    -p mode:=synthetic \
    -p add_barrel:=true

# Load from directory
ros2 run lane_detection fake_image_publisher --ros-args \
    -p mode:=file \
    -p image_dir:=/path/to/test/images

# Load from video
ros2 run lane_detection fake_image_publisher --ros-args \
    -p mode:=video \
    -p video_path:=/path/to/video.mp4
```

## Expected Output Locations

- Threshold test results: `./threshold_test_results/`
- Model test results: `./model_test_results/`
- Line fitting results: `./line_fitting_results/`

## Dependencies

**For standalone scripts (no ROS):**
- numpy
- opencv-python (`cv2`)
- scikit-learn (for `test_line_fitting_standalone.py`)

**For deep learning testing:**
- torch
- ultralytics (for YOLO)

**For ROS scripts:**
- rclpy
- cv_bridge
- sensor_msgs

## Troubleshooting

### "Model weights not found"
Ensure model files are in `/lane_detection/models/`:
- `best_model_int8.pt` for YOLO
- `unet.pt` for UNet (if using UNet)

### "CUDA not available" warning
This is normal when using `--cpu` flag. The model will run on CPU.

### "ModuleNotFoundError: threshold_lane"
Run from the `debugging_scripts` directory, or add the `src` directory to your Python path:
```bash
export PYTHONPATH=$PYTHONPATH:/path/to/lane_detection/src
```

### OpenCV display issues (headless server)
Use `--no-display --save` to save outputs without GUI:
```bash
python3 test_threshold_standalone.py --no-display --save
```
