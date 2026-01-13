#!/usr/bin/env python3
"""
Standalone YOLO/UNet Lane Detection Tester (CPU Mode)

Purpose:
    Test deep learning lane detection models without ROS and with
    optional CPU-only mode (no GPU required).

Features:
    - Supports both YOLOv8 and UNet models
    - CPU-only mode with --cpu flag
    - No ROS dependencies for standalone testing
    - Visual output with OpenCV
    - Performance benchmarking

Usage:
    # Test YOLOv8 model on CPU
    python3 test_model_standalone.py --model yolo --cpu

    # Test UNet model on GPU
    python3 test_model_standalone.py --model unet

    # Test with specific image
    python3 test_model_standalone.py --model yolo --image /path/to/image.jpg --cpu

    # Test with directory of images
    python3 test_model_standalone.py --model yolo --dir /path/to/images --cpu

    # Specify model path
    python3 test_model_standalone.py --model yolo --weights /path/to/model.pt --cpu

Environment Variables:
    CUDA_VISIBLE_DEVICES="" - Set to empty to force CPU mode at system level
"""

import argparse
import os
import sys
import glob
import time
import numpy as np
import cv2

# Add parent src directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), 'src')
MODELS_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), 'models')
sys.path.insert(0, SRC_DIR)


def setup_cpu_mode():
    """Force CPU mode by setting environment variables before importing torch."""
    os.environ['CUDA_VISIBLE_DEVICES'] = ''
    # Some additional flags that might help
    os.environ['FORCE_CPU'] = '1'


def generate_synthetic_test_image(width=640, height=360):
    """Generate a synthetic test image with lane markings."""
    # Create dark gray road
    img = np.full((height, width, 3), 80, dtype=np.uint8)
    
    # Add road texture noise
    noise = np.random.randint(-5, 5, (height, width, 3), dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    
    # Add white lane lines with perspective
    lane_width = 15
    horizon_y = height // 3
    
    # Left lane
    pts_left = np.array([
        [width // 4 - lane_width, height],
        [width // 4 + lane_width, height],
        [width // 2 - 30, horizon_y + 50],
        [width // 2 - 50, horizon_y + 50]
    ], np.int32)
    cv2.fillPoly(img, [pts_left], (255, 255, 255))
    
    # Right lane
    pts_right = np.array([
        [3 * width // 4 - lane_width, height],
        [3 * width // 4 + lane_width, height],
        [width // 2 + 50, horizon_y + 50],
        [width // 2 + 30, horizon_y + 50]
    ], np.int32)
    cv2.fillPoly(img, [pts_right], (255, 255, 255))
    
    # Add sky
    for y in range(horizon_y):
        ratio = y / horizon_y
        sky_color = (200 + int(30 * ratio), 150 + int(50 * ratio), 100 + int(50 * ratio))
        img[y, :] = sky_color
    
    return img


class YOLOTester:
    """Test wrapper for YOLOv8 model."""
    
    def __init__(self, weights_path, use_cpu=False):
        # Import here to allow CPU setup first
        import torch
        from ultralytics import YOLO
        
        self.use_cpu = use_cpu
        self.device = 'cpu' if use_cpu else ('cuda' if torch.cuda.is_available() else 'cpu')
        
        print(f"Loading YOLOv8 model from: {weights_path}")
        print(f"Using device: {self.device}")
        
        self.model = YOLO(weights_path)
        
        if use_cpu:
            # Force CPU mode
            self.model.to('cpu')
        
        print("Model loaded successfully!")
    
    def predict(self, img, conf_threshold=0.5):
        """Run inference on an image."""
        import torch
        
        # Resize to expected input size
        img_resized = cv2.resize(img, (330, 180))
        
        # Run inference
        results = self.model(img_resized, device=self.device, verbose=False)
        
        # Process output
        output_image = np.zeros((180, 330), dtype=np.uint8)
        
        if results and results[0].masks:
            for k in range(len(results[0].masks)):
                mask = np.array(
                    results[0].masks[k].data.cpu() if not self.use_cpu else results[0].masks[k].data
                )
                label = results[0].names[int(results[0].boxes[k].cls)]
                conf = float(results[0].boxes[k].conf)
                
                if conf > conf_threshold and label == 'lane':
                    lane_mask = np.where(mask > 0.5, 255, 0).astype(np.uint8)
                    lane_mask = cv2.resize(lane_mask.squeeze(), (330, 180))
                    output_image = np.maximum(output_image, lane_mask)
        
        return output_image


class UNetTester:
    """Test wrapper for UNet model."""
    
    def __init__(self, weights_path, use_cpu=False):
        # Import here to allow CPU setup first
        import torch
        from unet_lane.UNet import UNet
        
        self.use_cpu = use_cpu
        self.device = 'cpu' if use_cpu else ('cuda' if torch.cuda.is_available() else 'cpu')
        
        print(f"Loading UNet model from: {weights_path}")
        print(f"Using device: {self.device}")
        
        self.model = UNet()
        self.model.load_state_dict(
            torch.load(weights_path, map_location=torch.device(self.device))
        )
        self.model.to(self.device)
        self.model.eval()
        
        print("Model loaded successfully!")
    
    def preprocess(self, img):
        """Preprocess image for UNet (4 channels: gray, edges, edges_inv, gradient)."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Adaptive Canny edge detection
        height, width = gray.shape
        edges_mask = np.zeros_like(gray)
        
        # Process quadrants
        for i, (y_slice, x_slice) in enumerate([
            (slice(0, height//2), slice(0, width//2)),
            (slice(0, height//2), slice(width//2, width)),
            (slice(height//2, height), slice(width//2, width)),
            (slice(height//2, height), slice(0, width//2))
        ]):
            region = gray[y_slice, x_slice]
            med = np.median(region)
            low = int(max(0, (1 - 0.205) * med))
            high = int(min(255, (1 + 0.205) * med))
            edges_mask[y_slice, x_slice] = cv2.Canny(region, low, high)
        
        edges_inv = cv2.bitwise_not(edges_mask)
        gradient = np.uint8(np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=-1)))
        
        # Stack channels
        output = np.zeros((gray.shape[0], gray.shape[1], 4), dtype=np.uint8)
        output[:, :, 0] = gray
        output[:, :, 1] = edges_mask
        output[:, :, 2] = edges_inv
        output[:, :, 3] = gradient
        
        return output
    
    def predict(self, img, threshold=0.5):
        """Run inference on an image."""
        import torch
        
        # Resize and preprocess
        img_resized = cv2.resize(img, (256, 160))
        preprocessed = self.preprocess(img_resized)
        
        # Convert to tensor
        input_tensor = torch.Tensor((preprocessed / 255.0).transpose(2, 0, 1))
        input_tensor = input_tensor.unsqueeze(0).to(self.device)
        
        # Run inference
        with torch.no_grad():
            output = self.model(input_tensor)[0][0]
            output = torch.sigmoid(output)
            output = output.cpu().numpy()
        
        # Threshold
        pred_mask = np.where(output > threshold, 255, 0).astype(np.uint8)
        
        # Resize to standard output size
        pred_mask = cv2.resize(pred_mask, (330, 180))
        
        return pred_mask


def test_model(tester, img, display=True, save_path=None):
    """Test model on a single image."""
    results = {}
    
    print("\n" + "-" * 40)
    
    # Run inference with timing
    t1 = time.perf_counter()
    output = tester.predict(img)
    t2 = time.perf_counter()
    
    inference_time = (t2 - t1) * 1000
    fps = 1000 / inference_time if inference_time > 0 else 0
    
    results['inference_time'] = inference_time
    results['fps'] = fps
    results['lane_pixels'] = np.sum(output > 0)
    
    print(f"Inference time: {inference_time:.2f} ms")
    print(f"Equivalent FPS: {fps:.1f}")
    print(f"Lane pixels detected: {results['lane_pixels']}")
    
    # Visualization
    if display or save_path:
        img_display = cv2.resize(img, (330, 180))
        output_display = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        
        # Create overlay
        overlay = img_display.copy()
        lane_mask = output > 0
        overlay[lane_mask, 2] = 255
        overlay[lane_mask, 0] = 0
        overlay[lane_mask, 1] = 0
        
        # Blend overlay
        blended = cv2.addWeighted(img_display, 0.6, overlay, 0.4, 0)
        
        # Build visualization
        vis = np.hstack([img_display, output_display, blended])
        
        # Add labels
        cv2.putText(vis, 'Input', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, 'Detection', (340, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, 'Overlay', (670, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, f'{fps:.1f} FPS', (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        if save_path:
            cv2.imwrite(save_path, vis)
            print(f"Saved to: {save_path}")
        
        if display:
            cv2.imshow('Model Test', vis)
            print("Press any key to continue, 'q' to quit...")
            key = cv2.waitKey(0)
            if key == ord('q'):
                return results, True
    
    return results, False


def main():
    parser = argparse.ArgumentParser(description='Test lane detection models standalone')
    parser.add_argument('--model', type=str, choices=['yolo', 'unet'], default='yolo',
                        help='Model type to test')
    parser.add_argument('--weights', type=str, help='Path to model weights')
    parser.add_argument('--cpu', action='store_true', help='Force CPU mode (no GPU)')
    parser.add_argument('--image', type=str, help='Path to test image')
    parser.add_argument('--dir', type=str, help='Path to directory of test images')
    parser.add_argument('--save', action='store_true', help='Save outputs instead of displaying')
    parser.add_argument('--output-dir', type=str, default='./model_test_results',
                        help='Directory to save results')
    parser.add_argument('--no-display', action='store_true', help='Disable display')
    parser.add_argument('--conf', type=float, default=0.5, help='Confidence threshold')
    args = parser.parse_args()
    
    # Setup CPU mode before importing torch
    if args.cpu:
        print("Forcing CPU mode...")
        setup_cpu_mode()
    
    display = not args.no_display and not args.save
    
    if args.save:
        os.makedirs(args.output_dir, exist_ok=True)
    
    # Determine weights path
    weights_path = args.weights
    if not weights_path:
        if args.model == 'yolo':
            weights_path = os.path.join(MODELS_DIR, 'best_model_int8.pt')
        else:
            # UNet - need to find a weights file
            weights_path = os.path.join(MODELS_DIR, 'unet.pt')
    
    if not os.path.isfile(weights_path):
        print(f"Error: Model weights not found at: {weights_path}")
        print("\nTo use this script, ensure you have model weights available.")
        print(f"For YOLO: Place weights at {os.path.join(MODELS_DIR, 'best_model_int8.pt')}")
        print(f"For UNet: Place weights at {os.path.join(MODELS_DIR, 'unet.pt')}")
        return 1
    
    # Create tester
    print("=" * 60)
    print(f"Lane Detection Model Test - {args.model.upper()}")
    print("=" * 60)
    
    try:
        if args.model == 'yolo':
            tester = YOLOTester(weights_path, use_cpu=args.cpu)
        else:
            tester = UNetTester(weights_path, use_cpu=args.cpu)
    except Exception as e:
        print(f"Error loading model: {e}")
        return 1
    
    all_results = []
    
    # Process images
    if args.image:
        if not os.path.isfile(args.image):
            print(f"Error: Image not found: {args.image}")
            return 1
        img = cv2.imread(args.image)
        save_path = os.path.join(args.output_dir, 'result.png') if args.save else None
        results, _ = test_model(tester, img, display=display, save_path=save_path)
        all_results.append(results)
        
    elif args.dir:
        if not os.path.isdir(args.dir):
            print(f"Error: Directory not found: {args.dir}")
            return 1
        
        patterns = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
        images = []
        for pattern in patterns:
            images.extend(glob.glob(os.path.join(args.dir, pattern)))
        images.sort()
        
        print(f"Found {len(images)} images")
        
        for i, img_path in enumerate(images):
            print(f"\n[{i+1}/{len(images)}] {os.path.basename(img_path)}")
            img = cv2.imread(img_path)
            if img is None:
                continue
            
            save_path = os.path.join(args.output_dir, f'result_{i:04d}.png') if args.save else None
            results, quit_requested = test_model(tester, img, display=display, save_path=save_path)
            all_results.append(results)
            
            if quit_requested:
                break
    else:
        # Use synthetic image
        print("\nNo input specified, using synthetic test image...")
        img = generate_synthetic_test_image()
        save_path = os.path.join(args.output_dir, 'synthetic_result.png') if args.save else None
        results, _ = test_model(tester, img, display=display, save_path=save_path)
        all_results.append(results)
    
    # Summary
    if all_results:
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        avg_time = np.mean([r['inference_time'] for r in all_results])
        avg_fps = 1000 / avg_time if avg_time > 0 else 0
        print(f"Average inference time: {avg_time:.2f} ms")
        print(f"Average FPS: {avg_fps:.1f}")
        print(f"Total images tested: {len(all_results)}")
        print(f"Device: {'CPU' if args.cpu else 'GPU/CUDA'}")
    
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
