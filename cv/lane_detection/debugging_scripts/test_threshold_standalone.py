#!/usr/bin/env python3
"""
Standalone Threshold Lane Detection Tester

Purpose:
    Test the threshold-based lane detection (classical mode) without ROS,
    without a GPU, and without camera hardware.

Features:
    - Runs entirely on CPU
    - No ROS dependencies
    - Visual output with OpenCV
    - Tests all threshold functions: white lanes, orange barrel filtering

Usage:
    # Test with synthetic image
    python3 test_threshold_standalone.py

    # Test with a specific image file
    python3 test_threshold_standalone.py --image /path/to/image.jpg

    # Test with a directory of images
    python3 test_threshold_standalone.py --dir /path/to/images

    # Save outputs instead of displaying
    python3 test_threshold_standalone.py --save --output-dir ./results
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
sys.path.insert(0, SRC_DIR)

from threshold_lane.threshold import (
    lane_detection,
    create_mask,
    create_orange_mask,
    get_mask,
    clean_barrels,
    rm_barrel
)


def generate_synthetic_test_image(width=640, height=360, add_barrel=False):
    """Generate a synthetic test image with lane markings."""
    # Create dark gray road
    img = np.full((height, width, 3), 80, dtype=np.uint8)
    
    # Add road texture noise
    noise = np.random.randint(-5, 5, (height, width, 3), dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    
    # Add white lane lines
    lane_width = 15
    
    # Left lane
    pts_left = np.array([
        [width // 4 - lane_width, height],
        [width // 4 + lane_width, height],
        [width // 2 - 20, height // 2],
        [width // 2 - 40, height // 2]
    ], np.int32)
    cv2.fillPoly(img, [pts_left], (255, 255, 255))
    
    # Right lane
    pts_right = np.array([
        [3 * width // 4 - lane_width, height],
        [3 * width // 4 + lane_width, height],
        [width // 2 + 40, height // 2],
        [width // 2 + 20, height // 2]
    ], np.int32)
    cv2.fillPoly(img, [pts_right], (255, 255, 255))
    
    # Add sky (to test top region masking)
    for y in range(height // 3):
        ratio = y / (height // 3)
        sky_color = (200 + int(30 * ratio), 150 + int(50 * ratio), 100 + int(50 * ratio))
        img[y, :] = sky_color
    
    # Optionally add orange barrel
    if add_barrel:
        barrel_x = width // 3
        barrel_y = int(height * 0.55)
        barrel_w = 40
        barrel_h = 60
        # Orange barrel
        cv2.rectangle(img, 
                     (barrel_x, barrel_y),
                     (barrel_x + barrel_w, barrel_y + barrel_h),
                     (0, 140, 255), -1)  # BGR orange
        # White stripes
        for stripe_y in range(barrel_y, barrel_y + barrel_h, 15):
            cv2.rectangle(img,
                         (barrel_x, stripe_y),
                         (barrel_x + barrel_w, stripe_y + 7),
                         (255, 255, 255), -1)
    
    return img


def test_single_image(img, display=True, save_path=None):
    """
    Run all threshold tests on a single image.
    
    Returns dict with timing and results.
    """
    results = {}
    
    # Ensure correct size
    img_resized = cv2.resize(img, (330, 180))
    
    # Convert to HSV for mask creation
    img_hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
    
    print("\n" + "=" * 60)
    print("Testing Threshold Lane Detection")
    print("=" * 60)
    
    # Test 1: White mask
    print("\n[1] Testing white lane mask...")
    t1 = time.perf_counter()
    white_mask = create_mask(img_hsv)
    t2 = time.perf_counter()
    results['white_mask_time'] = (t2 - t1) * 1000
    print(f"    Time: {results['white_mask_time']:.2f} ms")
    print(f"    White pixels detected: {np.sum(white_mask > 0)}")
    
    # Test 2: Orange mask
    print("\n[2] Testing orange barrel mask...")
    t1 = time.perf_counter()
    orange_mask = create_orange_mask(img_hsv)
    t2 = time.perf_counter()
    results['orange_mask_time'] = (t2 - t1) * 1000
    print(f"    Time: {results['orange_mask_time']:.2f} ms")
    print(f"    Orange pixels detected: {np.sum(orange_mask > 0)}")
    
    # Test 3: Full lane detection pipeline
    print("\n[3] Testing full lane_detection() pipeline...")
    t1 = time.perf_counter()
    output = lane_detection(img)
    t2 = time.perf_counter()
    results['full_pipeline_time'] = (t2 - t1) * 1000
    print(f"    Time: {results['full_pipeline_time']:.2f} ms")
    print(f"    Lane pixels detected: {np.sum(output > 0)}")
    
    # Calculate FPS equivalent
    fps = 1000 / results['full_pipeline_time'] if results['full_pipeline_time'] > 0 else 0
    print(f"    Equivalent FPS: {fps:.1f}")
    
    # Visualization
    if display or save_path:
        # Create visualization grid
        vis_height = 360
        vis_width = 660
        
        # Resize outputs for display
        img_display = cv2.resize(img, (330, 180))
        white_display = cv2.cvtColor(cv2.resize(white_mask, (330, 180)), cv2.COLOR_GRAY2BGR)
        orange_display = cv2.cvtColor(cv2.resize(orange_mask, (330, 180)), cv2.COLOR_GRAY2BGR)
        orange_display[:, :, 0] = 0  # Make it orange-tinted
        orange_display[:, :, 2] = np.clip(orange_display[:, :, 2] * 2, 0, 255).astype(np.uint8)
        
        output_display = cv2.cvtColor(cv2.resize(output.astype(np.uint8), (330, 180)), cv2.COLOR_GRAY2BGR)
        
        # Create overlay
        overlay = img_display.copy()
        lane_mask = cv2.resize(output.astype(np.uint8), (330, 180)) > 0
        overlay[lane_mask, 2] = 255  # Red overlay on lanes
        overlay[lane_mask, 0] = 0
        overlay[lane_mask, 1] = 0
        
        # Build grid
        grid = np.zeros((vis_height, vis_width, 3), dtype=np.uint8)
        grid[0:180, 0:330] = img_display
        grid[0:180, 330:660] = white_display
        grid[180:360, 0:330] = output_display
        grid[180:360, 330:660] = overlay
        
        # Add labels
        cv2.putText(grid, 'Input', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(grid, 'White Mask', (340, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(grid, 'Lane Detection', (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(grid, 'Overlay', (340, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(grid, f'{fps:.1f} FPS', (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if save_path:
            cv2.imwrite(save_path, grid)
            print(f"\n    Saved visualization to: {save_path}")
        
        if display:
            cv2.imshow('Threshold Lane Detection Test', grid)
            print("\n    Press any key to continue, 'q' to quit...")
            key = cv2.waitKey(0)
            if key == ord('q'):
                return results, True  # Signal to quit
    
    return results, False


def main():
    parser = argparse.ArgumentParser(description='Test threshold lane detection standalone')
    parser.add_argument('--image', type=str, help='Path to a single test image')
    parser.add_argument('--dir', type=str, help='Path to directory of test images')
    parser.add_argument('--save', action='store_true', help='Save output instead of displaying')
    parser.add_argument('--output-dir', type=str, default='./threshold_test_results',
                        help='Directory to save results')
    parser.add_argument('--with-barrel', action='store_true', 
                        help='Add orange barrel to synthetic test image')
    parser.add_argument('--no-display', action='store_true', help='Disable display')
    args = parser.parse_args()
    
    display = not args.no_display and not args.save
    
    if args.save:
        os.makedirs(args.output_dir, exist_ok=True)
    
    all_results = []
    
    if args.image:
        # Test single image
        if not os.path.isfile(args.image):
            print(f"Error: Image not found: {args.image}")
            return 1
        img = cv2.imread(args.image)
        save_path = os.path.join(args.output_dir, 'result.png') if args.save else None
        results, _ = test_single_image(img, display=display, save_path=save_path)
        all_results.append(results)
        
    elif args.dir:
        # Test directory of images
        if not os.path.isdir(args.dir):
            print(f"Error: Directory not found: {args.dir}")
            return 1
        
        patterns = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
        images = []
        for pattern in patterns:
            images.extend(glob.glob(os.path.join(args.dir, pattern)))
        images.sort()
        
        print(f"Found {len(images)} images in {args.dir}")
        
        for i, img_path in enumerate(images):
            print(f"\n[{i+1}/{len(images)}] Processing: {os.path.basename(img_path)}")
            img = cv2.imread(img_path)
            if img is None:
                print(f"  Warning: Could not read {img_path}")
                continue
            
            save_path = None
            if args.save:
                save_path = os.path.join(args.output_dir, f'result_{i:04d}.png')
            
            results, quit_requested = test_single_image(img, display=display, save_path=save_path)
            all_results.append(results)
            
            if quit_requested:
                break
    
    else:
        # Generate and test synthetic images
        print("No input specified, using synthetic test images...")
        
        # Test without barrel
        print("\n>>> Testing synthetic image WITHOUT barrel <<<")
        img = generate_synthetic_test_image(add_barrel=False)
        save_path = os.path.join(args.output_dir, 'synthetic_no_barrel.png') if args.save else None
        results, quit_requested = test_single_image(img, display=display, save_path=save_path)
        all_results.append(results)
        
        if not quit_requested:
            # Test with barrel
            print("\n>>> Testing synthetic image WITH barrel <<<")
            img = generate_synthetic_test_image(add_barrel=True)
            save_path = os.path.join(args.output_dir, 'synthetic_with_barrel.png') if args.save else None
            results, _ = test_single_image(img, display=display, save_path=save_path)
            all_results.append(results)
    
    # Print summary
    if all_results:
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        avg_time = np.mean([r['full_pipeline_time'] for r in all_results])
        avg_fps = 1000 / avg_time if avg_time > 0 else 0
        print(f"Average pipeline time: {avg_time:.2f} ms")
        print(f"Average FPS: {avg_fps:.1f}")
        print(f"Total images tested: {len(all_results)}")
    
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
