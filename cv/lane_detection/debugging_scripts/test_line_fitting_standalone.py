#!/usr/bin/env python3
"""
Standalone Line Fitting Tester

Purpose:
    Test the line_fitting.py module's lane curve fitting algorithms
    without ROS dependencies.

Features:
    - Tests DBSCAN clustering
    - Tests spline interpolation
    - Visual output showing fitted curves
    - No GPU required

Usage:
    # Test with synthetic lane mask
    python3 test_line_fitting_standalone.py

    # Test with a mask image file
    python3 test_line_fitting_standalone.py --mask /path/to/mask.png

    # Save outputs
    python3 test_line_fitting_standalone.py --save --output-dir ./results
"""

import argparse
import os
import sys
import time
import numpy as np
import cv2

# Add parent src directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), 'src')
sys.path.insert(0, SRC_DIR)

from line_fitting import fit_lanes, lane_fitting, sort_by_cluster


def generate_synthetic_mask(width=330, height=180, num_lanes=2, curved=False):
    """Generate a synthetic lane mask for testing."""
    mask = np.zeros((height, width), dtype=np.uint8)
    
    if curved:
        # Generate curved lanes
        for lane_idx in range(num_lanes):
            x_base = width // (num_lanes + 1) * (lane_idx + 1)
            
            for y in range(height // 3, height):
                # Add some curvature
                y_ratio = (y - height // 3) / (height - height // 3)
                curve_offset = int(30 * np.sin(y_ratio * np.pi) * (0.5 - lane_idx / num_lanes))
                x = x_base + curve_offset
                
                # Draw lane point with some width
                cv2.circle(mask, (x, y), 5, 1, -1)
    else:
        # Generate straight lanes
        lane_width = 10
        
        for lane_idx in range(num_lanes):
            # Calculate lane position with perspective
            bottom_x = width // (num_lanes + 1) * (lane_idx + 1)
            top_x = width // 2 + (bottom_x - width // 2) * 0.4
            
            pts = np.array([
                [bottom_x - lane_width, height],
                [bottom_x + lane_width, height],
                [int(top_x + lane_width * 0.5), height // 3],
                [int(top_x - lane_width * 0.5), height // 3]
            ], np.int32)
            cv2.fillPoly(mask, [pts], 1)
    
    return mask


def test_line_fitting(mask, display=True, save_path=None):
    """Test line fitting on a mask."""
    results = {}
    
    print("\n" + "=" * 60)
    print("Testing Line Fitting")
    print("=" * 60)
    
    # Count input pixels
    lane_pixels = np.sum(mask > 0)
    print(f"\nInput mask: {mask.shape[0]}x{mask.shape[1]}")
    print(f"Lane pixels: {lane_pixels}")
    
    if lane_pixels == 0:
        print("Warning: No lane pixels in mask!")
        return {'error': 'empty_mask'}, False
    
    # Run line fitting with timing
    print("\nRunning fit_lanes()...")
    t1 = time.perf_counter()
    fitted_lanes = fit_lanes(mask)
    t2 = time.perf_counter()
    
    fitting_time = (t2 - t1) * 1000
    results['fitting_time'] = fitting_time
    results['num_lanes'] = len(fitted_lanes) if fitted_lanes else 0
    
    print(f"Time: {fitting_time:.2f} ms")
    print(f"Lanes detected: {results['num_lanes']}")
    
    if fitted_lanes:
        for i, lane in enumerate(fitted_lanes):
            print(f"  Lane {i+1}: {len(lane)} points")
    
    # Visualization
    if display or save_path:
        # Create visualization
        vis_height = 360
        vis_width = 660
        
        # Original mask (scaled up)
        mask_display = cv2.resize((mask * 255).astype(np.uint8), (330, 180))
        mask_display = cv2.cvtColor(mask_display, cv2.COLOR_GRAY2BGR)
        
        # Fitted lanes overlay
        fitted_display = np.zeros((180, 330, 3), dtype=np.uint8)
        
        if fitted_lanes:
            colors = [
                (0, 255, 0),    # Green
                (255, 0, 0),    # Blue
                (0, 0, 255),    # Red
                (255, 255, 0),  # Cyan
                (255, 0, 255),  # Magenta
            ]
            
            for i, lane in enumerate(fitted_lanes):
                color = colors[i % len(colors)]
                # line_fitting returns [[x, y], ...] where x=col, y=row
                # OpenCV needs (x, y) = (col, row) for drawing
                pts = np.array([[int(p[0]), int(p[1])] for p in lane], np.int32)
                if len(pts) > 1:
                    cv2.polylines(fitted_display, [pts], False, color, 2)
                    # Draw points
                    for pt in pts:
                        cv2.circle(fitted_display, tuple(pt), 3, color, -1)
        
        # Combined overlay
        combined = mask_display.copy()
        # Add fitted lanes in different color
        if fitted_lanes:
            for i, lane in enumerate(fitted_lanes):
                color = colors[i % len(colors)]
                # line_fitting returns [[x, y], ...] where x=col, y=row
                pts = np.array([[int(p[0]), int(p[1])] for p in lane], np.int32)
                if len(pts) > 1:
                    cv2.polylines(combined, [pts], False, color, 3)
        
        # Build grid
        grid = np.zeros((vis_height, vis_width, 3), dtype=np.uint8)
        grid[0:180, 0:330] = mask_display
        grid[0:180, 330:660] = fitted_display
        grid[180:360, 0:330] = combined
        
        # Add DBSCAN clustering visualization
        cluster_display = np.zeros((180, 330, 3), dtype=np.uint8)
        
        # Re-run clustering for visualization
        smoothed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, 
                                    cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)))
        rows = np.where(smoothed == 1)[0].reshape(-1, 1)
        cols = np.where(smoothed == 1)[1].reshape(-1, 1)
        coords = np.concatenate((rows, cols), axis=1)
        
        if len(coords) > 0:
            from sklearn.cluster import DBSCAN
            clustering = DBSCAN(eps=9, min_samples=35).fit(coords)
            labels = clustering.labels_
            
            unique_labels = set(labels)
            for label in unique_labels:
                if label == -1:
                    color = (128, 128, 128)  # Gray for noise
                else:
                    color = colors[label % len(colors)]
                
                mask_points = coords[labels == label]
                for pt in mask_points:
                    cv2.circle(cluster_display, (int(pt[1]), int(pt[0])), 1, color, -1)
        
        grid[180:360, 330:660] = cluster_display
        
        # Add labels
        cv2.putText(grid, 'Input Mask', (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(grid, 'Fitted Splines', (340, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(grid, 'Combined', (10, 205), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(grid, 'DBSCAN Clusters', (340, 205), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(grid, f'{fitting_time:.1f} ms | {results["num_lanes"]} lanes', 
                   (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        if save_path:
            cv2.imwrite(save_path, grid)
            print(f"\nSaved visualization to: {save_path}")
        
        if display:
            cv2.imshow('Line Fitting Test', grid)
            print("\nPress any key to continue, 'q' to quit...")
            key = cv2.waitKey(0)
            if key == ord('q'):
                return results, True
    
    return results, False


def main():
    parser = argparse.ArgumentParser(description='Test line fitting standalone')
    parser.add_argument('--mask', type=str, help='Path to mask image file')
    parser.add_argument('--save', action='store_true', help='Save outputs')
    parser.add_argument('--output-dir', type=str, default='./line_fitting_results',
                        help='Directory to save results')
    parser.add_argument('--no-display', action='store_true', help='Disable display')
    parser.add_argument('--curved', action='store_true', help='Use curved lanes for synthetic test')
    parser.add_argument('--num-lanes', type=int, default=2, help='Number of lanes for synthetic test')
    args = parser.parse_args()
    
    display = not args.no_display
    
    if args.save:
        os.makedirs(args.output_dir, exist_ok=True)
    
    all_results = []
    
    if args.mask:
        # Load mask from file
        if not os.path.isfile(args.mask):
            print(f"Error: Mask file not found: {args.mask}")
            return 1
        
        mask = cv2.imread(args.mask, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            print(f"Error: Could not read mask file")
            return 1
        
        # Normalize to binary
        mask = (mask > 127).astype(np.uint8)
        
        save_path = os.path.join(args.output_dir, 'result.png') if args.save else None
        results, _ = test_line_fitting(mask, display=display, save_path=save_path)
        all_results.append(results)
    
    else:
        # Generate synthetic masks
        print("No input specified, using synthetic test masks...")
        
        # Test straight lanes
        print("\n>>> Testing STRAIGHT lanes <<<")
        mask = generate_synthetic_mask(num_lanes=args.num_lanes, curved=False)
        save_path = os.path.join(args.output_dir, 'straight_lanes.png') if args.save else None
        results, quit_requested = test_line_fitting(mask, display=display, save_path=save_path)
        all_results.append(results)
        
        if not quit_requested:
            # Test curved lanes
            print("\n>>> Testing CURVED lanes <<<")
            mask = generate_synthetic_mask(num_lanes=args.num_lanes, curved=True)
            save_path = os.path.join(args.output_dir, 'curved_lanes.png') if args.save else None
            results, quit_requested = test_line_fitting(mask, display=display, save_path=save_path)
            all_results.append(results)
        
        if not quit_requested:
            # Test single lane
            print("\n>>> Testing SINGLE lane <<<")
            mask = generate_synthetic_mask(num_lanes=1, curved=False)
            save_path = os.path.join(args.output_dir, 'single_lane.png') if args.save else None
            results, _ = test_line_fitting(mask, display=display, save_path=save_path)
            all_results.append(results)
    
    # Summary
    if all_results:
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        valid_results = [r for r in all_results if 'fitting_time' in r]
        if valid_results:
            avg_time = np.mean([r['fitting_time'] for r in valid_results])
            total_lanes = sum([r['num_lanes'] for r in valid_results])
            print(f"Average fitting time: {avg_time:.2f} ms")
            print(f"Total lanes detected: {total_lanes}")
        print(f"Total tests: {len(all_results)}")
    
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
