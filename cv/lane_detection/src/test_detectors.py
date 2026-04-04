"""
Test script to compare our CV models using the same program as ran on the rover.
Runs both models on the same video feed and displays their masks.
"""

import cv2
import numpy as np
from classical_lane_detection import ClassicalLaneDetector
from ml_lane_detection import MachineLearningLaneDetector

# --- CONFIGURATION ---
VIDEO_PATH = 'sim_capture.mp4'
MODEL_PATH = '../models/best_model_int8.pt'
TARGET_W, TARGET_H = 640, 320  # The dimensions your model expects

def get_center_center_clip(frame, clip_w, clip_h):
    """
    Extracts a window of size clip_w x clip_h from the exact center of the frame.
    """
    h, w = frame.shape[:2]
    
    # Calculate the center point of the original high-res frame
    center_x, center_y = w // 2, h // 2
    
    # Calculate the start and end coordinates based on target size
    start_x = center_x - (clip_w // 2)
    end_x = start_x + clip_w
    
    start_y = center_y - (clip_h // 2)
    end_y = start_y + clip_h
    
    # Boundary checks to prevent crashing if the clip is larger than the frame
    start_x = max(0, start_x)
    start_y = max(0, start_y)
    end_x = min(w, end_x)
    end_y = min(h, end_y)
    
    return frame[start_y:end_y, start_x:end_x]

def create_overlay(original, mask, color=(0, 255, 0), alpha=0.5):
    output = original.copy()
    if mask.shape[:2] != original.shape[:2]:
        mask = cv2.resize(mask, (original.shape[1], original.shape[0]))
    
    # Create a colored version of the ROI
    colored_layer = np.full_like(original, color)
    
    # Only blend where the mask is 255
    mask_bool = mask == 255
    output[mask_bool] = cv2.addWeighted(original, 1-alpha, colored_layer, alpha, 0)[mask_bool]
    
    return output

def main():
    cap = cv2.VideoCapture(VIDEO_PATH)
    classical = ClassicalLaneDetector(width=TARGET_W, height=TARGET_H, horizon_crop=0, morph_size=0, morph_open_iters=0, morph_close_iters=0)
    ml_model = MachineLearningLaneDetector(model_path=MODEL_PATH, width=TARGET_W, height=TARGET_H)

    current_delay = 1
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        # 1. Clip the Region of Interest
        roi = get_center_center_clip(frame, TARGET_W, TARGET_H)

        # 2. Run detections
        mask_classical = classical.detect(roi)
        mask_ml = ml_model.detect(roi)

        # 3. Create Overlays
        view_classical = create_overlay(roi, mask_classical, color=(255, 0, 0)) # Blue
        view_ml = create_overlay(roi, mask_ml, color=(0, 255, 0))              # Green

        # 4. Add text labels to each view before stacking
        # Parameters: (image, text, position, font, scale, color, thickness)
        cv2.putText(view_classical, "CLASSICAL (HSV)", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        cv2.putText(view_ml, "YOLO ML (INT8)", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # 5. Combine and Display
        comparison = np.hstack((view_classical, view_ml))
        
        # Scaling up for visibility
        display_output = cv2.resize(comparison, (TARGET_W * 2, TARGET_H)) 
        
        cv2.imshow('Clipped Lane Detection Comparison', display_output)

        key = cv2.waitKey(current_delay) & 0xFF
        if key == ord('s'):
            current_delay = 100  # Switch to Slow Motion
        elif key == ord('w'):
            current_delay = 1    # Resume Fast
        elif key == ord(' '):
            print("Paused. Press any key to resume...")
            cv2.waitKey(0)       # Wait for any key to unpause
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()