"""
Machine Learning-based lane detection using YOLO segmentation.
"""

import torch
import numpy as np
import cv2
from ultralytics import YOLO

class MachineLearningLaneDetector:
    """
    YOLO-based lane detection using segmentation masks.
    """
    def __init__(self, model_path, width=330, height=180, confidence_threshold=0.5):
        self.width = width
        self.height = height
        self.confidence_threshold = confidence_threshold
        
        # Load the YOLO model
        self.model = YOLO(model_path)
        
        # Determine device
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

    def detect(self, frame):
        """
        Detect lane markings using YOLO segmentation.

        Args:
            frame: BGR image from camera

        Returns:
            Binary mask (width x height) with 255 for lane pixels, 0 otherwise.
        """
        # 1. Resize input to expected model dimensions
        input_img = cv2.resize(frame, (self.width, self.height))
        # Ensure 3 channels (strip alpha if present)
        input_img = input_img[:, :, :3]

        # 2. Run Inference
        results = self.model(input_img, verbose=False, device=self.device)
        
        # 3. Initialize an empty mask
        combined_mask = np.zeros((self.height, self.width), dtype=np.uint8)

        # 4. Process results
        if results and results[0].masks is not None:
            # results[0].masks contains the segmentation masks
            # results[0].boxes contains the metadata (classes, confidence)
            masks = results[0].masks.data
            boxes = results[0].boxes
            names = results[0].names

            for i, mask_tensor in enumerate(masks):
                conf = float(boxes[i].conf)
                cls_idx = int(boxes[i].cls)
                label = names[cls_idx]

                # Filter by label and confidence
                if label == 'lane' and conf > self.confidence_threshold:
                    # Move mask to CPU and convert to numpy
                    mask_np = mask_tensor.cpu().numpy() if hasattr(mask_tensor, 'cpu') else mask_tensor
                    
                    # YOLO masks are often smaller than the input (e.g., 160x160), 
                    # so we resize back to our target dimensions
                    mask_resized = cv2.resize(mask_np, (self.width, self.height))
                    
                    # Convert to binary (255 for lane)
                    binary_mask = (mask_resized > 0.5).astype(np.uint8) * 255
                    
                    # Combine with the main mask using bitwise OR
                    combined_mask = cv2.bitwise_or(combined_mask, binary_mask)

        return combined_mask