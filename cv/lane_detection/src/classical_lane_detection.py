"""
Classical lane detection using HSV color thresholding.

Detects white lane markings by thresholding in HSV color space, then removes
orange barrel/pylon regions to avoid false positives. No ML model required.
"""
import cv2
import numpy as np

class ClassicalLaneDetector:
    """
    Classical lane detection using HSV color thresholding and barrel exclusion.
    """

    def __init__(self, width=330, height=180, white_sensitivity=20, downscale_factor=1, horizon_crop=0.15):
        self.width = width
        self.height = height
        
        # HSV Threshold constants
        self.white_sensitivity = white_sensitivity # Low for narrow range, high for wide range of whites
        self.lower_white = np.array([0, 0, 255 - self.white_sensitivity])
        self.upper_white = np.array([255, self.white_sensitivity, 255])
        
        self.lower_orange = np.array([10, 100, 100])
        self.upper_orange = np.array([50, 255, 255])

        self.downscale_factor = downscale_factor
        self.horizon_crop = horizon_crop

    def _create_white_mask(self, img_hsv):
        """Create mask for white pixels (lane lines) in HSV."""
        return cv2.inRange(img_hsv, self.lower_white, self.upper_white)

    def _create_orange_mask(self, img_hsv):
        """Create mask for orange pixels (barrels/pylons) in HSV."""
        return cv2.inRange(img_hsv, self.lower_orange, self.upper_orange)

    def _get_mask(self, img_hsv, mask_method):
        """Downscale, apply mask method, upscale back."""
        h, w = img_hsv.shape[:2]
        
        small = cv2.resize(img_hsv, (w // self.downscale_factor, h // self.downscale_factor), interpolation=cv2.INTER_AREA)
        
        mask = mask_method(small)

        binary = np.where(mask > 0, 255, 0).astype(np.uint8)

        large = cv2.resize(binary, (w, h), interpolation=cv2.INTER_AREA)
        return large

    def _expand_barrels(self, mask):
        """Expand barrel detections vertically by +/-30px to create exclusion zones."""
        # If no barrels detected, return original mask to avoid unnecessary processing
        if mask.max() == 0:
            return mask
            
        h, w = mask.shape[:2]

        # Transformation matrices for vertical shifting
        m_up = np.float64([[1, 0, 0], [0, 1, 30]])
        m_down = np.float64([[1, 0, 0], [0, 1, -30]])
        
        up = cv2.warpAffine(mask, m_up, (w, h))
        down = cv2.warpAffine(mask, m_down, (w, h))
        
        # Combine original with shifted versions
        return cv2.bitwise_or(mask, cv2.bitwise_or(up, down))

    def detect(self, frame):
        """
        Detect lane markings using HSV color thresholding.

        Args:
            frame: BGR image from camera

        Returns:
            Binary mask (width x height) with 255 for lane pixels, 0 otherwise.
        """
        # Resize to target dimensions
        img = cv2.resize(frame, (self.width, self.height))
        
        # Convert BGR to HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Generate Lane Mask
        lanes = self._get_mask(img_hsv, self._create_white_mask)

        # Generate Barrel Mask and Expand it
        barrels = self._get_mask(img_hsv, self._create_orange_mask)
        barrels_expanded = self._expand_barrels(barrels)

        # Subtract barrels from lanes to remove false positives
        # Using bitwise_and with 'not' is cleaner for binary masks
        output = cv2.bitwise_and(lanes, cv2.bitwise_not(barrels_expanded))
        
        # Crop the top (sky/horizon) to remove distant false positives
        if self.horizon_crop > 0:
            crop_line = int(self.horizon_crop * self.height)
            output[:crop_line, :] = 0

        return output