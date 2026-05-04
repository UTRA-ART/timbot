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

    def __init__(self, width=330, height=180, white_sensitivity=20, downscale_factor=1, horizon_crop=0.15, morph_size=3, morph_open_iters=1, morph_close_iters=1):
        self.width = width
        self.height = height
        
        # HSV Threshold constants
        self.white_sensitivity = white_sensitivity # Low for narrow range, high for wide range of whites
        self.lower_white = np.array([0, 0, 255 - self.white_sensitivity])
        self.upper_white = np.array([255, self.white_sensitivity, 255])
        
        self.lower_orange = np.array([10, 100, 100])
        self.upper_orange = np.array([50, 255, 255])

        self.downscale_factor = downscale_factor

        # Morphological filtering parameters
        self.morph_size = morph_size
        self.morph_open_iters = morph_open_iters
        self.morph_close_iters = morph_close_iters

        # Larger for more aggressive noise removal and gap filling, but risks eroding thin lane lines if too large
        self.morph_kernel = np.ones((self.morph_size, self.morph_size), np.uint8)

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

    def _apply_morphology(self, mask):
        """Removes small noise and fills gaps in lane lines."""
        # https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html

        # 1. OPENING: Erase small white 'dust' (False Positives)
        # This removes any white blobs smaller than the kernel
        # Increase iterations if you still see small blobs on the ground
        if self.morph_open_iters > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=self.morph_open_iters)

        # 2. CLOSING: Fill small black gaps in white lines (Dashed lines)
        # This helps 'stitch' a dashed lane into a solid shape
        # Increase iterations if you see gaps in the lane lines
        if self.morph_close_iters > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=self.morph_close_iters)
        return mask

    def _apply_hough_filter(self, binary_mask):
        """Alternative filter using Hough Transform"""
        # 1. Detect edges
        edges = cv2.Canny(binary_mask, 50, 150)
        
        # 2. Find lines
        # rho=1, theta=pi/180, threshold=50, minLineLength=50, maxLineGap=20
        lines = cv2.HoughLinesP(binary_mask, 1, np.pi/180, 50, minLineLength=50, maxLineGap=20)
        
        hough_mask = np.zeros_like(binary_mask)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y2 - y1) > abs(x2 - x1) * 0.5: # Only keep 'mostly vertical' lines
                    cv2.line(hough_mask, (x1, y1), (x2, y2), 255, thickness=2)
                    
        return hough_mask

    def _filter_by_area(self, binary_mask, min_area=150):
        """
        Removes small noise blobs while preserving thin, long lines.
        """
        # 1. Label every disconnected 'blob' of white pixels
        # connectivity=8 looks at diagonals; 4 only looks up/down/left/right
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary_mask, connectivity=8
        )

        # 2. Create a blank canvas
        clean_mask = np.zeros_like(binary_mask)

        # 3. Loop through found blobs (label 0 is the background, so skip it)
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            
            # Only keep blobs that have enough pixels to be a lane segment
            if area > min_area:
                clean_mask[labels == i] = 255
                
        return clean_mask

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

        # Apply morphological filtering to clean up outliers
        # lanes = self._apply_morphology(lanes)

        # Generate Hough Mask (alternative approach)
        # lanes = self._apply_hough_filter(lanes)

        # Filter by area
        lanes = self._filter_by_area(lanes, min_area=300)

        # Generate Barrel Mask and Expand it
        barrels = self._get_mask(img_hsv, self._create_orange_mask)
        barrels_expanded = self._expand_barrels(barrels)

        # Subtract barrels
        output = cv2.bitwise_and(lanes, cv2.bitwise_not(barrels_expanded))
        
        # Crop horizon
        if self.horizon_crop > 0:
            crop_line = int(self.horizon_crop * self.height)
            output[:crop_line, :] = 0

        return output