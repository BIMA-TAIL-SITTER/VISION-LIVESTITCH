import logging
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import StitchConfig

logger = logging.getLogger(__name__)


class Registrator:
    """Geometric transformation estimation between frames.
    
    Supports:
    - Partial Affine (4-DOF: rotation, uniform scale, translation) — preferred for stability
    - Full Homography (8-DOF) via RANSAC — fallback
    - GPS-guided ROI prediction to restrict feature detection area
    """
    
    def __init__(self, config: StitchConfig):
        self.ransac_threshold = config.ransac_reproj_threshold
        self.prefer_affine = config.prefer_affine
        self.force_rigid = getattr(config, "force_rigid", True)
        self.min_matches = config.min_match_count
        self.roi_padding = config.roi_padding_factor
    
    def estimate_transform(
        self,
        kp_src: np.ndarray,   # (N, 2) source keypoint coords
        kp_dst: np.ndarray,   # (M, 2) destination keypoint coords  
        match_indices: List[Tuple[int, int]],  # (src_idx, dst_idx) pairs
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Estimate geometric transform from matched keypoints.
        
        Strategy:
        1. Try estimateAffinePartial2D (rotation + uniform scale + translation)
           - If force_rigid is True: scale is normalized to exactly 1.0 (Rigid SE(2)).
             This completely prevents cumulative scale drift in drone orthomosaics.
        2. If prefer_affine is True, do NOT fall back to 8-DOF homography!
           Full homography on nadir aerial images causes severe projective runaway.
        3. Only use findHomography if prefer_affine is False.
        
        Returns:
            Tuple of (3x3 transform matrix, inlier mask) or None
        """
        if len(match_indices) < self.min_matches:
            return None
        
        pts_src = np.float32([kp_src[i] for i, _ in match_indices]).reshape(-1, 1, 2)
        pts_dst = np.float32([kp_dst[j] for _, j in match_indices]).reshape(-1, 1, 2)
        
        if self.prefer_affine:
            # Try Partial Affine first (4-DOF: rotation, uniform scale, translation)
            M, inliers = cv2.estimateAffinePartial2D(
                pts_src, pts_dst,
                method=cv2.RANSAC,
                ransacReprojThreshold=self.ransac_threshold,
                maxIters=2000,
                confidence=0.99,
            )
            if M is not None and inliers is not None:
                inlier_count = int(inliers.sum())
                inlier_ratio = inlier_count / len(match_indices)
                if inlier_ratio >= 0.25 and inlier_count >= self.min_matches:
                    a, b = M[0, 0], M[0, 1]
                    s = float(np.sqrt(a * a + b * b))
                    # Scale sanity check before normalization
                    if 0.7 <= s <= 1.4:
                        if self.force_rigid:
                            # Normalize scale to exactly 1.0 (Euclidean SE(2) Rigid Transform)
                            # Eliminates cumulative scale drift across sequential frames
                            M[:2, :2] /= s
                        H = np.eye(3, dtype=np.float64)
                        H[:2, :] = M
                        logger.debug(
                            f"Affine (rigid={self.force_rigid}): "
                            f"{inlier_count}/{len(match_indices)} inliers ({inlier_ratio:.1%})"
                        )
                        return H, inliers
                    else:
                        logger.warning(f"Affine rejected: implausible scale {s:.3f}")
            # Do NOT fall back to 8-DOF homography when prefer_affine is True!
            logger.warning("Affine registration failed")
            return None
        
        # Fallback: Full Homography (only when prefer_affine is False)
        H, inliers = cv2.findHomography(
            pts_src, pts_dst,
            cv2.RANSAC,
            self.ransac_threshold,
            maxIters=2000,
            confidence=0.99,
        )
        if H is not None and inliers is not None:
            inlier_count = int(inliers.sum())
            if inlier_count >= self.min_matches:
                logger.debug(f"Homography: {inlier_count}/{len(match_indices)} inliers")
                return H, inliers
        
        logger.warning("Homography transform estimation failed")
        return None
    
    def predict_overlap_roi(
        self,
        image_shape: Tuple[int, int],   # (height, width) of new image
        gps_offset_px: Tuple[float, float],  # (dx, dy) pixel offset from GPS
    ) -> np.ndarray:
        """Predict overlap region as binary mask using GPS pixel offset.
        
        The overlap region is where the new image overlaps with the previous frame.
        We create a mask that covers this predicted overlap area + padding.
        
        If GPS offset is small or zero, return full mask (detect everywhere).
        
        Args:
            image_shape: (height, width) of the new image
            gps_offset_px: estimated (dx, dy) pixel offset from GPS
            
        Returns:
            Binary mask (uint8) same size as image, 255 in predicted overlap region
        """
        h, w = image_shape[:2]
        dx, dy = gps_offset_px
        
        # If offset is very small or unknown, use full image
        if abs(dx) < 10 and abs(dy) < 10:
            return np.full((h, w), 255, dtype=np.uint8)
        
        # The overlap region in the new image is opposite to the movement direction
        # If drone moved right (dx > 0), overlap is on the LEFT side of new image
        # If drone moved down (dy > 0), overlap is on the TOP side of new image
        
        pad_x = int(w * self.roi_padding)
        pad_y = int(h * self.roi_padding)
        
        # Calculate overlap bounds in the new image coordinate system
        if dx >= 0:  # moved right -> overlap on left
            x_start = 0
            x_end = min(w, int(w - dx + pad_x))
        else:  # moved left -> overlap on right
            x_start = max(0, int(-dx - pad_x))
            x_end = w
        
        if dy >= 0:  # moved down -> overlap on top
            y_start = 0
            y_end = min(h, int(h - dy + pad_y))
        else:  # moved up -> overlap on bottom
            y_start = max(0, int(-dy - pad_y))
            y_end = h
        
        # Clamp bounds
        x_start = max(0, x_start)
        x_end = min(w, x_end)
        y_start = max(0, y_start)
        y_end = min(h, y_end)
        
        # Ensure minimum ROI size (at least 30% of image)
        if (x_end - x_start) < w * 0.3:
            return np.full((h, w), 255, dtype=np.uint8)
        if (y_end - y_start) < h * 0.3:
            return np.full((h, w), 255, dtype=np.uint8)
        
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y_start:y_end, x_start:x_end] = 255
        
        return mask
    
    @staticmethod
    def compute_canvas_bounds(
        current_bounds: Tuple[float, float, float, float],  # (xmin, ymin, xmax, ymax)
        new_shape: Tuple[int, int],  # (height, width)
        H: np.ndarray,  # 3x3 transform for new image
        margin: int = 200,
    ) -> Tuple[float, float, float, float]:
        """Compute expanded canvas bounds to fit the new transformed image."""
        h, w = new_shape[:2]
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        transformed = cv2.perspectiveTransform(corners, H)
        
        new_xmin = float(transformed[:, 0, 0].min()) - margin
        new_ymin = float(transformed[:, 0, 1].min()) - margin
        new_xmax = float(transformed[:, 0, 0].max()) + margin
        new_ymax = float(transformed[:, 0, 1].max()) + margin
        
        xmin = min(current_bounds[0], new_xmin)
        ymin = min(current_bounds[1], new_ymin)
        xmax = max(current_bounds[2], new_xmax)
        ymax = max(current_bounds[3], new_ymax)
        
        return (xmin, ymin, xmax, ymax)
