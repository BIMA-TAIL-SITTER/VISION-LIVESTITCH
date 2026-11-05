import cv2
import numpy as np
import logging
from collections import deque
from typing import List, Tuple, Optional
from .config import StitchConfig

logger = logging.getLogger(__name__)

class FeatureEngine:
    """SIFT feature detection and matching with ring-buffer caching."""
    
    def __init__(self, config: StitchConfig):
        # Create SIFT detector with config params
        self.sift = cv2.SIFT_create(
            nfeatures=config.sift_n_features,
            contrastThreshold=config.sift_contrast_threshold,
            edgeThreshold=config.sift_edge_threshold,
            sigma=config.sift_sigma,
        )
        self.matcher = cv2.BFMatcher()
        self.ratio = config.lowe_ratio
        self.min_matches = config.min_match_count
        
        # Ring buffer cache: stores (keypoints_coords, descriptors, frame_id)
        # keypoints_coords is np.float32 array of shape (N, 2)
        self.cache: deque = deque(maxlen=config.feature_cache_size)
        self._frame_counter = 0
    
    def detect(self, image: np.ndarray, roi_mask: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Detect SIFT features in image.
        
        Args:
            image: Input image (BGR or grayscale)
            roi_mask: Optional binary mask. SIFT only detects in white (255) regions.
            
        Returns:
            kp_coords: (N, 2) float32 array of keypoint (x, y) coordinates
            descriptors: (N, 128) float32 array of SIFT descriptors
        """
        # Convert to grayscale if needed
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        
        # Detect with optional mask
        keypoints, descriptors = self.sift.detectAndCompute(gray, roi_mask)
        
        if keypoints is None or len(keypoints) == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 128), dtype=np.float32)
        
        # Convert KeyPoint objects to coordinate array (faster for downstream)
        kp_coords = np.array([kp.pt for kp in keypoints], dtype=np.float32)
        
        return kp_coords, descriptors
    
    def match(self, des1: np.ndarray, des2: np.ndarray) -> List[Tuple[int, int]]:
        """Match descriptors using BFMatcher + Lowe's ratio test.
        
        Returns:
            List of (queryIdx, trainIdx) tuples for good matches
        """
        if des1 is None or des2 is None or len(des1) < 2 or len(des2) < 2:
            return []
        
        raw_matches = self.matcher.knnMatch(des1, des2, k=2)
        
        good = []
        for pair in raw_matches:
            if len(pair) == 2:
                m, n = pair
                if m.distance < self.ratio * n.distance:
                    good.append((m.queryIdx, m.trainIdx))
        
        return good
    
    def find_best_match_in_cache(self, des_new: np.ndarray) -> Optional[Tuple[List, np.ndarray, int]]:
        """Match new descriptors against all cached frames, return best match.
        
        Returns:
            Tuple of (match_indices, cached_kp_coords, cached_frame_id) or None
        """
        if not self.cache or des_new is None or len(des_new) < 2:
            return None
        
        best_matches = None
        best_kp = None
        best_id = -1
        best_count = 0
        
        for cached_kp, cached_des, frame_id in self.cache:
            matches = self.match(des_new, cached_des)
            if len(matches) > best_count:
                best_count = len(matches)
                best_matches = matches
                best_kp = cached_kp
                best_id = frame_id
        
        if best_count >= self.min_matches:
            logger.debug(f"Best cache match: frame {best_id} with {best_count} matches")
            return best_matches, best_kp, best_id
        
        logger.warning(f"No sufficient match in cache (best: {best_count} < {self.min_matches})")
        return None
    
    def add_to_cache(self, kp_coords: np.ndarray, descriptors: np.ndarray) -> int:
        """Add features to ring-buffer cache. Returns assigned frame_id."""
        frame_id = self._frame_counter
        self._frame_counter += 1
        self.cache.append((kp_coords.copy(), descriptors.copy(), frame_id))
        logger.debug(f"Cached frame {frame_id} ({len(kp_coords)} features, cache size: {len(self.cache)})")
        return frame_id
    
    def clear_cache(self):
        self.cache.clear()
        self._frame_counter = 0
