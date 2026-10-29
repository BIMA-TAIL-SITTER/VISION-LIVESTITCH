"""
GPS Heading Corrector — Eliminates rotation drift using GPS trajectory.

For nadir drone photography, consecutive frames should have minimal
relative rotation. However, small errors in SIFT matching accumulate
over hundreds of frames, causing the mosaic to rotate 30-45°.

This module estimates the flight heading from GPS coordinates and
pre-rotates each frame to a consistent orientation BEFORE feature
matching, dramatically improving alignment accuracy.

This serves a similar purpose to PROGRAM-SENDER1's `computeUnRotMatrix()`
but uses GPS trajectory instead of IMU data.
"""

import logging
import math
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two GPS points in meters."""
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class HeadingCorrector:
    """Correct frame rotation using GPS trajectory heading.
    
    Strategy:
    1. Compute forward azimuth (bearing) between consecutive GPS points.
    2. Use the first reliable heading as reference (north-up alignment).
    3. For each subsequent frame, compute heading delta from reference.
    4. Pre-rotate the frame by the negative delta so all frames share
       the same orientation — eliminating cumulative rotation drift.
    
    The heading is smoothed using an exponential moving average to
    reject GPS noise from hovering or slow-moving segments.
    """
    
    def __init__(self, min_distance_m: float = 2.0, smoothing: float = 0.6):
        """
        Args:
            min_distance_m: Minimum GPS distance for reliable heading.
                           Below this, heading is unreliable (drone may be hovering).
            smoothing: EMA smoothing factor (0=no smooth, 1=max smooth).
        """
        self.min_distance_m = min_distance_m
        self.smoothing = smoothing
        
        self._reference_heading: Optional[float] = None  # First reliable heading
        self._smoothed_heading: Optional[float] = None    # EMA-smoothed heading
        self._prev_gps: Optional[Dict[str, float]] = None
        self._heading_history: list = []
    
    def compute_heading(
        self, 
        prev_lat: float, prev_lon: float,
        curr_lat: float, curr_lon: float,
    ) -> Optional[float]:
        """Compute forward azimuth (bearing) in degrees [0, 360).
        
        Uses standard geodetic forward azimuth formula.
        Returns None if distance is too small for reliable heading.
        """
        dist = _haversine_m(prev_lat, prev_lon, curr_lat, curr_lon)
        if dist < self.min_distance_m:
            return None  # Too close — heading unreliable
        
        lat1 = math.radians(prev_lat)
        lat2 = math.radians(curr_lat)
        dlon = math.radians(curr_lon - prev_lon)
        
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        
        heading = math.degrees(math.atan2(x, y))
        heading = heading % 360.0  # Normalize to [0, 360)
        
        return heading
    
    def update_and_get_correction(
        self,
        prev_gps: Dict[str, float],
        curr_gps: Dict[str, float],
    ) -> Optional[float]:
        """Compute rotation correction angle for the current frame.
        
        Args:
            prev_gps: Previous GPS with "latitude", "longitude" keys.
            curr_gps: Current GPS with "latitude", "longitude" keys.
            
        Returns:
            Rotation correction angle in degrees (counter-clockwise positive).
            None if heading cannot be reliably estimated.
        """
        heading = self.compute_heading(
            prev_gps["latitude"], prev_gps["longitude"],
            curr_gps["latitude"], curr_gps["longitude"],
        )
        
        if heading is None:
            # Not enough GPS movement — use last known heading
            if self._smoothed_heading is not None:
                heading = self._smoothed_heading
            else:
                return None
        
        # Initialize reference heading on first reliable computation
        if self._reference_heading is None:
            self._reference_heading = heading
            self._smoothed_heading = heading
            self._heading_history.append(heading)
            logger.info(f"Reference heading set: {heading:.1f}°")
            return 0.0  # First frame = no correction needed
        
        # Smooth heading with EMA (handles GPS jitter)
        # Use circular mean to handle 360/0 wraparound
        delta = heading - self._smoothed_heading
        # Normalize delta to [-180, 180]
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        
        self._smoothed_heading = (self._smoothed_heading + 
                                   (1 - self.smoothing) * delta) % 360.0
        self._heading_history.append(self._smoothed_heading)
        
        # Correction = negative of heading change from reference
        correction = self._reference_heading - self._smoothed_heading
        # Normalize to [-180, 180]
        if correction > 180:
            correction -= 360
        elif correction < -180:
            correction += 360
        
        return correction
    
    def correct_image(
        self,
        image: np.ndarray,
        correction_deg: float,
    ) -> np.ndarray:
        """Rotate image by correction angle around its center.
        
        Uses warpAffine with BORDER_CONSTANT to maintain image size.
        The rotation undoes the heading change so all frames are
        aligned to the reference heading direction.
        
        Args:
            image: Input BGR image.
            correction_deg: Rotation angle in degrees (CCW positive).
            
        Returns:
            Rotated image (same size as input).
        """
        if abs(correction_deg) < 0.1:
            return image  # Skip rotation for negligible angles
        
        h, w = image.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        
        M = cv2.getRotationMatrix2D((cx, cy), correction_deg, 1.0)
        rotated = cv2.warpAffine(
            image, M, (w, h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        
        return rotated
    
    def reset(self):
        """Reset heading state for a new session."""
        self._reference_heading = None
        self._smoothed_heading = None
        self._prev_gps = None
        self._heading_history = []
    
    @property
    def reference_heading(self) -> Optional[float]:
        return self._reference_heading
    
    @property
    def current_heading(self) -> Optional[float]:
        return self._smoothed_heading
