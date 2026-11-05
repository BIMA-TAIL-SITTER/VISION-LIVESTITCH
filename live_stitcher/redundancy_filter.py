import logging
from typing import List, Tuple, Optional
from .gps_utils import haversine_m

logger = logging.getLogger(__name__)

class GPSRedundancyFilter:
    """Filter redundant images based on GPS distance threshold.
    
    Designed for sequential drone flight paths. Compares each new frame
    against the last accepted frame using Haversine distance.
    O(1) per check instead of O(N) all-pairs comparison.
    """
    
    def __init__(self, threshold_m: float = 3.0):
        self.threshold_m = threshold_m
        self._accepted_positions: List[Tuple[float, float]] = []
        self._rejected_count: int = 0
        self._accepted_count: int = 0
    
    def should_accept(self, lat: float, lon: float) -> bool:
        """Check if image at (lat, lon) should be accepted.
        
        Returns True if:
        - This is the first image, OR
        - Distance from last accepted position >= threshold_m
        """
        # First image always accepted
        if not self._accepted_positions:
            self._accepted_positions.append((lat, lon))
            self._accepted_count += 1
            logger.info(f"First frame accepted at ({lat:.6f}, {lon:.6f})")
            return True
        
        last_lat, last_lon = self._accepted_positions[-1]
        dist = haversine_m(lat, lon, last_lat, last_lon)
        
        if dist >= self.threshold_m:
            self._accepted_positions.append((lat, lon))
            self._accepted_count += 1
            logger.debug(f"Frame accepted: dist={dist:.2f}m >= {self.threshold_m}m")
            return True
        else:
            self._rejected_count += 1
            logger.debug(f"Frame rejected: dist={dist:.2f}m < {self.threshold_m}m")
            return False
    
    @property
    def stats(self) -> dict:
        return {
            "accepted": self._accepted_count,
            "rejected": self._rejected_count,
            "total": self._accepted_count + self._rejected_count,
            "acceptance_rate": self._accepted_count / max(1, self._accepted_count + self._rejected_count),
        }
    
    @property 
    def last_position(self) -> Optional[Tuple[float, float]]:
        return self._accepted_positions[-1] if self._accepted_positions else None
    
    def reset(self):
        self._accepted_positions.clear()
        self._rejected_count = 0
        self._accepted_count = 0
        logger.info("Redundancy filter reset")
