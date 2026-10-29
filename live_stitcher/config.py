"""
Configuration module for the Live Stitcher.
"""
from dataclasses import dataclass, asdict
from typing import Any, Dict


@dataclass
class StitchConfig:
    """Configuration for the live image stitcher."""
    # Feature Detection
    sift_n_features: int = 2000         # Increased from 1500 for better coverage on monotone terrain
    sift_contrast_threshold: float = 0.03
    sift_edge_threshold: float = 10.0
    sift_sigma: float = 1.6
    downsample_factor: int = 3          # Spatial downsample (3x = 9x pixel reduction)
    
    # Feature Matching  
    lowe_ratio: float = 0.55           # Tightened from 0.6 — fewer but higher-quality matches
    min_match_count: int = 12          # Lowered from 15 — accept more frames, reduce rejection rate
    
    # Redundancy Filter
    gps_distance_threshold_m: float = 3.0
    
    # Registration
    ransac_reproj_threshold: float = 3.0  # Tightened from 5.0 — more precise transform estimation
    prefer_affine: bool = True          # Prefer Partial Affine (4-DOF) over Homography (8-DOF)
    force_rigid: bool = True            # Force scale = 1.0 (Rigid SE(2)) to prevent scale drift in aerial flights
    
    # Canvas
    blend_feather_power: float = 3.0    # Increased from 2.0 — sharper transition like PROGRAM-SENDER1
    canvas_initial_padding: int = 500
    canvas_expand_margin: int = 200
    
    # Blending Quality
    enable_exposure_compensation: bool = True   # Match brightness in overlap region before blending
    enable_multiband_blend: bool = True         # Use Laplacian pyramid blend instead of simple feather
    multiband_levels: int = 2                   # Pyramid levels (2 = fast, 3 = smoother)
    exposure_gain_limit: float = 1.4            # Max gain for exposure compensation [1/limit, limit]
    
    # GPS Heading Correction
    enable_heading_correction: bool = True      # Correct frame rotation using GPS trajectory heading
    heading_min_distance_m: float = 2.0         # Min GPS distance for reliable heading estimation
    
    # Feature Cache
    feature_cache_size: int = 5
    
    # GPS-guided ROI
    roi_padding_factor: float = 0.3     # Extra padding around predicted overlap ROI (30%)
    gsd_default: float = 0.05           # Default Ground Sampling Distance (m/px) at downsample
    
    # Output
    output_dir: str = "output"
    save_intermediates: bool = False
    
    # Receiver
    receiver_host: str = "0.0.0.0"
    receiver_port: int = 5001

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "StitchConfig":
        """Create a StitchConfig instance from a dictionary."""
        valid_keys = cls.__dataclass_fields__.keys()
        filtered_d = {k: v for k, v in d.items() if k in valid_keys}
        return cls(**filtered_d)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the StitchConfig instance to a dictionary."""
        return asdict(self)
