"""
Live Stitcher — Incremental SIFT-based Mosaic Orchestrator.

This module contains the main LiveStitcher class that orchestrates the
entire incremental stitching pipeline:

    1. GPS redundancy filtering
    2. Image downsampling
    3. GPS-guided ROI prediction
    4. SIFT feature detection (ROI-restricted)
    5. Feature matching against cache
    6. Affine/Homography estimation
    7. Global transform accumulation
    8. Incremental canvas blending

Each frame is processed in constant time O(1) regardless of mosaic size,
because only the new image is warped and blended — the existing canvas
is never re-warped.

Author: BIMA SWARM — VISION-LIVESTITCH
"""

import logging
import time
import os
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np

from .config import StitchConfig
from .gps_utils import (
    extract_gps_exif,
    haversine_m,
    gps_to_local_meters,
    estimate_pixel_offset,
)
from .redundancy_filter import GPSRedundancyFilter
from .feature_engine import FeatureEngine
from .registrator import Registrator
from .canvas import IncrementalCanvas
from .heading_corrector import HeadingCorrector

logger = logging.getLogger(__name__)


@dataclass
class FrameStats:
    """Timing and quality statistics for a single frame."""

    frame_id: int = 0
    timestamp: float = 0.0
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_alt: float = 0.0
    was_accepted: bool = False
    redundancy_dist_m: float = 0.0
    num_features_detected: int = 0
    num_matches: int = 0
    inlier_ratio: float = 0.0
    transform_type: str = ""  # "affine" or "homography"
    time_downsample_ms: float = 0.0
    time_detect_ms: float = 0.0
    time_match_ms: float = 0.0
    time_transform_ms: float = 0.0
    time_warp_blend_ms: float = 0.0
    time_total_ms: float = 0.0


@dataclass
class SessionStats:
    """Aggregate statistics for the stitching session."""

    total_frames_received: int = 0
    total_frames_accepted: int = 0
    total_frames_rejected: int = 0
    total_frames_failed: int = 0
    avg_time_per_frame_ms: float = 0.0
    frame_times: List[float] = field(default_factory=list)
    canvas_size: Tuple[int, int] = (0, 0)


class LiveStitcher:
    """Incremental live stitcher using SIFT features and GPS guidance.

    Designed for real-time drone image stitching where images arrive
    one at a time with GPS metadata. Each frame is stitched incrementally
    onto an accumulator canvas without re-processing previous frames.

    Usage:
        stitcher = LiveStitcher(config)

        # Mode 1: Process image with GPS dict
        mosaic = stitcher.process_frame(image, {"latitude": -7.12, "longitude": 112.78, "altitude": 50.0})

        # Mode 2: Process image file (GPS extracted from EXIF)
        mosaic = stitcher.process_file("path/to/image.jpg")

        # Get current mosaic at any time
        mosaic = stitcher.get_mosaic()
    """

    def __init__(self, config: Optional[StitchConfig] = None):
        """Initialize LiveStitcher.

        Args:
            config: Stitching configuration. Uses defaults if None.
        """
        self.config = config or StitchConfig()

        # Sub-modules
        self.filter = GPSRedundancyFilter(
            threshold_m=self.config.gps_distance_threshold_m
        )
        self.engine = FeatureEngine(self.config)
        self.registrator = Registrator(self.config)
        self.canvas = IncrementalCanvas(self.config)
        
        # GPS Heading Corrector — eliminates rotation drift
        self.heading_corrector: Optional[HeadingCorrector] = None
        if getattr(self.config, 'enable_heading_correction', True):
            self.heading_corrector = HeadingCorrector(
                min_distance_m=getattr(self.config, 'heading_min_distance_m', 2.0),
            )

        # Global transformation state
        self._H_global = np.eye(3, dtype=np.float64)
        self._H_prev_rel = np.eye(3, dtype=np.float64)

        # Maps feature cache frame_id -> H_global at that frame
        # Used for correct chaining when cache match is not the last frame
        self._H_global_per_frame: Dict[int, np.ndarray] = {}

        # GPS state
        self._origin_gps: Optional[Dict[str, float]] = None
        self._prev_gps: Optional[Dict[str, float]] = None
        self._gsd_estimate: float = self.config.gsd_default

        # Counters
        self._frame_count = 0
        self._accepted_count = 0
        self._failed_count = 0
        self._session_stats = SessionStats()

        # Output
        self._output_dir = Path(self.config.output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)

        logger.info(
            f"LiveStitcher initialized | SIFT={self.config.sift_n_features} "
            f"| ratio={self.config.lowe_ratio} | GPS_thresh={self.config.gps_distance_threshold_m}m "
            f"| downsample={self.config.downsample_factor}x"
            f"| heading_correction={'ON' if self.heading_corrector else 'OFF'}"
            f"| exposure_comp={'ON' if self.config.enable_exposure_compensation else 'OFF'}"
            f"| multiband={'ON' if self.config.enable_multiband_blend else 'OFF'}"
        )

    def process_frame(
        self,
        image: np.ndarray,
        gps: Dict[str, float],
    ) -> Optional[np.ndarray]:
        """Process a single frame through the incremental stitching pipeline.

        Args:
            image: BGR image (np.ndarray).
            gps: GPS metadata dict with keys "latitude", "longitude", "altitude".

        Returns:
            Updated mosaic image (np.ndarray) if frame was accepted and
            stitched successfully. None if frame was rejected by redundancy
            filter or if stitching failed.
        """
        t_start = time.perf_counter()
        self._frame_count += 1
        stats = FrameStats(
            frame_id=self._frame_count,
            timestamp=time.time(),
            gps_lat=gps.get("latitude", 0.0),
            gps_lon=gps.get("longitude", 0.0),
            gps_alt=gps.get("altitude", 0.0),
        )

        lat = gps["latitude"]
        lon = gps["longitude"]
        alt = gps.get("altitude", 0.0)

        # ── Step 1: Redundancy Filter ───────────────────────────────────
        if not self.filter.should_accept(lat, lon):
            stats.was_accepted = False
            self._session_stats.total_frames_received += 1
            self._session_stats.total_frames_rejected += 1
            logger.debug(f"Frame {self._frame_count} rejected (redundant)")
            return None

        stats.was_accepted = True
        self._session_stats.total_frames_received += 1
        self._session_stats.total_frames_accepted += 1

        # ── Step 2: Downsample ──────────────────────────────────────────
        t0 = time.perf_counter()
        ds = self.config.downsample_factor
        if ds > 1:
            working = image[::ds, ::ds].copy()
        else:
            working = image.copy()
        stats.time_downsample_ms = (time.perf_counter() - t0) * 1000

        # ── Step 2b: GPS Heading Correction ─────────────────────────────
        # Pre-rotate frame based on GPS trajectory heading to eliminate
        # cumulative rotation drift. Similar to PROGRAM-SENDER1's
        # computeUnRotMatrix but uses GPS trajectory instead of IMU.
        if self.heading_corrector is not None and self._prev_gps is not None:
            curr_gps = {"latitude": lat, "longitude": lon}
            correction = self.heading_corrector.update_and_get_correction(
                self._prev_gps, curr_gps
            )
            if correction is not None and abs(correction) > 0.1:
                working = self.heading_corrector.correct_image(working, correction)
                logger.debug(
                    f"Heading correction: {correction:.1f}° "
                    f"(heading={self.heading_corrector.current_heading:.1f}°)"
                )

        # ── Step 3: First Frame — Initialize ────────────────────────────
        if self._accepted_count == 0:
            self._origin_gps = {"latitude": lat, "longitude": lon, "altitude": alt}
            self._prev_gps = {"latitude": lat, "longitude": lon, "altitude": alt}
            self._H_global = np.eye(3, dtype=np.float64)

            # Initialize canvas
            self.canvas.initialize(working)

            # Cache features of first frame
            t0 = time.perf_counter()
            kp, des = self.engine.detect(working)
            stats.time_detect_ms = (time.perf_counter() - t0) * 1000
            stats.num_features_detected = len(kp)

            if len(kp) > 0:
                frame_id = self.engine.add_to_cache(kp, des)
                # Store H_global for this frame so future matches can chain correctly
                self._H_global_per_frame[frame_id] = self._H_global.copy()

            self._accepted_count = 1
            stats.time_total_ms = (time.perf_counter() - t_start) * 1000
            self._session_stats.frame_times.append(stats.time_total_ms)

            logger.info(
                f"Frame 1 initialized canvas | {len(kp)} features | "
                f"{stats.time_total_ms:.1f}ms"
            )
            return self.canvas.get_mosaic()

        # ── Step 4: GPS-guided ROI Prediction ───────────────────────────
        roi_mask = None
        if self._prev_gps is not None:
            try:
                dx_px, dy_px = estimate_pixel_offset(
                    lat,
                    lon,
                    self._prev_gps["latitude"],
                    self._prev_gps["longitude"],
                    self._gsd_estimate * ds,  # scale GSD by downsample factor
                )
                roi_mask = self.registrator.predict_overlap_roi(
                    working.shape[:2], (dx_px, dy_px)
                )
            except Exception as e:
                logger.debug(f"ROI prediction failed, using full image: {e}")
                roi_mask = None

        # ── Step 5: SIFT Feature Detection (ROI-restricted) ─────────────
        t0 = time.perf_counter()
        kp_new, des_new = self.engine.detect(working, roi_mask)
        stats.time_detect_ms = (time.perf_counter() - t0) * 1000
        stats.num_features_detected = len(kp_new)

        if len(kp_new) < self.config.min_match_count:
            logger.warning(
                f"Frame {self._frame_count}: insufficient features "
                f"({len(kp_new)} < {self.config.min_match_count})"
            )
            self._failed_count += 1
            self._session_stats.total_frames_failed += 1
            return None

        # ── Step 6: Feature Matching against Cache ──────────────────────
        t0 = time.perf_counter()
        match_result = self.engine.find_best_match_in_cache(des_new)
        stats.time_match_ms = (time.perf_counter() - t0) * 1000

        if match_result is None:
            logger.warning(
                f"Frame {self._frame_count}: no sufficient matches in cache"
            )
            self._failed_count += 1
            self._session_stats.total_frames_failed += 1
            return None

        match_indices, cached_kp, cached_frame_id = match_result
        stats.num_matches = len(match_indices)

        # ── Step 7: Transform Estimation ────────────────────────────────
        t0 = time.perf_counter()
        transform_result = self.registrator.estimate_transform(
            kp_new, cached_kp, match_indices
        )
        stats.time_transform_ms = (time.perf_counter() - t0) * 1000

        if transform_result is None:
            logger.warning(
                f"Frame {self._frame_count}: transform estimation failed"
            )
            self._failed_count += 1
            self._session_stats.total_frames_failed += 1
            return None

        H_rel, inlier_mask = transform_result
        inlier_count = int(inlier_mask.sum()) if inlier_mask is not None else 0
        stats.inlier_ratio = inlier_count / max(1, len(match_indices))
        stats.transform_type = (
            "affine" if self.config.prefer_affine else "homography"
        )

        # ── Step 7b: Sanity-check the relative transform ─────────────────
        # For affine transforms, also check rotation angle and scale ratio.
        # Bad SIFT matches on monotone terrain (grass, sand) often produce
        # valid-looking RANSAC results but with impossible rotation/scale.
        h_img, w_img = working.shape[:2]
        tx = abs(H_rel[0, 2])
        ty = abs(H_rel[1, 2])

        reject_reason = None

        # Check 1: Translation must be < 1.5× image size
        if tx > w_img * 1.5 or ty > h_img * 1.5:
            reject_reason = (
                f"translation ({tx:.0f}, {ty:.0f}) px > 1.5× "
                f"image ({w_img * 1.5:.0f}, {h_img * 1.5:.0f}) px"
            )

        # Check 2: For affine matrix, check rotation and scale
        if reject_reason is None:
            # H_rel is 3x3; top-left 2x2 encodes rotation+scale
            a, b = H_rel[0, 0], H_rel[0, 1]
            scale = np.sqrt(a * a + b * b)
            angle_deg = abs(np.degrees(np.arctan2(b, a)))
            if angle_deg > 180:
                angle_deg = 360 - angle_deg
            if scale < 0.6 or scale > 1.6:
                reject_reason = f"scale {scale:.3f} outside [0.6, 1.6]"
            elif angle_deg > 25.0:
                reject_reason = f"rotation {angle_deg:.1f}° > 25°"

        if reject_reason is not None:
            logger.warning(
                f"Frame {self._frame_count}: transform REJECTED — {reject_reason}"
            )
            self._failed_count += 1
            self._session_stats.total_frames_failed += 1
            return None

        # ── Step 8: Update Global Transform ─────────────────────────────
        # Chain H_rel against H_global of the MATCHED cached frame.
        H_base = self._H_global_per_frame.get(cached_frame_id, self._H_global)
        self._H_prev_rel = H_rel.copy()
        self._H_global = H_base @ H_rel

        # ── Step 9: Warp & Blend onto Canvas ────────────────────────────
        t0 = time.perf_counter()
        try:
            ok = self.canvas.add_image(working, self._H_global)
            if not ok:
                logger.warning(
                    f"Frame {self._frame_count}: canvas REJECTED frame — "
                    f"expansion too large (bad transform). Rolling back H_global."
                )
                self._H_global = H_base.copy()
                self._failed_count += 1
                self._session_stats.total_frames_failed += 1
                return None
        except Exception as e:
            logger.error(f"Frame {self._frame_count}: canvas blend failed: {e}")
            self._H_global = H_base.copy()
            self._failed_count += 1
            self._session_stats.total_frames_failed += 1
            return None
        stats.time_warp_blend_ms = (time.perf_counter() - t0) * 1000

        # ── Step 10: Cache Features + Store H_global ────────────────────
        frame_id = self.engine.add_to_cache(kp_new, des_new)
        self._H_global_per_frame[frame_id] = self._H_global.copy()

        # Prune H_global dict to only keep entries that are still in the ring cache
        cached_ids = {entry[2] for entry in self.engine.cache}
        self._H_global_per_frame = {
            k: v for k, v in self._H_global_per_frame.items()
            if k in cached_ids
        }

        # Update GPS state
        self._prev_gps = {"latitude": lat, "longitude": lon, "altitude": alt}
        self._accepted_count += 1

        # Timing
        stats.time_total_ms = (time.perf_counter() - t_start) * 1000
        self._session_stats.frame_times.append(stats.time_total_ms)

        # Update session stats
        self._session_stats.canvas_size = (
            self.canvas.canvas_w,
            self.canvas.canvas_h,
        )
        self._session_stats.avg_time_per_frame_ms = (
            sum(self._session_stats.frame_times)
            / len(self._session_stats.frame_times)
        )

        logger.info(
            f"Frame {self._frame_count} stitched | "
            f"features={stats.num_features_detected} matches={stats.num_matches} "
            f"inliers={inlier_count} ({stats.inlier_ratio:.0%}) | "
            f"detect={stats.time_detect_ms:.0f}ms match={stats.time_match_ms:.0f}ms "
            f"transform={stats.time_transform_ms:.0f}ms blend={stats.time_warp_blend_ms:.0f}ms | "
            f"total={stats.time_total_ms:.0f}ms"
        )

        return self.canvas.get_mosaic()

    def process_file(self, image_path: str) -> Optional[np.ndarray]:
        """Process an image file with GPS from EXIF metadata.

        Args:
            image_path: Path to image file (JPEG with GPS EXIF tags).

        Returns:
            Updated mosaic image or None.

        Raises:
            FileNotFoundError: If image file does not exist.
            ValueError: If image cannot be read or has no GPS metadata.
        """
        path = Path(image_path)
        if not path.exists():
            raise FileNotFoundError(f"Image not found: {image_path}")

        # Read image
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"Failed to read image: {image_path}")

        # Extract GPS from EXIF
        try:
            gps = extract_gps_exif(str(path))
        except ValueError as e:
            logger.warning(f"No GPS in {path.name}: {e}")
            raise

        return self.process_frame(image, gps)

    def get_mosaic(self) -> np.ndarray:
        """Return the current mosaic image.

        Returns:
            BGR mosaic image as np.ndarray (uint8).
        """
        return self.canvas.get_mosaic()

    def save_mosaic(self, filename: str = "mosaic_latest.png") -> str:
        """Save current mosaic to disk.

        Args:
            filename: Output filename.

        Returns:
            Full path to saved file.
        """
        mosaic = self.get_mosaic()
        output_path = self._output_dir / filename
        cv2.imwrite(str(output_path), mosaic)
        logger.info(f"Mosaic saved: {output_path} ({mosaic.shape[1]}x{mosaic.shape[0]})")
        return str(output_path)

    def get_stats(self) -> Dict:
        """Return session statistics.

        Returns:
            Dictionary with timing and quality metrics.
        """
        return {
            "total_received": self._session_stats.total_frames_received,
            "total_accepted": self._session_stats.total_frames_accepted,
            "total_rejected": self._session_stats.total_frames_rejected,
            "total_failed": self._session_stats.total_frames_failed,
            "avg_time_ms": self._session_stats.avg_time_per_frame_ms,
            "canvas_size": self._session_stats.canvas_size,
            "filter_stats": self.filter.stats,
            "feature_cache_size": len(self.engine.cache),
        }

    def reset(self):
        """Reset all state for a new session."""
        self.filter.reset()
        self.engine.clear_cache()
        self.canvas.reset()
        self._H_global = np.eye(3, dtype=np.float64)
        self._H_prev_rel = np.eye(3, dtype=np.float64)
        self._H_global_per_frame = {}
        self._origin_gps = None
        self._prev_gps = None
        self._frame_count = 0
        self._accepted_count = 0
        self._failed_count = 0
        self._session_stats = SessionStats()
        logger.info("LiveStitcher reset — ready for new session")
