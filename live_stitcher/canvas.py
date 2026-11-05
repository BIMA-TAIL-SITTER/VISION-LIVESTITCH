"""
Incremental Canvas with Multi-Band Blending & Exposure Compensation.

Key improvements over v1:
- Laplacian pyramid blending in overlap ROI (eliminates ghosting)
- Exposure compensation (eliminates visible seam lines)
- Distance-transform feather with power=3.0 (sharper transitions)

Performance: All blending operations are restricted to the overlap
bounding box ROI — NOT the full canvas — keeping per-frame time O(1).
"""

import logging
from typing import Optional, Tuple

import cv2
import numpy as np

from .config import StitchConfig

logger = logging.getLogger(__name__)


class IncrementalCanvas:
    """Accumulator-based canvas that grows incrementally.
    
    Key Design: Instead of re-warping the entire mosaic each time a new image
    is added, we maintain weighted color and weight accumulator buffers.
    New images are warped and blended INTO the existing accumulators,
    making per-frame time CONSTANT regardless of mosaic size.
    
    Blending uses multi-band (Laplacian pyramid) or distance-transform
    feather weights in the overlap region only, for efficient and seamless
    transitions.
    """
    
    def __init__(self, config: StitchConfig):
        self.padding = config.canvas_initial_padding
        self.expand_margin = config.canvas_expand_margin
        self.feather_power = config.blend_feather_power
        
        # Blending quality settings
        self.enable_exposure_comp = getattr(config, 'enable_exposure_compensation', True)
        self.enable_multiband = getattr(config, 'enable_multiband_blend', True)
        self.multiband_levels = getattr(config, 'multiband_levels', 2)
        self.exposure_gain_limit = getattr(config, 'exposure_gain_limit', 1.4)
        
        # Accumulator buffers (initialized on first image)
        self.color_acc: Optional[np.ndarray] = None   # float32 (H, W, 3) weighted colors
        self.weight_acc: Optional[np.ndarray] = None   # float32 (H, W, 1) total weights
        
        # Canvas coordinate system
        self.origin_x: float = 0.0  # x offset: canvas pixel 0 corresponds to world x = origin_x
        self.origin_y: float = 0.0
        self.canvas_w: int = 0
        self.canvas_h: int = 0
        
        self._frame_count = 0
    
    def _get_translation_matrix(self) -> np.ndarray:
        """Get 3x3 translation matrix that maps world coords to canvas pixel coords."""
        T = np.eye(3, dtype=np.float64)
        T[0, 2] = -self.origin_x
        T[1, 2] = -self.origin_y
        return T
    
    def initialize(self, first_image: np.ndarray):
        """Initialize canvas with the first image centered with padding."""
        h, w = first_image.shape[:2]
        
        self.canvas_h = h + 2 * self.padding
        self.canvas_w = w + 2 * self.padding
        self.origin_x = -self.padding
        self.origin_y = -self.padding
        
        self.color_acc = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.float32)
        self.weight_acc = np.zeros((self.canvas_h, self.canvas_w, 1), dtype=np.float32)
        
        # Place first image at center (with padding offset)
        py, px = self.padding, self.padding
        self.color_acc[py:py+h, px:px+w] = first_image.astype(np.float32)
        self.weight_acc[py:py+h, px:px+w] = 1.0
        
        self._frame_count = 1
        logger.info(f"Canvas initialized: {self.canvas_w}x{self.canvas_h} (image: {w}x{h})")
    
    def add_image(self, image: np.ndarray, H_global: np.ndarray):
        """Add a new image to the canvas using ROI-based warping.
        
        Steps:
        1. Compose H_global with canvas translation to get H_canvas
        2. Compute bounding box of transformed image in canvas space
        3. Expand canvas if needed (pad accumulators, DON'T re-warp)
        4. Warp new image into ROI-sized buffer (small, constant size)
        5. Apply exposure compensation in overlap region
        6. Insert/blend ROI region into canvas using multi-band or feather
        """
        if self.color_acc is None:
            self.initialize(image)
            return True
        
        h, w = image.shape[:2]
        T = self._get_translation_matrix()
        H_canvas = T @ H_global  # Transform: image world -> canvas pixels
        
        # Compute bounding box of transformed image in canvas space
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        transformed_corners = cv2.perspectiveTransform(corners, H_canvas)
        
        tc_x = transformed_corners[:, 0, 0]
        tc_y = transformed_corners[:, 0, 1]
        
        need_expand = (
            tc_x.min() < 0 or tc_y.min() < 0 or
            tc_x.max() > self.canvas_w or tc_y.max() > self.canvas_h
        )
        
        if need_expand:
            # Guard: reject if expansion in any direction > 4× image size
            expand_left  = max(0, -tc_x.min())
            expand_top   = max(0, -tc_y.min())
            expand_right = max(0, tc_x.max() - self.canvas_w)
            expand_bot   = max(0, tc_y.max() - self.canvas_h)
            max_expand = max(expand_left, expand_top, expand_right, expand_bot)
            if max_expand > max(h, w) * 4.0:
                logger.warning(
                    f"Canvas expansion REJECTED: {max_expand:.0f}px > 4× image "
                    f"({max(h, w) * 4:.0f}px). Transform likely degenerate."
                )
                return False

            self._expand_canvas(tc_x.min(), tc_y.min(), tc_x.max(), tc_y.max())
            # Recompute H_canvas after expansion
            T = self._get_translation_matrix()
            H_canvas = T @ H_global
            # Recompute corners
            transformed_corners = cv2.perspectiveTransform(corners, H_canvas)
            tc_x = transformed_corners[:, 0, 0]
            tc_y = transformed_corners[:, 0, 1]
        
        # --- ROI-based warping (key optimization) ---
        roi_margin = 5
        roi_x1 = max(0, int(np.floor(tc_x.min())) - roi_margin)
        roi_y1 = max(0, int(np.floor(tc_y.min())) - roi_margin)
        roi_x2 = min(self.canvas_w, int(np.ceil(tc_x.max())) + roi_margin)
        roi_y2 = min(self.canvas_h, int(np.ceil(tc_y.max())) + roi_margin)
        roi_w = roi_x2 - roi_x1
        roi_h = roi_y2 - roi_y1
        
        if roi_w <= 0 or roi_h <= 0:
            logger.warning("ROI has zero size, skipping frame")
            return False
        
        # Offset H_canvas so that the ROI top-left maps to (0,0) in the ROI buffer
        T_roi = np.eye(3, dtype=np.float64)
        T_roi[0, 2] = -roi_x1
        T_roi[1, 2] = -roi_y1
        H_roi = T_roi @ H_canvas
        
        # Warp into ROI-sized buffer (MUCH smaller than full canvas)
        warped_roi = cv2.warpPerspective(
            image, H_roi,
            (roi_w, roi_h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        
        # Create mask for warped image (non-black pixels)
        warped_gray = cv2.cvtColor(warped_roi, cv2.COLOR_BGR2GRAY)
        new_mask = (warped_gray > 0).astype(np.uint8)
        
        # Extract corresponding ROI from canvas accumulators
        canvas_color_roi = self.color_acc[roi_y1:roi_y2, roi_x1:roi_x2]
        canvas_weight_roi = self.weight_acc[roi_y1:roi_y2, roi_x1:roi_x2]
        existing_mask = (canvas_weight_roi[:, :, 0] > 0).astype(np.uint8)
        
        # Overlap and new-only masks
        overlap_mask = new_mask & existing_mask
        new_only_mask = new_mask & (~existing_mask.astype(bool)).astype(np.uint8)
        
        # --- Exposure Compensation (before blending) ---
        if self.enable_exposure_comp and overlap_mask.any():
            warped_roi = self._compensate_exposure(
                warped_roi, new_mask, canvas_color_roi, existing_mask
            )
        
        # --- Direct insert for non-overlapping pixels ---
        if new_only_mask.any():
            mask3 = np.stack([new_only_mask] * 3, axis=-1).astype(bool)
            canvas_color_roi[mask3] = warped_roi.astype(np.float32)[mask3]
            canvas_weight_roi[new_only_mask.astype(bool), 0] = 1.0
        
        # --- Blend overlapping pixels ---
        if overlap_mask.any():
            if self.enable_multiband:
                self._multiband_blend_roi(
                    warped_roi, new_mask, existing_mask, overlap_mask,
                    canvas_color_roi,
                )
            else:
                self._feather_blend_roi(
                    warped_roi, new_mask, existing_mask, overlap_mask,
                    canvas_color_roi,
                )
            canvas_weight_roi[overlap_mask.astype(bool), 0] = 1.0
        
        self._frame_count += 1
        logger.debug(
            f"Frame {self._frame_count} added (ROI: {roi_w}x{roi_h} in "
            f"canvas {self.canvas_w}x{self.canvas_h})"
        )
        return True
    
    # ─── Exposure Compensation ──────────────────────────────────────────
    
    def _compensate_exposure(
        self,
        new_image: np.ndarray,
        new_mask: np.ndarray,
        existing_roi: np.ndarray,
        existing_mask: np.ndarray,
    ) -> np.ndarray:
        """Match brightness of new image to existing canvas in overlap region.
        
        Computes per-channel gain from mean pixel values in the overlap,
        then applies the gain to the entire new image. Gain is clamped
        to prevent over-correction from small or unrepresentative overlaps.
        """
        overlap = (new_mask > 0) & (existing_mask > 0)
        overlap_count = overlap.sum()
        
        if overlap_count < 200:
            return new_image  # Too few pixels for reliable estimation
        
        # Compute per-channel mean in overlap region
        new_float = new_image.astype(np.float32)
        exist_float = existing_roi  # Already float32
        
        gains = np.ones(3, dtype=np.float32)
        for c in range(3):
            new_mean = new_float[overlap, c].mean()
            exist_mean = exist_float[overlap, c].mean()
            
            if new_mean > 5.0:  # Avoid division by near-zero
                g = exist_mean / new_mean
                gains[c] = np.clip(g, 1.0 / self.exposure_gain_limit, 
                                   self.exposure_gain_limit)
        
        # Apply gain to entire image
        result = new_image.astype(np.float32)
        for c in range(3):
            result[:, :, c] *= gains[c]
        
        result = np.clip(result, 0, 255).astype(np.uint8)
        
        avg_gain = gains.mean()
        if abs(avg_gain - 1.0) > 0.05:
            logger.debug(f"Exposure compensation: gain={gains} (avg={avg_gain:.3f})")
        
        return result
    
    # ─── Multi-Band (Laplacian Pyramid) Blending ────────────────────────
    
    def _multiband_blend_roi(
        self,
        warped_new: np.ndarray,
        new_mask: np.ndarray,
        existing_mask: np.ndarray,
        overlap_mask: np.ndarray,
        canvas_color_roi: np.ndarray,
    ):
        """Apply Laplacian pyramid blending within the overlap ROI.
        
        This combines the sharpness of multi-band blending (no ghosting)
        with the efficiency of ROI-restricted processing (O(1) per frame).
        
        Steps:
        1. Find tight bounding box of overlap region
        2. Build distance-transform blend mask (like feather but for pyramid)
        3. Build Laplacian pyramids for both images in overlap
        4. Blend pyramids using the mask
        5. Reconstruct and write back to canvas
        """
        coords = cv2.findNonZero(overlap_mask)
        if coords is None:
            return
        x, y, bw, bh = cv2.boundingRect(coords)
        
        # Minimum size for pyramid blending (must be >= 2^levels)
        min_dim = 2 ** (self.multiband_levels + 1)
        if bw < min_dim or bh < min_dim:
            # Fall back to feather blend for very small overlaps
            self._feather_blend_roi(
                warped_new, new_mask, existing_mask, overlap_mask,
                canvas_color_roi,
            )
            return
        
        # --- Extract overlap sub-ROI ---
        sub_existing = canvas_color_roi[y:y+bh, x:x+bw].copy()  # float32
        sub_new = warped_new[y:y+bh, x:x+bw].astype(np.float32)
        
        # --- Build smooth blend mask from distance transforms ---
        roi_new_mask = new_mask[y:y+bh, x:x+bw].astype(np.uint8) * 255
        roi_exist_mask = existing_mask[y:y+bh, x:x+bw].astype(np.uint8) * 255
        
        dist_new = cv2.distanceTransform(roi_new_mask, cv2.DIST_L2, 3).astype(np.float32)
        dist_exist = cv2.distanceTransform(roi_exist_mask, cv2.DIST_L2, 3).astype(np.float32)
        
        total = dist_new + dist_exist + 1e-6
        # Alpha for the NEW image (how much of the new image to use)
        alpha_new = dist_new / total
        # Apply power curve for sharper transition
        alpha_new = alpha_new ** self.feather_power
        # Re-normalize
        alpha_exist = (1.0 - alpha_new)
        
        # Smooth the mask for pyramid blending
        mask_smooth = cv2.GaussianBlur(alpha_new, (21, 21), 0)
        
        # --- Laplacian Pyramid Blending ---
        # Ensure dimensions are even for pyrDown
        bh_even = bh - (bh % 2)
        bw_even = bw - (bw % 2)
        if bh_even < min_dim or bw_even < min_dim:
            self._feather_blend_roi(
                warped_new, new_mask, existing_mask, overlap_mask,
                canvas_color_roi,
            )
            return
        
        img_A = sub_existing[:bh_even, :bw_even]  # existing canvas
        img_B = sub_new[:bh_even, :bw_even]        # new image
        mask_pyr_base = mask_smooth[:bh_even, :bw_even]
        
        blended = self._laplacian_blend(img_A, img_B, mask_pyr_base, self.multiband_levels)
        
        # Write blended result back
        overlap_sub = overlap_mask[y:y+bh_even, x:x+bw_even].astype(bool)
        overlap_3 = np.stack([overlap_sub] * 3, axis=-1)
        
        canvas_sub = canvas_color_roi[y:y+bh_even, x:x+bw_even]
        canvas_sub[overlap_3] = blended[overlap_3]
    
    def _laplacian_blend(
        self,
        img_A: np.ndarray,  # float32 (H, W, 3) — existing
        img_B: np.ndarray,  # float32 (H, W, 3) — new
        mask: np.ndarray,   # float32 (H, W) — blend mask [0=A, 1=B]
        levels: int = 2,
    ) -> np.ndarray:
        """Blend two images using Laplacian pyramid.
        
        Operates per-channel to build Gaussian/Laplacian pyramids,
        blend at each level using the mask pyramid, and reconstruct.
        
        Args:
            img_A: Background image (existing canvas region). float32.
            img_B: Foreground image (new warped image). float32.
            mask: Blend mask [0.0 = img_A, 1.0 = img_B]. float32.
            levels: Number of pyramid levels.
            
        Returns:
            Blended image as float32.
        """
        h, w = img_A.shape[:2]
        result = np.zeros_like(img_A)
        
        # Build mask Gaussian pyramid
        mask_pyr = [mask.copy()]
        m = mask.copy()
        for _ in range(levels):
            m = cv2.pyrDown(m)
            mask_pyr.append(m)
        
        for c in range(3):
            ch_A = img_A[:, :, c]
            ch_B = img_B[:, :, c]
            
            # Gaussian pyramids
            gp_A = [ch_A.copy()]
            gp_B = [ch_B.copy()]
            ga, gb = ch_A.copy(), ch_B.copy()
            for _ in range(levels):
                ga = cv2.pyrDown(ga)
                gb = cv2.pyrDown(gb)
                gp_A.append(ga)
                gp_B.append(gb)
            
            # Laplacian pyramids
            lp_A = [gp_A[-1]]
            lp_B = [gp_B[-1]]
            for i in range(levels, 0, -1):
                size = (gp_A[i-1].shape[1], gp_A[i-1].shape[0])
                la = gp_A[i-1] - cv2.pyrUp(gp_A[i], dstsize=size)
                lb = gp_B[i-1] - cv2.pyrUp(gp_B[i], dstsize=size)
                lp_A.append(la)
                lp_B.append(lb)
            
            # Blend each pyramid level using corresponding mask level
            blended_pyr = []
            for i, (la, lb) in enumerate(zip(lp_A, lp_B)):
                # Resize mask to match pyramid level
                mp = cv2.resize(mask_pyr[levels - i] if i < len(mask_pyr) else mask_pyr[-1],
                               (la.shape[1], la.shape[0]))
                blended = la * (1.0 - mp) + lb * mp
                blended_pyr.append(blended)
            
            # Reconstruct from blended Laplacian pyramid
            reconstructed = blended_pyr[0]
            for i in range(1, len(blended_pyr)):
                size = (blended_pyr[i].shape[1], blended_pyr[i].shape[0])
                reconstructed = cv2.pyrUp(reconstructed, dstsize=size) + blended_pyr[i]
            
            result[:, :, c] = np.clip(reconstructed, 0, 255)
        
        return result
    
    # ─── Feather Blending (fallback) ────────────────────────────────────

    def _feather_blend_roi(
        self,
        warped_new: np.ndarray,
        new_mask: np.ndarray,
        existing_mask: np.ndarray,
        overlap_mask: np.ndarray,
        canvas_color_roi: np.ndarray,
    ):
        """Apply distance-transform feather blending within the ROI.
        
        Fallback when overlap is too small for multi-band pyramid.
        """
        # Find tight bounding box of overlap within the ROI
        coords = cv2.findNonZero(overlap_mask)
        if coords is None:
            return
        x, y, bw, bh = cv2.boundingRect(coords)
        
        # Crop to overlap bounding box
        roi_new_mask = new_mask[y:y+bh, x:x+bw].astype(np.uint8) * 255
        roi_exist_mask = existing_mask[y:y+bh, x:x+bw].astype(np.uint8) * 255
        
        # Distance transforms
        dist_new = cv2.distanceTransform(roi_new_mask, cv2.DIST_L2, 3).astype(np.float32)
        dist_exist = cv2.distanceTransform(roi_exist_mask, cv2.DIST_L2, 3).astype(np.float32)
        
        # Feather weights with power curve
        total = dist_new + dist_exist + 1e-6
        alpha_exist = (dist_exist / total) ** self.feather_power
        alpha_new = (dist_new / total) ** self.feather_power
        
        # Normalize so they sum to 1
        alpha_sum = alpha_exist + alpha_new + 1e-6
        alpha_exist /= alpha_sum
        alpha_new /= alpha_sum
        
        # Blend in overlap sub-ROI
        sub_existing = canvas_color_roi[y:y+bh, x:x+bw]
        sub_new = warped_new[y:y+bh, x:x+bw].astype(np.float32)
        
        alpha_exist_3 = np.stack([alpha_exist] * 3, axis=-1)
        alpha_new_3 = np.stack([alpha_new] * 3, axis=-1)
        
        overlap_sub = overlap_mask[y:y+bh, x:x+bw].astype(bool)
        overlap_3 = np.stack([overlap_sub] * 3, axis=-1)
        
        blended = alpha_exist_3 * sub_existing + alpha_new_3 * sub_new
        sub_existing[overlap_3] = blended[overlap_3]
    
    # ─── Canvas Expansion ───────────────────────────────────────────────
    
    def _expand_canvas(self, min_x: float, min_y: float, max_x: float, max_y: float):
        """Expand canvas to fit new bounds. Pads existing accumulators WITHOUT re-warping."""
        margin = self.expand_margin
        
        pad_left = max(0, int(-min_x + margin))
        pad_top = max(0, int(-min_y + margin))
        pad_right = max(0, int(max_x - self.canvas_w + margin))
        pad_bottom = max(0, int(max_y - self.canvas_h + margin))
        
        if pad_left == 0 and pad_top == 0 and pad_right == 0 and pad_bottom == 0:
            return
        
        logger.info(f"Expanding canvas: L={pad_left} T={pad_top} R={pad_right} B={pad_bottom}")
        
        # Pad accumulators
        self.color_acc = np.pad(
            self.color_acc,
            ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
            mode='constant', constant_values=0,
        )
        self.weight_acc = np.pad(
            self.weight_acc,
            ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
            mode='constant', constant_values=0,
        )
        
        # Update origin and dimensions
        self.origin_x -= pad_left
        self.origin_y -= pad_top
        self.canvas_w = self.color_acc.shape[1]
        self.canvas_h = self.color_acc.shape[0]
    
    # ─── Mosaic Extraction ──────────────────────────────────────────────
    
    def get_mosaic(self) -> np.ndarray:
        """Return current mosaic as uint8 BGR image.
        
        Memory-efficient: crops the float32 accumulator FIRST using the
        weight mask (cheap, single-channel), then clips and converts ONLY
        the cropped region.
        """
        if self.color_acc is None:
            return np.zeros((100, 100, 3), dtype=np.uint8)
        
        # Step 1: Find bounding box from weight_acc (cheap single-channel)
        weight_mask = (self.weight_acc[:, :, 0] > 0).astype(np.uint8)
        coords = cv2.findNonZero(weight_mask)
        if coords is None:
            return np.zeros((100, 100, 3), dtype=np.uint8)
        
        x, y, w, h = cv2.boundingRect(coords)
        margin = 10
        x = max(0, x - margin)
        y = max(0, y - margin)
        x2 = min(self.color_acc.shape[1], x + w + 2 * margin)
        y2 = min(self.color_acc.shape[0], y + h + 2 * margin)
        
        # Step 2: Slice ONLY the content region (no full-canvas copy)
        cropped_float = self.color_acc[y:y2, x:x2]
        
        # Step 3: Clip in-place on the slice, then convert to uint8
        np.clip(cropped_float, 0, 255, out=cropped_float)
        mosaic = cropped_float.astype(np.uint8)
        
        logger.debug(f"Mosaic: {mosaic.shape[1]}x{mosaic.shape[0]} "
                     f"from canvas {self.canvas_w}x{self.canvas_h}")
        return mosaic
    
    @property
    def frame_count(self) -> int:
        return self._frame_count
    
    def reset(self):
        self.color_acc = None
        self.weight_acc = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.canvas_w = 0
        self.canvas_h = 0
        self._frame_count = 0
        logger.info("Canvas reset")
