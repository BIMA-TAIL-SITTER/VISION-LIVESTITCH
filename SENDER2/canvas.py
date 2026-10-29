"""Bounded ROI accumulator with explicit geometric coverage, including black pixels."""
import cv2
import numpy as np


class CanvasLimit(ValueError):
    pass


class MosaicCanvas:
    def __init__(self, max_pixels):
        self.max_pixels = max_pixels
        self.color = None
        self.weight = None
        self.origin = (0, 0)

    def add(self, image, valid_mask, H):
        h, w = image.shape[:2]
        if not np.isfinite(H).all():
            raise ValueError("nonfinite transform")
        corners = cv2.perspectiveTransform(np.float32([[[0,0]], [[w,0]], [[w,h]], [[0,h]]]), H)[:,0]
        lo = np.floor(corners.min(axis=0)).astype(int)
        hi = np.ceil(corners.max(axis=0)).astype(int)
        roi_w, roi_h = hi-lo
        if roi_w <= 0 or roi_h <= 0 or roi_w*roi_h > self.max_pixels:
            raise CanvasLimit("frame footprint exceeds canvas budget")
        low, high = lo.copy(), hi.copy()
        if self.color is not None:
            low = np.minimum(low, self.origin)
            high = np.maximum(high, np.array(self.origin)+[self.color.shape[1], self.color.shape[0]])
        width, height = high-low
        if width*height > self.max_pixels:
            raise CanvasLimit("segment canvas budget reached")
        # Warp weight and weighted colors in the same interpolation domain.
        padded = np.pad((valid_mask > 0).astype(np.uint8), 1)
        distance = cv2.distanceTransform(padded, cv2.DIST_L2, 3)[1:-1,1:-1]
        weight = np.minimum(distance/20, 1).astype(np.float32)
        weight[valid_mask == 0] = 0
        T = np.array([[1,0,-lo[0]], [0,1,-lo[1]], [0,0,1]], float)
        warped_weight = cv2.warpPerspective(weight, T@H, (int(roi_w),int(roi_h)))
        warped_color = cv2.warpPerspective(image.astype(np.float32)*weight[:,:,None], T@H,
                                           (int(roi_w),int(roi_h)))
        # Allocate only after all limits and warps succeeded. Existing state stays valid on rejection.
        if self.color is None or tuple(low) != self.origin or self.color.shape[:2] != (height,width):
            color = np.zeros((height,width,3), np.float32)
            weights = np.zeros((height,width), np.float32)
            if self.color is not None:
                ox, oy = np.array(self.origin)-low
                oh, ow = self.weight.shape
                color[oy:oy+oh, ox:ox+ow] = self.color
                weights[oy:oy+oh, ox:ox+ow] = self.weight
            self.color, self.weight, self.origin = color, weights, tuple(int(x) for x in low)
        x, y = lo-np.array(self.origin)
        self.color[y:y+roi_h, x:x+roi_w] += warped_color
        self.weight[y:y+roi_h, x:x+roi_w] += warped_weight

    def render(self, max_side=None):
        if self.color is None:
            return None
        color, weight = self.color, self.weight
        if max_side and max(color.shape[:2]) > max_side:
            factor = max_side/max(color.shape[:2])
            size = (max(1,round(color.shape[1]*factor)), max(1,round(color.shape[0]*factor)))
            color = cv2.resize(color, size, interpolation=cv2.INTER_AREA)
            weight = cv2.resize(weight, size, interpolation=cv2.INTER_AREA)
        return np.clip(color/np.maximum(weight[:,:,None], 1e-6), 0, 255).astype(np.uint8)

    @property
    def shape(self):
        return list(self.weight.shape) if self.weight is not None else [0,0]
