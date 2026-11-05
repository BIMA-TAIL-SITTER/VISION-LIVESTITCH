"""Calibrated preprocessing and post-fit validated 2-D registration.

Attitude is a quality gate only, never an uncalibrated pixel rotation. Similarity
is the default local planar approximation; neither model produces an orthophoto.
"""
from dataclasses import dataclass
from pathlib import Path
import cv2
import numpy as np
from pydantic import Field, model_validator
from .models import StrictModel


class Calibration(StrictModel):
    calibration_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,64}$")
    camera_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    model: str = "opencv_pinhole"
    image_width: int = Field(gt=0)
    image_height: int = Field(gt=0)
    camera_matrix: list[list[float]]
    distortion: list[float]
    rms_error_px: float = Field(ge=0)

    @model_validator(mode="after")
    def matrix_valid(self):
        K = np.asarray(self.camera_matrix, dtype=float)
        if self.model != "opencv_pinhole" or K.shape != (3, 3) or not np.isfinite(K).all():
            raise ValueError("invalid pinhole camera model")
        if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]):
            raise ValueError("invalid intrinsic matrix")
        if len(self.distortion) not in {4, 5, 8, 12, 14}:
            raise ValueError("invalid distortion length")
        return self

    @classmethod
    def load(cls, path):
        return cls.model_validate_json(Path(path).read_text(encoding="utf-8"))


class Preprocessor:
    def __init__(self, config, calibration=None):
        self.config = config
        self.calibration = calibration
        self.maps = {}

    def prepare(self, image, meta):
        h, w = image.shape[:2]
        if (w, h) != (meta.preview_width, meta.preview_height):
            raise ValueError("decoded dimensions differ from metadata")
        if self.config.require_calibration and self.calibration is None:
            raise ValueError("calibration_required")
        scale = min(1.0, self.config.working_width/w)
        size = (max(1, round(w*scale)), max(1, round(h*scale)))
        working = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        mask = np.full(working.shape[:2], 255, np.uint8)
        if self.calibration:
            cal = self.calibration
            if (meta.camera_id, meta.calibration_id) != (cal.camera_id, cal.calibration_id):
                raise ValueError("camera/calibration identity mismatch")
            if (meta.capture_width, meta.capture_height) != (cal.image_width, cal.image_height):
                raise ValueError("capture resolution differs from calibration; recalibrate this mode")
            if abs(w/h - cal.image_width/cal.image_height) > 0.005:
                raise ValueError("preview crop/aspect ratio differs from calibrated mode")
            if size not in self.maps:
                K = np.asarray(cal.camera_matrix, np.float64).copy()
                K[0] *= size[0]/cal.image_width
                K[1] *= size[1]/cal.image_height
                self.maps[size] = cv2.initUndistortRectifyMap(K, np.asarray(cal.distortion), None, K, size, cv2.CV_32FC1)
            mx, my = self.maps[size]
            working = cv2.remap(working, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            mask = cv2.remap(mask, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
        if not np.any(mask):
            raise ValueError("no valid calibrated pixels")
        return working, mask


@dataclass
class Registration:
    H: np.ndarray
    inliers: int
    ratio: float
    median_px: float
    p95_px: float
    coverage: float
    scale: float
    overlap: float

    def metrics(self):
        return {k: v for k, v in self.__dict__.items() if k != "H"}


def fit_rigid(src, dst):
    """Least-squares SE(2) fit, including translation refit around centroids."""
    a, b = src.mean(axis=0), dst.mean(axis=0)
    U, _, Vt = np.linalg.svd((src-a).T @ (dst-b))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    H = np.eye(3)
    H[:2, :2], H[:2, 2] = R, b-R@a
    return H


def estimate_registration(src, dst, src_shape, dst_shape, cfg):
    src, dst = np.asarray(src, np.float32), np.asarray(dst, np.float32)
    if len(src) < cfg.min_inliers:
        return None
    M, mask = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC,
        ransacReprojThreshold=cfg.ransac_px, maxIters=3000, confidence=0.995, refineIters=10)
    if M is None or mask is None or not np.isfinite(M).all():
        return None
    candidates = mask.ravel().astype(bool)
    if candidates.sum() < cfg.min_inliers:
        return None
    H = np.vstack([M, [0, 0, 1]])
    if cfg.model == "rigid":
        H = fit_rigid(src[candidates], dst[candidates])
    # Validate the ACTUAL returned model after all modifications.
    pred = cv2.perspectiveTransform(src[:, None, :], H)[:, 0]
    residual = np.linalg.norm(pred-dst, axis=1)
    keep = residual <= cfg.ransac_px
    count = int(keep.sum())
    if count < cfg.min_inliers or count/len(src) < cfg.min_inlier_ratio:
        return None
    median, p95 = float(np.median(residual[keep])), float(np.percentile(residual[keep], 95))
    if median > cfg.max_median_error_px or p95 > cfg.max_p95_error_px:
        return None
    scale = float(np.sqrt(np.linalg.det(H[:2, :2])))
    if not cfg.min_scale <= scale <= cfg.max_scale:
        return None
    coverage = min(abs(cv2.contourArea(cv2.convexHull(src[keep])))/(src_shape[0]*src_shape[1]),
                   abs(cv2.contourArea(cv2.convexHull(dst[keep])))/(dst_shape[0]*dst_shape[1]))
    if coverage < cfg.min_feature_coverage:
        return None
    h, w = src_shape[:2]
    dh, dw = dst_shape[:2]
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    warped = cv2.perspectiveTransform(corners[:, None], H)[:, 0]
    area, _ = cv2.intersectConvexConvex(warped.astype(np.float32),
        np.float32([[0, 0], [dw, 0], [dw, dh], [0, dh]]))
    overlap = float(area / max(1, min(abs(cv2.contourArea(warped)), dw*dh)))
    if overlap < cfg.min_overlap:
        return None
    return Registration(H, count, count/len(src), median, p95, float(coverage), scale, overlap)
