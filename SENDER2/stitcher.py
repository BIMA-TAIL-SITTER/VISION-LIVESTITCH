"""Per-UAV sequential state, transactional registration, bounded keyframe recovery."""
from collections import deque
from dataclasses import dataclass
import cv2
import numpy as np
from .canvas import MosaicCanvas, CanvasLimit
from .geometry import Preprocessor, estimate_registration


@dataclass
class Keyframe:
    frame_id: int
    points: np.ndarray
    descriptors: np.ndarray
    shape: tuple
    H: np.ndarray


class LiveEngine:
    def __init__(self, config, calibration=None, on_segment_end=None):
        self.cfg = config
        self.preprocessor = Preprocessor(config, calibration)
        self.sift = cv2.SIFT_create(nfeatures=config.sift_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self.on_segment_end = on_segment_end
        self.canvas = MosaicCanvas(config.max_canvas_pixels)
        self.cache = deque(maxlen=config.keyframe_cache)
        self.segment = 0
        self.failures = 0
        self.epoch = None
        self.geometry_signature = None
        self.last_frame = None
        self.segment_start_reason = "initial"

    def _new_segment(self, reason):
        if self.canvas.color is not None and self.on_segment_end:
            self.on_segment_end(self, reason)
        if self.canvas.color is not None:
            self.segment += 1
        self.canvas = MosaicCanvas(self.cfg.max_canvas_pixels)
        self.cache.clear()
        self.failures = 0
        self.last_frame = None
        self.segment_start_reason = reason

    def _gate(self, meta):
        t = meta.telemetry
        if t.attitude is None:
            return "attitude_missing" if self.cfg.require_attitude else None
        if t.attitude_age_ms is None or t.attitude_age_ms > self.cfg.max_telemetry_age_ms:
            return "attitude_stale"
        if abs(t.attitude.roll_deg) > self.cfg.max_roll_deg or abs(t.attitude.pitch_deg) > self.cfg.max_pitch_deg:
            return "attitude_exceeds_gate"
        return None

    def process(self, image, meta):
        reason = self._gate(meta)
        if reason:
            return {"status":"rejected", "reason":reason}
        working, mask = self.preprocessor.prepare(image, meta)
        gray = cv2.cvtColor(working, cv2.COLOR_BGR2GRAY)
        blur = float(cv2.Laplacian(gray, cv2.CV_64F)[mask>0].var())
        if blur < self.cfg.blur_threshold:
            return {"status":"rejected", "reason":"blur", "blur_score":blur}
        kp, descriptors = self.sift.detectAndCompute(gray, mask)
        if descriptors is None or len(kp) < self.cfg.min_inliers:
            # A textureless first frame must not poison the reference cache.
            return {"status":"rejected", "reason":"insufficient_features"}
        points = np.float32([p.pt for p in kp])
        signature = (meta.camera_id, meta.calibration_id, working.shape[:2])
        if self.epoch is not None and str(meta.stream_epoch) != self.epoch:
            self._new_segment("sender_restart")
        elif self.geometry_signature is not None and signature != self.geometry_signature:
            self._new_segment("camera_geometry_changed")
        self.epoch, self.geometry_signature = str(meta.stream_epoch), signature
        best = None
        # Every candidate gets geometric validation; a bad largest match set does not block recovery.
        for cached in reversed(self.cache):
            raw = self.matcher.knnMatch(descriptors, cached.descriptors, k=2)
            good = [pair[0] for pair in raw if len(pair)==2 and pair[0].distance < self.cfg.ratio_test*pair[1].distance]
            unique = {}
            for match in sorted(good, key=lambda m:m.distance):
                unique.setdefault(match.trainIdx, match)
            good = list(unique.values())
            if len(good) < self.cfg.min_inliers:
                continue
            registration = estimate_registration(points[[m.queryIdx for m in good]],
                cached.points[[m.trainIdx for m in good]], working.shape, cached.shape, self.cfg)
            if registration is None:
                continue
            H = cached.H @ registration.H
            global_scale = float(np.sqrt(np.linalg.det(H[:2,:2])))
            if not np.isfinite(H).all() or not 0.4 <= global_scale <= 2.5:
                continue
            score = registration.inliers/(1+registration.median_px)
            if best is None or score > best[0]:
                best = score, H, registration, cached.frame_id
        details = {}
        if not self.cache:
            H = np.eye(3)
        elif best:
            _, H, registration, reference = best
            details = {"registration":registration.metrics(), "reference_frame_id":reference}
        else:
            self.failures += 1
            if self.failures < self.cfg.recovery_failures:
                return {"status":"registration_failed", "reason":"no_valid_keyframe_match", "consecutive_failures":self.failures}
            self._new_segment("registration_lost")
            H = np.eye(3)
        try:
            self.canvas.add(working, mask, H)
        except CanvasLimit:
            self._new_segment("canvas_budget")
            H = np.eye(3)
            details = {}
            self.canvas.add(working, mask, H)
        # Commit only after the canvas accepted this frame.
        self.cache.append(Keyframe(meta.frame_id, points, descriptors, working.shape, H.copy()))
        self.last_frame = meta
        self.failures = 0
        return {"status":"stitched", "segment":self.segment, "segment_start_reason":self.segment_start_reason,
                "H_working_to_segment":H.tolist(), "working_size":[working.shape[1], working.shape[0]],
                "canvas_origin":list(self.canvas.origin), "canvas_shape":self.canvas.shape,
                "calibrated":self.preprocessor.calibration is not None,
                "georeferenced":False, "blur_score":blur, **details}
