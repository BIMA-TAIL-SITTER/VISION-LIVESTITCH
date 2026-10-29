"""Versioned wire contract. Unknown measurements remain null."""
from typing import Literal
from uuid import UUID
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, validate_assignment=True)


class Position(StrictModel):
    latitude_deg: float = Field(ge=-90, le=90)
    longitude_deg: float = Field(ge=-180, le=180)
    altitude_msl_m: float | None = None
    altitude_relative_home_m: float | None = None
    altitude_agl_m: float | None = None
    horizontal_accuracy_m: float | None = Field(default=None, ge=0)
    fix_type: int | None = Field(default=None, ge=0, le=8)


class Attitude(StrictModel):
    roll_deg: float = Field(ge=-180, le=180)
    pitch_deg: float = Field(ge=-90, le=90)
    yaw_deg: float = Field(ge=-180, le=360)
    convention: Literal["body_FRD_to_NED_euler_deg"] = "body_FRD_to_NED_euler_deg"


class Telemetry(StrictModel):
    position: Position | None = None
    attitude: Attitude | None = None
    position_age_ms: float | None = Field(default=None, ge=0)
    attitude_age_ms: float | None = Field(default=None, ge=0)
    synchronization: Literal["host_receive_time", "none"] = "none"
    source_system: int | None = Field(default=None, ge=1, le=255)


class FrameMeta(StrictModel):
    schema_version: Literal[2] = 2
    mission_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    uav_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    stream_epoch: UUID
    stream_started_unix_ns: int = Field(gt=0)
    frame_id: int = Field(ge=0, lt=2**63)
    capture_timestamp_unix_ns: int = Field(gt=0)
    capture_timestamp_monotonic_ns: int = Field(ge=0)
    capture_clock: Literal["sender_host_utc_unverified"] = "sender_host_utc_unverified"
    capture_time_source: Literal["host_read_completed", "replay_read_completed", "hardware_exposure"]
    capture_uncertainty_ms: float | None = Field(default=None, ge=0)
    camera_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    calibration_id: str | None = Field(default=None, pattern=r"^[A-Za-z0-9_-]{1,64}$")
    capture_width: int = Field(gt=0, le=20000)
    capture_height: int = Field(gt=0, le=20000)
    preview_width: int = Field(gt=0, le=4096)
    preview_height: int = Field(gt=0, le=4096)
    preview_encoding: Literal["jpeg"] = "jpeg"
    telemetry: Telemetry = Field(default_factory=Telemetry)
    simulation: bool = False
    source_filename: str | None = Field(default=None, max_length=255)

    @model_validator(mode="after")
    def dimensions(self):
        if self.preview_width * self.preview_height > 8_000_000:
            raise ValueError("preview exceeds 8 megapixels")
        return self

    @property
    def key(self):
        return self.mission_id, self.uav_id, str(self.stream_epoch), self.frame_id


class EngineConfig(StrictModel):
    working_width: int = Field(default=960, ge=160, le=2048)
    sift_features: int = Field(default=2500, ge=100, le=10000)
    ratio_test: float = Field(default=0.7, gt=0, lt=1)
    min_inliers: int = Field(default=16, ge=4)
    min_inlier_ratio: float = Field(default=0.45, gt=0, le=1)
    max_median_error_px: float = Field(default=1.8, gt=0)
    max_p95_error_px: float = Field(default=3.5, gt=0)
    ransac_px: float = Field(default=3.0, gt=0)
    min_feature_coverage: float = Field(default=0.025, gt=0, lt=1)
    min_overlap: float = Field(default=0.15, gt=0, lt=1)
    min_scale: float = Field(default=0.75, gt=0)
    max_scale: float = Field(default=1.33, gt=0)
    model: Literal["similarity", "rigid"] = "similarity"
    keyframe_cache: int = Field(default=6, ge=2, le=30)
    recovery_failures: int = Field(default=4, ge=1, le=50)
    max_canvas_pixels: int = Field(default=24_000_000, ge=100000, le=100_000_000)
    blur_threshold: float = Field(default=15, ge=0)
    max_roll_deg: float = Field(default=30, gt=0, le=90)
    max_pitch_deg: float = Field(default=30, gt=0, le=90)
    max_telemetry_age_ms: float = Field(default=500, gt=0)
    require_attitude: bool = True
    require_calibration: bool = True
    publish_interval_s: float = Field(default=1.0, ge=0.1)
    preview_max_side: int = Field(default=1600, ge=160, le=4096)

    @model_validator(mode="after")
    def scale_bounds(self):
        if not self.min_scale <= 1 <= self.max_scale:
            raise ValueError("scale bounds must contain 1")
        return self


class UAVConfig(StrictModel):
    uav_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    port: int = Field(ge=1, le=65535)
    camera_id: str = Field(default="camera1", pattern=r"^[A-Za-z0-9_-]{1,32}$")
    calibration_file: str | None = None


class GroundConfig(StrictModel):
    mission_id: str = Field(default="flight01", pattern=r"^[A-Za-z0-9_-]{1,32}$")
    bind_host: str = "0.0.0.0"
    output_dir: str = "SENDER2/runs"
    queue_size: int = Field(default=4, ge=1, le=32)
    max_queue_age_s: float = Field(default=5, gt=0, le=120)
    reassembly_timeout_s: float = Field(default=2, gt=0, le=30)
    allow_simulation: bool = False
    engine: EngineConfig = Field(default_factory=EngineConfig)
    uavs: list[UAVConfig] = Field(default_factory=lambda: [
        UAVConfig(uav_id="UAV1", port=5600), UAVConfig(uav_id="UAV2", port=5610)])

    @model_validator(mode="after")
    def unique_streams(self):
        if not 1 <= len(self.uavs) <= 2:
            raise ValueError("SENDER2 supports one or two independent UAVs")
        if len({u.uav_id for u in self.uavs}) != len(self.uavs):
            raise ValueError("duplicate UAV identity")
        if len({u.port for u in self.uavs}) != len(self.uavs):
            raise ValueError("each UAV needs its own UDP port")
        return self
