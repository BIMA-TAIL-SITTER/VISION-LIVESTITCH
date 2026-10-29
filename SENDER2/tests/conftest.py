from pathlib import Path
import time
import uuid
import cv2
import numpy as np
import pytest
from SENDER2.models import FrameMeta, EngineConfig


@pytest.fixture
def workdir():
    # Workspace-local persistent test artifacts avoid platform temp cleanup issues.
    path = Path(__file__).resolve().parents[1]/"test_artifacts"/uuid.uuid4().hex
    path.mkdir(parents=True)
    return path


@pytest.fixture
def meta_factory():
    epoch = uuid.uuid4()
    started = time.time_ns()
    def make(**changes):
        values = dict(mission_id="test01",uav_id="UAV1",stream_epoch=epoch,
            stream_started_unix_ns=started,frame_id=0,capture_timestamp_unix_ns=started,
            capture_timestamp_monotonic_ns=time.monotonic_ns(),capture_time_source="replay_read_completed",
            camera_id="camera1",capture_width=320,capture_height=240,preview_width=320,
            preview_height=240,simulation=True)
        values.update(changes)
        return FrameMeta(**values)
    return make


@pytest.fixture
def engine_config():
    return EngineConfig(working_width=320,require_attitude=False,require_calibration=False,
        min_inliers=12,sift_features=1000,blur_threshold=0,max_canvas_pixels=500000,
        publish_interval_s=0.1)


@pytest.fixture
def textured():
    rng = np.random.default_rng(27)
    image = rng.integers(0,256,(240,600,3),np.uint8)
    # Add larger stable geometric features as well as texture.
    for i in range(30):
        x,y = int(rng.integers(10,580)),int(rng.integers(10,220))
        cv2.circle(image,(x,y),int(rng.integers(3,12)),(255,255,255),2)
    return image
