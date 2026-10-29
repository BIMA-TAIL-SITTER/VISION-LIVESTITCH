import json
import math
import numpy as np
import pytest
from pydantic import ValidationError
from SENDER2.models import GroundConfig, UAVConfig
from SENDER2.sender import FrameSender
from SENDER2.telemetry import TelemetryBuffer


def test_signed_telemetry_units_datums_and_age():
    buffer = TelemetryBuffer()
    buffer.add_attitude(1_000_000_000,math.radians(-20),math.radians(-5),math.radians(90))
    buffer.add_position(1_000_000_000,-71234567,1121234567,123000,80000,3)
    t = buffer.snapshot(1_100_000_000)
    assert t.attitude.roll_deg==pytest.approx(-20)
    assert t.attitude.pitch_deg==pytest.approx(-5)
    assert t.attitude.yaw_deg==pytest.approx(90)
    assert t.attitude_age_ms==100
    assert t.position.altitude_msl_m==123 and t.position.altitude_relative_home_m==80
    assert t.position.altitude_agl_m is None
    assert buffer.snapshot(1).attitude is None
    buffer.clear()
    assert buffer.snapshot(2_000_000_000).attitude is None


def test_archive_precedes_network_and_failure_preserves_original(workdir):
    original = b"original file bytes unchanged"
    def fail(packet):
        assert list(workdir.rglob("*.jpg"))
        raise OSError("simulated radio failure")
    sender = FrameSender("localhost",5600,"flight01","UAV1","camera1",workdir,
                         simulation=True,send_datagram=fail)
    with pytest.raises(OSError):
        sender.send(np.zeros((240,320,3),np.uint8),original_bytes=original,extension=".jpg")
    assert next(workdir.rglob("*.jpg")).read_bytes()==original
    record = json.loads(next(workdir.rglob("000000000000.json")).read_text())
    assert record["frame"]["telemetry"]["position"] is None
    assert record["frame"]["telemetry"]["attitude"] is None
    assert sender.frame_id==1


@pytest.mark.parametrize("name",["../escape","a/b","C:\\escape", "a"*33])
def test_unsafe_identifiers_rejected(meta_factory,name):
    with pytest.raises(ValidationError):
        meta_factory(mission_id=name)


def test_nonfinite_telemetry_and_duplicate_ports_rejected(meta_factory):
    with pytest.raises(ValidationError):
        meta_factory(telemetry={"position":{"latitude_deg":float("nan"),"longitude_deg":0}})
    with pytest.raises(ValidationError):
        GroundConfig(uavs=[UAVConfig(uav_id="UAV1",port=5600),UAVConfig(uav_id="UAV2",port=5600)])


def test_camera_archive_is_lossless_and_frame_ids_do_not_repeat(workdir):
    import cv2
    packets = []
    image = np.random.default_rng(12).integers(0,256,(240,320,3),np.uint8)
    sender = FrameSender("localhost",5600,"flight01","UAV1","camera1",workdir,
                         send_datagram=packets.append,max_mbps=1000)
    first,_ = sender.send(image)
    second,_ = sender.send(image)
    assert (first.frame_id,second.frame_id)==(0,1)
    stored = cv2.imread(str(next(workdir.rglob("000000000000.png"))))
    assert np.array_equal(stored,image)
    assert first.capture_time_source=="host_read_completed" and first.capture_uncertainty_ms is None
