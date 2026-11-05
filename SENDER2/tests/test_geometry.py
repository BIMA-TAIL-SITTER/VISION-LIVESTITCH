import uuid
import cv2
import numpy as np
import pytest
from SENDER2.geometry import Calibration, Preprocessor, estimate_registration
from SENDER2.canvas import MosaicCanvas, CanvasLimit
from SENDER2.models import Attitude, Telemetry
from SENDER2.stitcher import LiveEngine


def test_similarity_recovers_known_rotation_scale_translation(engine_config):
    points = np.random.default_rng(42).uniform([30,30],[250,180],(100,2)).astype(np.float32)
    M = cv2.getRotationMatrix2D((160,120),8,1.08)
    M[:,2] += [8,-4]
    expected = np.vstack([M,[0,0,1]])
    dst = cv2.perspectiveTransform(points[:,None],expected)[:,0]
    dst[-20:] = np.random.default_rng(3).uniform(0,200,(20,2))
    result = estimate_registration(points,dst,(240,320),(240,320),engine_config)
    assert result is not None and result.inliers >= 80
    assert np.max(np.abs(result.H-expected)) < 0.01
    assert result.median_px < 0.01


def test_rigid_refits_translation_and_reports_actual_residual(engine_config):
    engine_config.model = "rigid"
    points = np.random.default_rng(1).uniform([20,20],[290,210],(80,2)).astype(np.float32)
    M = cv2.getRotationMatrix2D((160,120),10,1)
    M[:,2] += [12,-3]
    H = np.vstack([M,[0,0,1]])
    dst = cv2.perspectiveTransform(points[:,None],H)[:,0]
    result = estimate_registration(points,dst,(240,320),(240,320),engine_config)
    assert result is not None and result.median_px < 0.01
    assert np.allclose(result.H,H,atol=0.001)
    # Real scale variation must be rejected by rigid, not reported as 80 false inliers.
    assert estimate_registration(points,points*1.1+[10,0],(240,320),(240,320),engine_config) is None


def test_concentrated_matches_rejected(engine_config):
    src = np.random.default_rng(1).uniform(100,105,(40,2)).astype(np.float32)
    assert estimate_registration(src,src+[3,2],(240,320),(240,320),engine_config) is None


def test_black_pixels_have_geometric_coverage_and_budget_is_transactional():
    c = MosaicCanvas(10000)
    image = np.zeros((40,40,3),np.uint8)
    mask = np.full((40,40),255,np.uint8)
    c.add(image,mask,np.eye(3))
    c.add(image,mask,np.array([[1,0,40],[0,1,0],[0,0,1]],float))
    assert (c.weight>0).sum() == 3200
    assert c.render().max() == 0
    before = c.weight.copy()
    with pytest.raises(CanvasLimit):
        c.add(image,mask,np.array([[1,0,1000],[0,1,0],[0,0,1]],float))
    assert np.array_equal(c.weight,before)


def test_canvas_identity_blend_does_not_corrupt_color():
    c = MosaicCanvas(10000)
    image = np.full((40,40,3),80,np.uint8)
    for _ in range(3):
        c.add(image,np.full((40,40),255,np.uint8),np.eye(3))
    assert np.max(np.abs(c.render().astype(int)-80)) <= 1


def test_calibration_identity_and_resolution_are_validated(meta_factory,engine_config,textured):
    cal = Calibration(calibration_id="cal1",camera_id="camera1",image_width=320,image_height=240,
        camera_matrix=[[300,0,160],[0,300,120],[0,0,1]],distortion=[0,0,0,0,0],rms_error_px=0)
    prep = Preprocessor(engine_config,cal)
    image,mask = prep.prepare(textured[:,:320],meta_factory(calibration_id="cal1"))
    assert image.shape == (240,320,3) and mask.min() == 255
    with pytest.raises(ValueError,match="identity"):
        prep.prepare(textured[:,:320],meta_factory(calibration_id="wrong"))
    with pytest.raises(ValueError,match="resolution"):
        prep.prepare(textured[:,:320],meta_factory(calibration_id="cal1",capture_width=640))


def test_failed_frame_does_not_poison_cache_and_recovery_uses_old_keyframe(meta_factory,engine_config,textured):
    e = LiveEngine(engine_config)
    assert e.process(textured[:,:320],meta_factory())["status"] == "stitched"
    assert e.process(np.zeros((240,320,3),np.uint8),meta_factory(frame_id=1))["status"] == "rejected"
    assert e.last_frame.frame_id == 0 and len(e.cache)==1
    assert e.process(textured[:,30:350],meta_factory(frame_id=2))["status"] == "stitched"
    assert e.last_frame.frame_id==2


def test_registration_failure_retains_state_then_starts_explicit_segment(meta_factory,engine_config,textured):
    engine_config.recovery_failures = 2
    closed = []
    e = LiveEngine(engine_config,on_segment_end=lambda eng,reason:closed.append((eng.segment,reason)))
    e.process(textured[:,:320],meta_factory())
    noise = np.random.default_rng(9).integers(0,256,(240,320,3),np.uint8)
    assert e.process(noise,meta_factory(frame_id=1))["status"] == "registration_failed"
    assert e.last_frame.frame_id==0
    result = e.process(noise,meta_factory(frame_id=2))
    assert result["status"]=="stitched" and result["segment"]==1
    assert closed==[(0,"registration_lost")]
    assert np.array_equal(e.cache[-1].H,np.eye(3))


def test_sender_restart_resets_geometry_separately(meta_factory,engine_config,textured):
    e = LiveEngine(engine_config)
    e.process(textured[:,:320],meta_factory())
    result = e.process(textured[:,30:350],meta_factory(stream_epoch=uuid.uuid4()))
    assert result["segment"]==1 and result["segment_start_reason"]=="sender_restart"
    assert np.array_equal(e.cache[-1].H,np.eye(3))


@pytest.mark.parametrize("roll",[-40,40])
def test_signed_attitude_gate(meta_factory,engine_config,textured,roll):
    t = Telemetry(attitude=Attitude(roll_deg=roll,pitch_deg=-2,yaw_deg=0),attitude_age_ms=10)
    e = LiveEngine(engine_config)
    result = e.process(textured[:,:320],meta_factory(telemetry=t))
    assert result["reason"]=="attitude_exceeds_gate"
    assert not e.cache


def test_stale_and_missing_attitude_are_explicit(meta_factory,engine_config,textured):
    engine_config.require_attitude=True
    e = LiveEngine(engine_config)
    assert e.process(textured[:,:320],meta_factory())["reason"]=="attitude_missing"
    t = Telemetry(attitude=Attitude(roll_deg=0,pitch_deg=0,yaw_deg=0),attitude_age_ms=9999)
    assert e.process(textured[:,:320],meta_factory(telemetry=t))["reason"]=="attitude_stale"
