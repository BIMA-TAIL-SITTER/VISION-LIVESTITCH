import math
import pytest
from pydantic import ValidationError
from SENDER2.waypoints import Mission, Waypoint, Fix, WaypointMonitor, distance_m


def mission():
    return Mission(mission_id="test_wp", ground_host="127.0.0.1", ground_port=5700,
        waypoints=[Waypoint(waypoint_id=f"WP{i+1}", latitude_deg=0, longitude_deg=i*0.001) for i in range(5)])


def fix(metres, stamp=100):
    return Fix(latitude_deg=math.degrees(metres/6371008.8), longitude_deg=0,
               position_timestamp_unix_s=stamp, fix_type=3)


def test_horizontal_distance_and_dateline():
    assert distance_m(0,0,0,0) == 0
    assert distance_m(0,179.999,0,-179.999) == pytest.approx(222.39, abs=0.01)


def test_radius_boundary_and_only_once_after_reentry(workdir):
    monitor = WaypointMonitor(mission(), workdir/"state.db")
    try:
        assert not monitor.observe(fix(20.1), now=100)["triggered"]
        assert len(monitor.observe(fix(19.9,101), now=101)["triggered"]) == 1
        assert not monitor.observe(fix(50,102), now=102)["triggered"]
        assert not monitor.observe(fix(0,103), now=103)["triggered"]
    finally:
        monitor.close()


@pytest.mark.parametrize("stamp,now", [(90,100),(101,100)])
def test_stale_and_future_positions_do_not_trigger(workdir, stamp, now):
    monitor = WaypointMonitor(mission(), workdir/"state.db")
    try:
        with pytest.raises(ValueError):
            monitor.observe(fix(0,stamp), now=now)
        assert all(w["state"]=="waiting" for w in monitor.status()["waypoints"])
    finally:
        monitor.close()


def test_pending_survives_restart_and_requires_correct_ack(workdir):
    path = workdir/"state.db"
    monitor = WaypointMonitor(mission(), path)
    event = monitor.observe(fix(0), now=100)["triggered"][0]
    monitor.dispatch_once(lambda p:{"type":"waypoint_ack","accepted":True,"event_id":"wrong"}, now=100)
    assert monitor.status()["waypoints"][0]["state"] == "pending"
    monitor.close()
    monitor = WaypointMonitor(mission(), path)
    try:
        assert not monitor.observe(fix(0,101), now=101)["triggered"]
        ids = []
        def ack(payload):
            ids.append(payload["event_id"])
            return {"type":"waypoint_ack", "accepted":True, "event_id":payload["event_id"]}
        monitor.dispatch_once(ack, now=200)
        monitor.dispatch_once(ack, now=300)
        assert ids == [event["event_id"]]
        assert monitor.status()["waypoints"][0]["state"] == "delivered"
    finally:
        monitor.close()


def test_changed_mission_cannot_reuse_trigger_state(workdir):
    path = workdir/"state.db"
    monitor = WaypointMonitor(mission(), path)
    monitor.close()
    changed = mission()
    changed.waypoints[0].latitude_deg = 1
    with pytest.raises(ValueError, match="mission changed"):
        WaypointMonitor(changed, path)


def test_exactly_five_valid_distinct_points_and_one_fixedwing():
    data = mission().model_dump()
    with pytest.raises(ValidationError):
        Mission(**{**data,"waypoints":data["waypoints"][:4]})
    with pytest.raises(ValidationError):
        Mission(**{**data,"uav_id":"COPTER1"})
    with pytest.raises(ValidationError):
        Waypoint(waypoint_id="WP1", latitude_deg=float("nan"),longitude_deg=0)
    with pytest.raises(ValidationError):
        Fix(latitude_deg=0,longitude_deg=0,position_timestamp_unix_s=100,fix_type=2)


def test_out_of_order_and_simulation_rejected(workdir):
    monitor = WaypointMonitor(mission(), workdir/"state.db")
    try:
        monitor.observe(fix(50), now=100)
        with pytest.raises(ValueError, match="out-of-order"):
            monitor.observe(fix(0), now=100)
        simulated = fix(0,101).model_copy(update={"simulation":True})
        with pytest.raises(ValueError, match="simulation"):
            monitor.observe(simulated, now=101)
    finally:
        monitor.close()
