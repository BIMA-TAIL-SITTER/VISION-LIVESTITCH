"""One fixed-wing approaches five points; real localhost UDP, including one lost ACK."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import uuid
from SENDER2.waypoints import Mission, Waypoint, Fix, WaypointMonitor
from SENDER2.storage import write_json
from SENDER2.tools.waypoint_receiver import Receiver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="SENDER2/runs/waypoint_simulation")
    args = parser.parse_args()
    root = Path(args.output) / uuid.uuid4().hex[:12]
    root.mkdir(parents=True)
    receiver = Receiver("127.0.0.1", 0, root/"ground.sqlite3", allow_simulation=True, drop_first_ack=True)
    thread = threading.Thread(target=receiver.run)
    thread.start()
    mission = Mission(mission_id="demo_"+root.name, ground_host="127.0.0.1", ground_port=receiver.address[1], simulation=True,
        waypoints=[Waypoint(waypoint_id=f"WP{i+1}", latitude_deg=-7.77+i*0.001, longitude_deg=110.37) for i in range(5)])
    write_json(root/"mission.json", mission.model_dump(mode="json"))
    monitor = WaypointMonitor(mission, root/"onboard.sqlite3")
    now = time.time()
    observations = []
    try:
        for wp in mission.waypoints:
            for metres in [35, 19, 10, 35]:
                now += 0.1
                fix = Fix(latitude_deg=wp.latitude_deg-math.degrees(metres/6371008.8), longitude_deg=wp.longitude_deg,
                          position_timestamp_unix_s=now, fix_type=3, simulation=True)
                result = monitor.observe(fix, now=now)
                observations.append({"target":wp.waypoint_id, "nominal_distance_m":metres, **result})
                print(f"{wp.waypoint_id} distance={metres}m triggers={len(result['triggered'])}")
            monitor.dispatch_once(now=now)
        # First ground ACK deliberately lost: repeat the same event, not a new request.
        monitor.dispatch_once(now=now+60)
        state = monitor.status()
        records = receiver.records()
        report = {"mode":"synthetic single-UAV positions; real UDP localhost", "observations":observations,
                  "status":state, "ground_records":records, "datagrams":receiver.datagrams,
                  "dropped_ack":receiver.dropped_ack}
        write_json(root/"report.json", report)
        assert len(records) == 5
        assert all(w["state"] == "delivered" for w in state["waypoints"])
        assert sum(len(o["triggered"]) for o in observations) == 5
        print(json.dumps({"report":str((root/"report.json").resolve()), "unique_waypoints":len(records),
                          "all_delivered":True, "datagrams":receiver.datagrams}, indent=2))
    finally:
        monitor.close()
        receiver.stop_event.set()
        thread.join(timeout=3)
        receiver.close()


if __name__ == "__main__":
    main()
