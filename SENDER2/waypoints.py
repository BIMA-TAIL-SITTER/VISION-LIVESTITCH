"""One fixed-wing, five user waypoints, 20 m horizontal trigger, UDP + ACK.

Runs on the Pi independently of stitching; never commands an autopilot.
"""
import argparse
import hashlib
import json
import logging
import math
from pathlib import Path
import socket
import sqlite3
import threading
import time
from typing import Literal
import uuid
from pydantic import Field, model_validator
from .models import StrictModel
from .storage import write_json

log = logging.getLogger(__name__)


class Waypoint(StrictModel):
    waypoint_id: str = Field(pattern=r"^WP[1-5]$")
    latitude_deg: float = Field(ge=-90, le=90)
    longitude_deg: float = Field(ge=-180, le=180)


class Mission(StrictModel):
    mission_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    uav_id: Literal["UAV1"] = "UAV1"
    ground_host: str = Field(min_length=1, max_length=253)
    ground_port: int = Field(ge=1, le=65535)
    radius_m: Literal[20] = 20
    max_position_age_s: float = Field(default=2, gt=0, le=30)
    simulation: bool = False
    waypoints: list[Waypoint] = Field(min_length=5, max_length=5)

    @model_validator(mode="after")
    def distinct_points(self):
        if {p.waypoint_id for p in self.waypoints} != {f"WP{i}" for i in range(1,6)}:
            raise ValueError("use each ID WP1..WP5 exactly once")
        if len({(p.latitude_deg, p.longitude_deg) for p in self.waypoints}) != 5:
            raise ValueError("waypoint coordinates must be distinct")
        return self


class Fix(StrictModel):
    latitude_deg: float = Field(ge=-90, le=90)
    longitude_deg: float = Field(ge=-180, le=180)
    position_timestamp_unix_s: float = Field(gt=0)
    fix_type: int = Field(ge=3, le=8)
    simulation: bool = False


def distance_m(lat1, lon1, lat2, lon2):
    """Horizontal great-circle distance on mean-radius earth, in metres."""
    a, b = math.radians(lat1), math.radians(lat2)
    h = math.sin((b-a)/2)**2 + math.cos(a)*math.cos(b)*math.sin(math.radians(lon2-lon1)/2)**2
    return 6371008.8 * 2 * math.asin(math.sqrt(min(1, max(0, h))))


class WaypointMonitor:
    def __init__(self, mission, database):
        self.mission = mission
        path = Path(database)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        with self.db:
            self.db.execute("CREATE TABLE IF NOT EXISTS missions (id TEXT PRIMARY KEY, fingerprint TEXT NOT NULL)")
            self.db.execute("CREATE TABLE IF NOT EXISTS fixes (mission TEXT PRIMARY KEY, stamp REAL, payload TEXT)")
            self.db.execute("CREATE TABLE IF NOT EXISTS events (mission TEXT, wp TEXT, id TEXT UNIQUE, payload TEXT, delivered INTEGER DEFAULT 0, attempts INTEGER DEFAULT 0, next_try REAL DEFAULT 0, error TEXT, PRIMARY KEY(mission,wp))")
            content = mission.model_dump(mode="json", exclude={"ground_host", "ground_port"})
            content["waypoints"] = sorted(content["waypoints"], key=lambda p:p["waypoint_id"])
            fingerprint = hashlib.sha256(json.dumps(content, sort_keys=True).encode()).hexdigest()
            row = self.db.execute("SELECT fingerprint FROM missions WHERE id=?", (mission.mission_id,)).fetchone()
            if row is None:
                self.db.execute("INSERT INTO missions VALUES (?,?)", (mission.mission_id, fingerprint))
        if row and row[0] != fingerprint:
            self.db.close()
            raise ValueError("mission changed: use a new mission_id instead of reusing old trigger state")

    def observe(self, fix, now=None):
        now = time.time() if now is None else now
        if fix.simulation != self.mission.simulation:
            raise ValueError("simulation mode differs from mission")
        age = now-fix.position_timestamp_unix_s
        if age < -0.5 or age > self.mission.max_position_age_s:
            raise ValueError("position stale or future-dated")
        triggered, distances = [], {}
        with self.lock, self.db:
            self.db.execute("BEGIN IMMEDIATE")
            old = self.db.execute("SELECT stamp FROM fixes WHERE mission=?", (self.mission.mission_id,)).fetchone()
            if old and fix.position_timestamp_unix_s <= old[0]:
                raise ValueError("duplicate or out-of-order position")
            self.db.execute("INSERT OR REPLACE INTO fixes VALUES (?,?,?)",
                            (self.mission.mission_id, fix.position_timestamp_unix_s, fix.model_dump_json()))
            for wp in self.mission.waypoints:
                distance = distance_m(fix.latitude_deg, fix.longitude_deg, wp.latitude_deg, wp.longitude_deg)
                distances[wp.waypoint_id] = distance
                if distance > self.mission.radius_m:
                    continue
                event_id = str(uuid.uuid5(uuid.NAMESPACE_URL, f"sender2:{self.mission.mission_id}:{self.mission.uav_id}:{wp.waypoint_id}"))
                payload = {"schema_version":1, "type":"waypoint_trigger", "event_id":event_id,
                    "mission_id":self.mission.mission_id, "uav_id":self.mission.uav_id,
                    "waypoint":wp.model_dump(), "distance_m":distance, "radius_m":20,
                    "triggered_at_unix_s":now, "position":fix.model_dump(), "simulation":self.mission.simulation}
                cursor = self.db.execute("INSERT OR IGNORE INTO events (mission,wp,id,payload) VALUES (?,?,?,?)",
                    (self.mission.mission_id, wp.waypoint_id, event_id, json.dumps(payload, allow_nan=False)))
                if cursor.rowcount:
                    triggered.append(payload)
        return {"distances_m":distances, "triggered":triggered}

    def dispatch_once(self, transport=None, now=None):
        now = time.time() if now is None else now
        with self.lock:
            rows = self.db.execute("SELECT * FROM events WHERE mission=? AND delivered=0 AND next_try<=?",
                                   (self.mission.mission_id, now)).fetchall()
        for row in rows:
            error = None
            try:
                ack = (transport or self._udp_send)(json.loads(row["payload"]))
                if ack.get("type") != "waypoint_ack" or ack.get("accepted") is not True or ack.get("event_id") != row["id"]:
                    raise ValueError("ground did not acknowledge this event_id")
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                log.warning("Waypoint pending: %s: %s", row["wp"], error)
            attempts = row["attempts"]+1
            with self.lock, self.db:
                self.db.execute("UPDATE events SET delivered=?,attempts=?,next_try=?,error=? WHERE id=?",
                    (int(error is None), attempts, now + min(30, 2**min(attempts,5)), error, row["id"]))

    def _udp_send(self, payload):
        data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode()
        if len(data) > 1200:
            raise ValueError("waypoint packet exceeds 1200 bytes")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(1)
            sock.connect((self.mission.ground_host, self.mission.ground_port))
            sock.send(data)
            return json.loads(sock.recv(1200))

    def status(self):
        with self.lock:
            events = {r["wp"]:dict(r) for r in self.db.execute("SELECT * FROM events WHERE mission=?", (self.mission.mission_id,))}
            fix = self.db.execute("SELECT payload FROM fixes WHERE mission=?", (self.mission.mission_id,)).fetchone()
        result = []
        for wp in self.mission.waypoints:
            row = events.get(wp.waypoint_id)
            result.append({**wp.model_dump(), "state":"waiting" if not row else "delivered" if row["delivered"] else "pending",
                "attempts":row["attempts"] if row else 0, "last_error":row["error"] if row else None,
                "event":json.loads(row["payload"]) if row else None})
        return {"mission_id":self.mission.mission_id, "uav_id":self.mission.uav_id, "radius_m":20,
                "simulation":self.mission.simulation, "last_position":json.loads(fix[0]) if fix else None, "waypoints":result}

    def close(self):
        self.db.close()


def run_live(args, mission):
    from .telemetry import MAVLinkReader
    monitor = WaypointMonitor(mission, args.database)
    reader = MAVLinkReader(args.fc, args.baud)
    stop = threading.Event()

    def deliver():
        while not stop.is_set():
            try:
                monitor.dispatch_once()
            except Exception:
                log.exception("Dispatch failed; pending events retained")
            stop.wait(0.5)

    sender = threading.Thread(target=deliver, name="waypoint-udp")
    reader.start()
    sender.start()
    last_sample = -1
    last_print = 0
    try:
        while True:
            stamp = time.monotonic_ns()
            snapshot = reader.buffer.snapshot(stamp)
            p = snapshot.position
            if p and snapshot.position_age_ms is not None and snapshot.position_age_ms <= mission.max_position_age_s*1000:
                sample = stamp-round(snapshot.position_age_ms*1e6)
                if sample > last_sample:
                    last_sample = sample
                    fix = Fix(latitude_deg=p.latitude_deg, longitude_deg=p.longitude_deg,
                        position_timestamp_unix_s=time.time()-snapshot.position_age_ms/1000, fix_type=p.fix_type)
                    try:
                        result = monitor.observe(fix)
                        for event in result["triggered"]:
                            log.info("TRIGGER %s distance=%.2f m", event["waypoint"]["waypoint_id"], event["distance_m"])
                        if time.monotonic()-last_print >= 1:
                            log.info("Distances (m): %s", {k:round(v,1) for k,v in result["distances_m"].items()})
                    except ValueError:
                        log.exception("Position rejected")
            if time.monotonic()-last_print >= 1:
                state = monitor.status()
                state["position_available"] = p is not None and snapshot.position_age_ms is not None and snapshot.position_age_ms <= mission.max_position_age_s*1000
                write_json(args.status, state)
                last_print = time.monotonic()
            time.sleep(0.1)
    except KeyboardInterrupt:
        log.info("Stopping; pending waypoint events remain in SQLite")
    finally:
        stop.set()
        reader.stop()
        sender.join()
        write_json(args.status, monitor.status())
        monitor.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    configure = commands.add_parser("configure", help="Enter five latitude/longitude pairs")
    configure.add_argument("--output", default="SENDER2/waypoints.config.json")
    configure.add_argument("--mission", required=True)
    configure.add_argument("--ground-host", required=True)
    configure.add_argument("--ground-port", type=int, default=5700)
    run = commands.add_parser("run", help="Pi: read FC position, trigger and send UDP")
    run.add_argument("--config", default="SENDER2/waypoints.config.json")
    run.add_argument("--database", default="SENDER2/runs/waypoints/state.sqlite3")
    run.add_argument("--status", default="SENDER2/runs/waypoints/status.json")
    run.add_argument("--fc", required=True)
    run.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if args.command == "configure":
        output = Path(args.output)
        if output.exists():
            parser.error("output exists; choose a new --output to preserve the mission")
        points = []
        for i in range(1,6):
            while True:
                try:
                    lat, lon = map(float, input(f"WP{i} latitude longitude (example: -7.77 110.37): ").replace(",", " ").split())
                    wp = Waypoint(waypoint_id=f"WP{i}", latitude_deg=lat, longitude_deg=lon)
                    if any((p.latitude_deg,p.longitude_deg)==(lat,lon) for p in points):
                        raise ValueError("duplicate coordinate")
                    points.append(wp)
                    break
                except ValueError as exc:
                    print(f"Invalid input: {exc}")
        mission = Mission(mission_id=args.mission, ground_host=args.ground_host, ground_port=args.ground_port, waypoints=points)
        write_json(output, mission.model_dump(mode="json"))
        print(f"Saved {output.resolve()}")
    else:
        mission = Mission.model_validate_json(Path(args.config).read_text(encoding="utf-8"))
        if mission.simulation:
            parser.error("live run requires simulation=false")
        run_live(args, mission)


if __name__ == "__main__":
    main()
