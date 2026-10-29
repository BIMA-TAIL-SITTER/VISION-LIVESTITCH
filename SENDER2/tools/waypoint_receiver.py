"""Standalone ground UDP receiver with durable deduplication and ACK; no flight control."""
import argparse
import json
import logging
from pathlib import Path
import socket
import sqlite3
import threading
from typing import Literal
from uuid import UUID
from pydantic import Field, model_validator
from SENDER2.models import StrictModel
from SENDER2.waypoints import Fix, Waypoint


class Trigger(StrictModel):
    schema_version: Literal[1]
    type: Literal["waypoint_trigger"]
    event_id: UUID
    mission_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    uav_id: Literal["UAV1"]
    waypoint: Waypoint
    distance_m: float = Field(ge=0, le=20)
    radius_m: Literal[20]
    triggered_at_unix_s: float = Field(gt=0)
    position: Fix
    simulation: bool

    @model_validator(mode="after")
    def mode_consistent(self):
        if self.simulation != self.position.simulation:
            raise ValueError("inconsistent simulation flag")
        return self


class Receiver:
    def __init__(self, host, port, database, allow_simulation=False, drop_first_ack=False):
        self.allow_simulation = allow_simulation
        self.drop_first_ack = drop_first_ack
        self.dropped_ack = False
        self.datagrams = 0
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        Path(database).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(database, check_same_thread=False)
        self.db.execute("CREATE TABLE IF NOT EXISTS received (id TEXT PRIMARY KEY, payload TEXT NOT NULL)")
        self.db.commit()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.2)
        self.address = self.sock.getsockname()

    def run(self):
        while not self.stop_event.is_set():
            try:
                data, address = self.sock.recvfrom(1201)
            except socket.timeout:
                continue
            except OSError:
                if self.stop_event.is_set():
                    break
                raise
            try:
                self.datagrams += 1
                if len(data) > 1200:
                    raise ValueError("oversized waypoint datagram")
                event = Trigger.model_validate_json(data)
                if event.simulation and not self.allow_simulation:
                    raise ValueError("simulation rejected; enable only for testing")
                event_id, payload = str(event.event_id), event.model_dump_json()
                with self.lock, self.db:
                    old = self.db.execute("SELECT payload FROM received WHERE id=?", (event_id,)).fetchone()
                    if old and json.loads(old[0]) != json.loads(payload):
                        raise ValueError("conflicting payload for existing event_id")
                    self.db.execute("INSERT OR IGNORE INTO received VALUES (?,?)", (event_id, payload))
                # ACK only after durable commit. Retransmission receives the same ACK.
                if self.drop_first_ack and not self.dropped_ack:
                    self.dropped_ack = True
                    continue
                ack = {"type":"waypoint_ack", "event_id":event_id, "accepted":True}
                self.sock.sendto(json.dumps(ack).encode(), address)
                logging.info("%s %s WP=%s lat=%s lon=%s duplicate=%s", event.mission_id, event.uav_id,
                    event.waypoint.waypoint_id, event.waypoint.latitude_deg, event.waypoint.longitude_deg, old is not None)
            except (ValueError, UnicodeError):
                logging.exception("Invalid waypoint packet")

    def records(self):
        with self.lock:
            return [json.loads(r[0]) for r in self.db.execute("SELECT payload FROM received ORDER BY rowid")]

    def close(self):
        self.sock.close()
        self.db.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5700)
    parser.add_argument("--database", default="SENDER2/runs/waypoints/ground.sqlite3")
    parser.add_argument("--allow-simulation", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    receiver = Receiver(args.host, args.port, args.database, args.allow_simulation)
    print(f"Ground waypoint UDP listening on {receiver.address}", flush=True)
    try:
        receiver.run()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop_event.set()
        receiver.close()


if __name__ == "__main__":
    main()
