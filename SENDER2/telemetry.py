"""Optional MAVLink reader; signed body attitude, explicit vertical datums.

Samples are aligned to companion-host receive time, not asserted to be exposure
time. Hardware trigger/TIMESYNC support needs device-specific validation.
"""
from collections import deque
import logging
import math
import threading
import time
from .models import Attitude, Position, Telemetry

log = logging.getLogger(__name__)


class TelemetryBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.attitudes = deque(maxlen=200)
        self.positions = deque(maxlen=200)
        self.system = None

    def add_attitude(self, timestamp_ns, roll_rad, pitch_rad, yaw_rad):
        attitude = Attitude(roll_deg=math.degrees(roll_rad), pitch_deg=math.degrees(pitch_rad),
                            yaw_deg=math.degrees(yaw_rad))
        with self.lock:
            self.attitudes.append((timestamp_ns, attitude))

    def add_position(self, timestamp_ns, lat_e7, lon_e7, alt_mm, relative_alt_mm, fix_type=None):
        position = Position(latitude_deg=lat_e7/1e7, longitude_deg=lon_e7/1e7,
            altitude_msl_m=alt_mm/1000, altitude_relative_home_m=relative_alt_mm/1000,
            altitude_agl_m=None, fix_type=fix_type)
        with self.lock:
            self.positions.append((timestamp_ns, position))

    def snapshot(self, capture_ns):
        with self.lock:
            attitude = next((s for s in reversed(self.attitudes) if s[0] <= capture_ns), None)
            position = next((s for s in reversed(self.positions) if s[0] <= capture_ns), None)
            return Telemetry(attitude=attitude[1] if attitude else None,
                position=position[1] if position else None,
                attitude_age_ms=(capture_ns-attitude[0])/1e6 if attitude else None,
                position_age_ms=(capture_ns-position[0])/1e6 if position else None,
                source_system=self.system, synchronization="host_receive_time")

    def clear(self):
        with self.lock:
            self.attitudes.clear()
            self.positions.clear()
            self.system = None


class MAVLinkReader(threading.Thread):
    def __init__(self, connection, baud=57600):
        super().__init__(name="sender2-telemetry", daemon=True)
        # Lazy dependency: ground/replay/calibration do not need pymavlink.
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.connection, self.baud = connection, baud
        self.buffer = TelemetryBuffer()
        self.stop_event = threading.Event()
        self.link = None

    def run(self):
        while not self.stop_event.is_set():
            try:
                self.buffer.clear()
                self.link = self.mavutil.mavlink_connection(self.connection, baud=self.baud)
                hb = self.link.wait_heartbeat(timeout=5)
                if hb is None:
                    raise TimeoutError("FC heartbeat timeout")
                system = self.link.target_system
                self.buffer.system = system
                self.link.mav.request_data_stream_send(system, self.link.target_component,
                    self.mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                last_received = time.monotonic()
                fix, fix_time = None, 0
                while not self.stop_event.is_set():
                    msg = self.link.recv_match(type=["ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT"], blocking=True, timeout=0.5)
                    if msg is None:
                        if time.monotonic()-last_received > 5:
                            raise TimeoutError("FC telemetry stale; reconnecting")
                        continue
                    if msg.get_srcSystem() != system:
                        continue
                    last_received = time.monotonic()
                    stamp = time.monotonic_ns()
                    if msg.get_type() == "ATTITUDE":
                        self.buffer.add_attitude(stamp, msg.roll, msg.pitch, msg.yaw)
                    elif msg.get_type() == "GPS_RAW_INT":
                        fix, fix_time = msg.fix_type, last_received
                    elif fix is not None and fix >= 3 and last_received-fix_time < 3:
                        self.buffer.add_position(stamp, msg.lat, msg.lon, msg.alt, msg.relative_alt, fix)
            except Exception:
                log.exception("Telemetry link unavailable; no synthetic metadata will be substituted")
            finally:
                if self.link:
                    self.link.close()
                    self.link = None
                self.buffer.clear()
            self.stop_event.wait(1)

    def stop(self):
        self.stop_event.set()
        self.join(timeout=7)
        if self.is_alive():
            log.error("Telemetry thread did not stop within deadline")
