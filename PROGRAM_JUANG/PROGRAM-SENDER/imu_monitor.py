#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-SENDER | imu_monitor.py
=============================================================================
Dijalankan di: Raspberry Pi 5 (di dalam UAV)

Fungsi STANDALONE:
  - Membaca data IMU dari Flight Controller via MAVLink (serial / UDP)
  - Menampilkan attitude real-time di terminal
  - Mengirim status attitude ke ground via UDP JSON (port TELEM_PORT)
  - Dapat diintegrasikan sebagai library oleh sender.py

Koneksi FC → Raspi:
  - UART GPIO: FC TX → Raspi GPIO15 (RX), FC RX → Raspi GPIO14 (TX)
    Port: /dev/ttyAMA0  Baud: 57600
  - USB FTDI: /dev/ttyUSB0

ArduPilot setup:
  - SERIAL2_PROTOCOL = 2  (MAVLink 2)
  - SERIAL2_BAUD = 57     (57600 baud)

PX4 setup:
  - MAV_1_CONFIG = TELEM2
  - SER_TELEM2_BAUD = 57600

Cara Jalankan (standalone):
  python3 imu_monitor.py
  python3 imu_monitor.py --port /dev/ttyUSB0 --baud 115200
  python3 imu_monitor.py  
=============================================================================
"""

import time
import math
import json
import socket
import argparse
import logging
import sys
import os
import threading

sys.path.insert(0, os.path.dirname(__file__))
import config

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [IMU] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("imu_monitor")

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    log.error("pymavlink tidak tersedia! Install: pip install pymavlink")


# =============================================================================
# Attitude State
# =============================================================================
class AttitudeState:
    """Menyimpan state attitude pesawat terkini."""

    def __init__(self):
        self.roll_deg   = 0.0
        self.pitch_deg  = 0.0
        self.yaw_deg    = 0.0
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate   = 0.0
        self.timestamp  = 0.0
        self.is_stable  = True
        self.armed      = False
        self.mode       = "UNKNOWN"

        # GPS
        self.lat              = 0.0
        self.lon              = 0.0
        self.alt_rel_m        = 0.0
        self.alt_msl_m        = 0.0
        self.fix_type         = 0
        self.satellites_visible = 0
        self.has_gps_fix      = False

        self._lock      = threading.Lock()

    def update(self, roll_deg, pitch_deg, yaw_deg,
               roll_rate=0, pitch_rate=0, yaw_rate=0):
        with self._lock:
            self.roll_deg   = abs(roll_deg)
            self.pitch_deg  = abs(pitch_deg)
            self.yaw_deg    = yaw_deg
            self.roll_rate  = roll_rate
            self.pitch_rate = pitch_rate
            self.yaw_rate   = yaw_rate
            self.timestamp  = time.time()

            # Evaluasi stabilitas
            self.is_stable = (
                self.roll_deg  < config.ROLL_THRESHOLD_DEG and
                self.pitch_deg < config.PITCH_THRESHOLD_DEG
            )

    def update_gps(self, lat, lon, alt_rel_m, alt_msl_m):
        with self._lock:
            self.lat       = lat
            self.lon       = lon
            self.alt_rel_m = alt_rel_m
            self.alt_msl_m = alt_msl_m

    def update_gps_fix(self, fix_type, satellites_visible):
        with self._lock:
            self.fix_type          = fix_type
            self.satellites_visible = satellites_visible
            self.has_gps_fix       = fix_type >= 3

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "timestamp":  self.timestamp,
                "roll_deg":   round(self.roll_deg,  2),
                "pitch_deg":  round(self.pitch_deg, 2),
                "yaw_deg":    round(self.yaw_deg,   2),
                "roll_rate":  round(self.roll_rate,  3),
                "pitch_rate": round(self.pitch_rate, 3),
                "yaw_rate":   round(self.yaw_rate,   3),
                "is_stable":  self.is_stable,
                "armed":      self.armed,
                "mode":       self.mode,
                "thresholds": {
                    "roll":  config.ROLL_THRESHOLD_DEG,
                    "pitch": config.PITCH_THRESHOLD_DEG
                },
                "gps": {
                    "latitude":  round(self.lat, 7),
                    "longitude": round(self.lon, 7),
                    "altitude_rel": round(self.alt_rel_m, 2),
                    "altitude_msl": round(self.alt_msl_m, 2),
                    "fix_type":  self.fix_type,
                    "satellites": self.satellites_visible,
                    "has_fix":   self.has_gps_fix,
                }
            }

    def status_line(self) -> str:
        status = "STABIL  " if self.is_stable else "BERBELOK"
        gps_status = f"GPS={'FIX' if self.has_gps_fix else 'NO-FIX'}({self.satellites_visible})"
        return (
            f"{status} | "
            f"Roll={self.roll_deg:6.2f}° "
            f"Pitch={self.pitch_deg:6.2f}° "
            f"Yaw={self.yaw_deg:7.2f}° | "
            f"Mode={self.mode} Armed={'Y' if self.armed else 'N'} | "
            f"{gps_status} lat={self.lat:.6f} lon={self.lon:.6f} alt={self.alt_rel_m:.1f}m"
        )


# =============================================================================
# MAVLink Reader
# =============================================================================
class MAVLinkReader(threading.Thread):
    """
    Thread yang membaca pesan MAVLink dari FC dan mengupdate AttitudeState.
    Mendukung pesan: ATTITUDE, HEARTBEAT, VFR_HUD
    """

    def __init__(self, connection_str: str, state: AttitudeState):
        super().__init__(daemon=True)
        self.connection_str = connection_str
        self.state          = state
        self._mav           = None
        self.connected      = False
        self.msg_count      = 0

    def run(self):
        if not MAVLINK_AVAILABLE:
            log.error("pymavlink tidak tersedia, IMU Monitor tidak bisa berjalan.")
            return

        log.info(f"Menghubungkan ke FC: {self.connection_str} ...")

        try:
            self._mav = mavutil.mavlink_connection(self.connection_str)
            log.info("Menunggu heartbeat FC ...")
            hb = self._mav.wait_heartbeat(timeout=15)
            if hb is None:
                log.error("Timeout: tidak ada heartbeat dari FC")
                return

            log.info(
                f"FC terhubung! "
                f"System ID={self._mav.target_system} "
                f"Component ID={self._mav.target_component}"
            )
            self.connected = True

        except Exception as e:
            log.error(f"Koneksi FC gagal: {e}")
            return

        # Request stream ATTITUDE & EXTRA1 @ 10 Hz
        self._request_streams()

        # Loop baca pesan
        while True:
            try:
                msg = self._mav.recv_match(
                    type=["ATTITUDE", "HEARTBEAT", "VFR_HUD", "SYS_STATUS"],
                    blocking=True,
                    timeout=2.0
                )
                if msg is None:
                    continue

                msg_type = msg.get_type()
                self.msg_count += 1

                if msg_type == "ATTITUDE":
                    self.state.update(
                        roll_deg   = math.degrees(msg.roll),
                        pitch_deg  = math.degrees(msg.pitch),
                        yaw_deg    = math.degrees(msg.yaw),
                        roll_rate  = math.degrees(msg.rollspeed),
                        pitch_rate = math.degrees(msg.pitchspeed),
                        yaw_rate   = math.degrees(msg.yawspeed)
                    )

                elif msg_type == "HEARTBEAT":
                    # Decode mode
                    try:
                        mode_name = mavutil.mode_string_v10(msg)
                    except Exception:
                        mode_name = str(msg.custom_mode)
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                    with self.state._lock:
                        self.state.mode  = mode_name
                        self.state.armed = armed

            except Exception as e:
                log.warning(f"MAVLink read error: {e}")
                time.sleep(0.5)

    def _request_streams(self):
        """Request data stream dari FC."""
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),  # ATTITUDE @ 10Hz
            (mavutil.mavlink.MAV_DATA_STREAM_ALL,    2),    # ALL @ 2Hz
        ]
        for stream_id, rate in streams:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                stream_id, rate, 1
            )
        log.info("Data stream diminta dari FC.")


# =============================================================================
# Telemetry Sender – kirim status IMU ke ground via UDP
# =============================================================================
class TelemetrySender(threading.Thread):
    """
    Mengirim status attitude ke ground station setiap N detik.
    Ground bisa memonitor kondisi pesawat secara remote.
    """

    def __init__(self, ground_host: str, ground_port: int,
                state: AttitudeState, interval: float = 1.0):
        super().__init__(daemon=True)
        self.ground_host = ground_host
        self.ground_port = ground_port
        self.state       = state
        self.interval    = interval
        self._sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        log.info(f"Telemetry Sender = {self.ground_host}:{self.ground_port} ({self.interval}s interval)")
        while True:
            try:
                data  = json.dumps(self.state.to_dict()).encode("utf-8")
                self._sock.sendto(data, (self.ground_host, self.ground_port))
            except Exception as e:
                log.warning(f"Telem send error: {e}")
            time.sleep(self.interval)


# =============================================================================
# Display Loop
# =============================================================================
def display_loop(state: AttitudeState, interval: float = 0.5):
    """Tampilkan attitude di terminal secara real-time."""
    prev_stable = None
    while True:
        line = state.status_line()

        # Highlight perubahan status stabil/berbelok
        if state.is_stable != prev_stable:
            if state.is_stable:
                log.info(f"{'='*60}")
                log.info("PESAWAT STABIL – Streaming AKTIF")
                log.info(f"{'='*60}")
            else:
                log.warning(f"{'='*60}")
                log.warning("PESAWAT BERBELOK – Streaming DIHENTIKAN")
                log.warning(f"  Roll={state.roll_deg:.1f}° (max {config.ROLL_THRESHOLD_DEG}°) | "
                            f"Pitch={state.pitch_deg:.1f}° (max {config.PITCH_THRESHOLD_DEG}°)")
                log.warning(f"{'='*60}")
            prev_stable = state.is_stable

        print(f"\r[IMU] {line}", end="", flush=True)
        time.sleep(interval)


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="IMU Monitor – baca attitude FC via MAVLink"
    )
    parser.add_argument("--port",        type=str,  default=config.FC_SERIAL_PORT,
                        help="Serial port atau UDP string (e.g. /dev/ttyAMA0 atau udp:14550)")
    parser.add_argument("--baud",        type=int,  default=config.FC_BAUD_RATE,
                        help="Baud rate (untuk koneksi serial)")
    parser.add_argument("--ground-host", type=str,  default=config.GROUND_IP,
                        help="IP ground station untuk kirim telemetry")
    parser.add_argument("--telem-port",  type=int,  default=config.TELEM_PORT,
                        help="UDP port telemetry ke ground")
    parser.add_argument("--no-send",     action="store_true",
                        help="Jangan kirim telemetry, hanya display lokal")
    parser.add_argument("--roll-thresh",  type=float, default=config.ROLL_THRESHOLD_DEG)
    parser.add_argument("--pitch-thresh", type=float, default=config.PITCH_THRESHOLD_DEG)
    return parser.parse_args()


def main():
    args = parse_args()

    # Update threshold dari argumen
    config.ROLL_THRESHOLD_DEG  = args.roll_thresh
    config.PITCH_THRESHOLD_DEG = args.pitch_thresh

    log.info("=" * 60)
    log.info("PROGRAM-SENDER IMU Monitor v1.0")
    log.info(f"  FC Port        : {args.port} @ {args.baud} baud")
    log.info(f"  Roll threshold : ±{args.roll_thresh}°")
    log.info(f"  Pitch threshold: ±{args.pitch_thresh}°")
    log.info(f"  Telemetry      : {'Disabled (--no-send)' if args.no_send else f'{args.ground_host}:{args.telem_port}'}")
    log.info("=" * 60)

    if not MAVLINK_AVAILABLE:
        log.error("pymavlink tidak terinstall. Jalankan: pip install pymavlink")
        log.error("Untuk testing tanpa FC, gunakan sender.py --no-fc")
        return

    # Buat connection string
    if "udp" in args.port.lower() or "tcp" in args.port.lower():
        conn_str = args.port
    else:
        conn_str = f"{args.port},{args.baud}"

    state  = AttitudeState()
    reader = MAVLinkReader(conn_str, state)
    reader.start()

    # Tunggu koneksi
    for _ in range(15):
        if reader.connected:
            break
        time.sleep(1)

    if not reader.connected:
        log.error("FC tidak merespons. Periksa kabel dan setting FC.")
        log.info("Tips:")
        log.info("  - ArduPilot: set SERIAL2_PROTOCOL=2, SERIAL2_BAUD=57")
        log.info("  - PX4: pastikan MAVLink terhubung ke port yang benar")
        log.info("  - Cek: ls /dev/ttyAMA* /dev/ttyUSB*")
        return

    # Sender telemetry ke ground
    if not args.no_send:
        telem = TelemetrySender(args.ground_host, args.telem_port, state)
        telem.start()

    log.info("Memulai monitoring. Tekan Ctrl+C untuk berhenti.")

    try:
        display_loop(state, interval=0.5)
    except KeyboardInterrupt:
        print()
        log.info(f"Selesai. Total pesan MAVLink: {reader.msg_count}")


if __name__ == "__main__":
    main()