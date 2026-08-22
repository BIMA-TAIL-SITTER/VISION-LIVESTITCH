#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-NOVA | photo_capture.py
=============================================================================
Dijalankan di: Raspberry Pi 5 (UAV)

Fungsi:
  Mengkonfigurasi DJI Osmo Action 5 Pro sebagai kamera FOTO (bukan video):
  - Menangkap 1 frame per detik (1 fps) dalam mode still/photo
  - Menginjeksi EXIF GPS dari Flight Controller via MAVLink ke setiap gambar
  - Menyimpan lokal dengan nama terstruktur
  - Mengirim frame JPEG + metadata ke ground via UDP

Perbedaan dengan sender.py:
  sender.py   → stream video berkelanjutan (banyak frame)
  photo_capture.py → 1 foto/detik, EXIF GPS tertanam, lebih presisi

Cara Jalankan:
  python3 photo_capture.py
  python3 photo_capture.py --no-fc --interval 1.0
  python3 photo_capture.py --host 100.76.49.111 --interval 2.0

=============================================================================
"""

import os
import sys
import time
import math
import json
import socket
import struct
import argparse
import logging
import threading
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
import piexif

sys.path.insert(0, os.path.dirname(__file__))
import config

# MAVLink opsional
try:
    from pymavlink import mavutil
    MAVLINK_OK = True
except ImportError:
    MAVLINK_OK = False

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [PHOTO] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("photo_capture")


# ─────────────────────────────────────────────────────────────────────────────
# GPS State – diisi oleh thread MAVLink
# ─────────────────────────────────────────────────────────────────────────────
class GPSState:
    """State GPS + attitude terkini dari FC."""
    def __init__(self):
        self._lock     = threading.Lock()
        self.lat       = 0.0
        self.lon       = 0.0
        self.alt_rel   = 0.0   # altitude MSL (meter)
        self.alt_abs   = 0.0   # altitude relative (AGL)
        self.roll_deg  = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0
        self.fix_type  = 0     # 0=no fix, 2=2D, 3=3D
        self.hdop      = 99.9
        self.sats      = 0
        self.is_stable = True
        self.valid     = False  # True jika GPS fix tersedia
        self.timestamp = 0.0

    def update_gps(self, lat, lon, alt_rel, alt_abs, fix_type, hdop, sats):
        with self._lock:
            self.lat      = lat
            self.lon      = lon
            self.alt_rel  = alt_rel
            self.alt_abs  = alt_abs
            self.fix_type = fix_type
            self.hdop     = hdop
            self.sats     = sats
            self.valid    = fix_type >= 3
            self.timestamp = time.time()

    def update_attitude(self, roll, pitch, yaw):
        with self._lock:
            self.roll_deg  = abs(math.degrees(roll))
            self.pitch_deg = abs(math.degrees(pitch))
            self.yaw_deg   = math.degrees(yaw)
            self.is_stable = (
                self.roll_deg  < config.ROLL_THRESHOLD_DEG and
                self.pitch_deg < config.PITCH_THRESHOLD_DEG
            )

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "lat":       self.lat,
                "lon":       self.lon,
                "alt_rel":   self.alt_rel,
                "alt_abs":   self.alt_abs,
                "roll_deg":  self.roll_deg,
                "pitch_deg": self.pitch_deg,
                "yaw_deg":   self.yaw_deg,
                "fix_type":  self.fix_type,
                "hdop":      self.hdop,
                "sats":      self.sats,
                "is_stable": self.is_stable,
                "valid":     self.valid,
                "timestamp": self.timestamp,
            }


# ─────────────────────────────────────────────────────────────────────────────
# MAVLink Thread
# ─────────────────────────────────────────────────────────────────────────────
class MAVLinkThread(threading.Thread):
    """
    Membaca GPS_RAW_INT, GLOBAL_POSITION_INT, dan ATTITUDE dari FC.
    Update GPSState secara real-time.
    """

    def __init__(self, conn_str: str, gps_state: GPSState):
        super().__init__(daemon=True)
        self.conn_str  = conn_str
        self.gps_state = gps_state
        self.connected = False
        self._mav      = None

    def run(self):
        if not MAVLINK_OK:
            log.warning("pymavlink tidak tersedia – GPS EXIF tidak akan tertanam.")
            return

        log.info(f"MAVLink: menghubungkan ke {self.conn_str} ...")
        try:
            self._mav = mavutil.mavlink_connection(self.conn_str)
            self._mav.wait_heartbeat(timeout=15)
            log.info(
                f"✓ FC terhubung! Sys={self._mav.target_system} "
                f"Comp={self._mav.target_component}"
            )
            self.connected = True
        except Exception as e:
            log.error(f"MAVLink koneksi gagal: {e}")
            return

        # Request streams
        self._request_streams()

        TYPES = ["GPS_RAW_INT", "GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD"]
        while True:
            try:
                msg = self._mav.recv_match(type=TYPES, blocking=True, timeout=2.0)
                if msg is None:
                    continue
                t = msg.get_type()

                if t == "GLOBAL_POSITION_INT":
                    # Lat/lon dalam 1e7 derajat
                    lat     = msg.lat / 1e7
                    lon     = msg.lon / 1e7
                    alt_msl = msg.alt / 1000.0      # mm → m
                    alt_rel = msg.relative_alt / 1000.0
                    self.gps_state.update_gps(
                        lat, lon, alt_rel, alt_msl,
                        fix_type=3, hdop=1.0, sats=12  # Dari GLOBAL_POSITION asumsi fix
                    )

                elif t == "GPS_RAW_INT":
                    # Update sats, hdop, fix_type saja (lat/lon dari GLOBAL_POSITION lebih akurat)
                    with self.gps_state._lock:
                        self.gps_state.fix_type = msg.fix_type
                        self.gps_state.hdop     = msg.eph / 100.0
                        self.gps_state.sats     = msg.satellites_visible
                        self.gps_state.valid    = msg.fix_type >= 3

                elif t == "ATTITUDE":
                    self.gps_state.update_attitude(msg.roll, msg.pitch, msg.yaw)

            except Exception as e:
                log.warning(f"MAVLink read error: {e}")
                time.sleep(0.5)

    def _request_streams(self):
        for stream_id, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION,  5),   # GPS @ 5Hz
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,   10),   # ATTITUDE @ 10Hz
        ]:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                stream_id, rate, 1
            )


# ─────────────────────────────────────────────────────────────────────────────
# EXIF Injector
# ─────────────────────────────────────────────────────────────────────────────
class EXIFInjector:
    """
    Menyuntikkan data GPS dari FC ke dalam JPEG menggunakan piexif.
    Standard EXIF GPS yang kompatibel dengan hampir semua software pemetaan
    (Agisoft Metashape, OpenDroneMap, QGIS).
    """

    @staticmethod
    def _to_rational(val: float) -> tuple:
        """Konversi float ke tuple rational (numerator, denominator) untuk EXIF."""
        # Gunakan presisi 7 desimal (≈ 1cm di ekuator)
        num = int(abs(val) * 10_000_000)
        den = 10_000_000
        return (num, den)

    @staticmethod
    def _dd_to_dms_rational(dd: float):
        """
        Konversi Decimal Degrees ke DMS (Degrees, Minutes, Seconds) rational.
        Return: ((deg_num, deg_den), (min_num, min_den), (sec_num, sec_den))
        """
        d = int(abs(dd))
        m_float = (abs(dd) - d) * 60
        m = int(m_float)
        s = (m_float - m) * 60

        return (
            (d, 1),
            (m, 1),
            (int(s * 10000), 10000)
        )

    def inject(self, jpeg_bytes: bytes, gps: dict) -> bytes:
        """
        Inject GPS EXIF ke JPEG bytes. Return bytes baru dengan EXIF.

        GPS dict harus punya: lat, lon, alt_rel, alt_abs, yaw_deg
        """
        lat = gps.get("lat", 0.0)
        lon = gps.get("lon", 0.0)
        alt = gps.get("alt_abs", 0.0)  # Gunakan altitude absolut (MSL)
        alt_rel = gps.get("alt_rel", 0.0)

        lat_ref = "N" if lat >= 0 else "S"
        lon_ref = "E" if lon >= 0 else "W"

        lat_dms = self._dd_to_dms_rational(lat)
        lon_dms = self._dd_to_dms_rational(lon)

        # Timestamp UTC
        now = datetime.utcnow()
        date_str = now.strftime("%Y:%m:%d")
        time_str = now.strftime("%H:%M:%S")
        gps_time = (
            (now.hour, 1),
            (now.minute, 1),
            (now.second, 1)
        )

        gps_ifd = {
            piexif.GPSIFD.GPSVersionID:         (2, 3, 0, 0),
            piexif.GPSIFD.GPSLatitudeRef:        lat_ref.encode(),
            piexif.GPSIFD.GPSLatitude:           lat_dms,
            piexif.GPSIFD.GPSLongitudeRef:       lon_ref.encode(),
            piexif.GPSIFD.GPSLongitude:          lon_dms,
            piexif.GPSIFD.GPSAltitudeRef:        0,              # 0=above sea level
            piexif.GPSIFD.GPSAltitude:           self._to_rational(max(0, alt)),
            piexif.GPSIFD.GPSTimeStamp:          gps_time,
            piexif.GPSIFD.GPSDateStamp:          now.strftime("%Y:%m:%d").encode(),
            piexif.GPSIFD.GPSImgDirectionRef:    b"T",           # True North
            piexif.GPSIFD.GPSImgDirection:       self._to_rational(gps.get("yaw_deg", 0.0) % 360),
            piexif.GPSIFD.GPSMeasureMode:        b"3",           # 3D
            piexif.GPSIFD.GPSDOP:                self._to_rational(gps.get("hdop", 1.0)),
            piexif.GPSIFD.GPSSatellites:         str(gps.get("sats", 0)).encode(),
        }

        zeroth_ifd = {
            piexif.ImageIFD.Make:     b"DJI",
            piexif.ImageIFD.Model:    b"Osmo Action 5 Pro",
            piexif.ImageIFD.Software: b"NOVA-v1.0",
            piexif.ImageIFD.DateTime: f"{date_str} {time_str}".encode(),
        }

        exif_ifd = {
            piexif.ExifIFD.DateTimeOriginal:  f"{date_str} {time_str}".encode(),
            piexif.ExifIFD.DateTimeDigitized: f"{date_str} {time_str}".encode(),
            # UserComment: simpan data ekstra sebagai JSON
            piexif.ExifIFD.UserComment: (
                "UNICODE\x00" +
                json.dumps({
                    "alt_agl": round(alt_rel, 2),
                    "roll":    round(gps.get("roll_deg", 0), 2),
                    "pitch":   round(gps.get("pitch_deg", 0), 2),
                    "yaw":     round(gps.get("yaw_deg", 0), 2),
                    "sats":    gps.get("sats", 0),
                    "hdop":    round(gps.get("hdop", 99), 2),
                })
            ).encode("utf-8"),
        }

        exif_dict = {
            "0th":  zeroth_ifd,
            "Exif": exif_ifd,
            "GPS":  gps_ifd,
        }

        try:
            exif_bytes = piexif.dump(exif_dict)
            # Insert EXIF ke JPEG
            result = piexif.insert(exif_bytes, jpeg_bytes)
            return result
        except Exception as e:
            log.warning(f"EXIF inject gagal: {e} – gambar dikirim tanpa EXIF")
            return jpeg_bytes


# ─────────────────────────────────────────────────────────────────────────────
# UDP Sender (sama dengan sender.py – fragment besar)
# ─────────────────────────────────────────────────────────────────────────────
class UDPSender:
    HEADER_FMT  = "!III"
    HEADER_SIZE = struct.calcsize(HEADER_FMT)

    def __init__(self, host: str, port: int, chunk: int = config.CHUNK_SIZE):
        self.host      = host
        self.port      = port
        self.chunk     = chunk
        self.sock      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_id  = 0

    def send(self, data: bytes) -> int:
        chunks    = [data[i:i+self.chunk] for i in range(0, len(data), self.chunk)]
        n         = len(chunks)
        self.frame_id = (self.frame_id + 1) % (2**32)
        sent = 0
        for idx, chunk in enumerate(chunks):
            hdr    = struct.pack(self.HEADER_FMT, self.frame_id, n, idx)
            self.sock.sendto(hdr + chunk, (self.host, self.port))
            sent  += len(hdr) + len(chunk)
        return sent

    def close(self):
        self.sock.close()


# ─────────────────────────────────────────────────────────────────────────────
# Metadata Sender – kirim JSON metadata bersamaan dengan gambar
# ─────────────────────────────────────────────────────────────────────────────
class MetaSender:
    """Kirim metadata GPS sebagai paket JSON kecil terpisah."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame_id: int, filename: str, gps: dict):
        payload = json.dumps({
            "type":      "photo_meta",
            "frame_id":  frame_id,
            "filename":  filename,
            "timestamp": gps.get("timestamp", time.time()),
            "latitude":  round(gps.get("lat", 0.0), 7),
            "longitude": round(gps.get("lon", 0.0), 7),
            "altitude":  round(gps.get("alt_abs", 0.0), 2),
            "alt_agl":   round(gps.get("alt_rel", 0.0), 2),
            "roll_deg":  round(gps.get("roll_deg", 0.0), 2),
            "pitch_deg": round(gps.get("pitch_deg", 0.0), 2),
            "yaw_deg":   round(gps.get("yaw_deg", 0.0), 2),
            "gps_fix":   gps.get("fix_type", 0),
            "sats":      gps.get("sats", 0),
        }).encode("utf-8")
        try:
            self.sock.sendto(payload, (self.host, self.port))
        except Exception as e:
            log.debug(f"Meta send error: {e}")

    def close(self):
        self.sock.close()


# ─────────────────────────────────────────────────────────────────────────────
# Camera
# ─────────────────────────────────────────────────────────────────────────────
def open_camera(index: int) -> cv2.VideoCapture:
    """
    Buka DJI Osmo Action 5 Pro dalam mode FOTO (UVC).
    Resolusi di-set ke max yang didukung via USB.
    """
    log.info(f"Membuka kamera index {index} (DJI Osmo Action 5 Pro) ...")

    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Kamera index {index} tidak bisa dibuka!")

    # Osmo Action 5 Pro via UVC: set ke 4K jika memungkinkan
    # Jika bandwidth terbatas, turunkan ke 1080p
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)       # Kamera tetap berjalan di 30fps

    # MJPEG buffer: ambil frame terbaru, bukan yang lama (burst 1 frame)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Gunakan format MJPEG agar lebih cepat dari USB
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log.info(f"✓ Kamera terbuka: {w}x{h}")
    return cap


def grab_latest_frame(cap: cv2.VideoCapture) -> np.ndarray:
    """
    Ambil frame TERBARU dari kamera.
    Teknik: flush buffer dengan grab() tanpa retrieve(), lalu retrieve sekali.
    Ini memastikan kita dapat gambar yang paling fresh (bukan yang tertunda di buffer).
    """
    # Flush semua frame lama di buffer
    for _ in range(3):
        cap.grab()
    # Ambil frame terbaru
    ret, frame = cap.retrieve()
    if not ret:
        raise RuntimeError("Gagal retrieve frame dari kamera")
    return frame


# ─────────────────────────────────────────────────────────────────────────────
# Local Save
# ─────────────────────────────────────────────────────────────────────────────
def save_local(jpeg_bytes: bytes, save_dir: Path, filename: str):
    """Simpan gambar lokal ke Raspi (sebagai backup / log)."""
    path = save_dir / filename
    with open(path, "wb") as f:
        f.write(jpeg_bytes)
    return path


# ─────────────────────────────────────────────────────────────────────────────
# Main Loop
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="NOVA Photo Capture – 1 foto/detik dengan EXIF GPS dari FC"
    )
    p.add_argument("--host",       type=str,   default=config.GROUND_IP)
    p.add_argument("--img-port",   type=int,   default=config.IMAGE_PORT)
    p.add_argument("--meta-port",  type=int,   default=config.OD_PORT)
    p.add_argument("--fc-port",    type=str,   default=config.FC_SERIAL_PORT)
    p.add_argument("--fc-baud",    type=int,   default=config.FC_BAUD_RATE)
    p.add_argument("--cam-index",  type=int,   default=config.CAMERA_INDEX)
    p.add_argument("--interval",   type=float, default=1.0,
                   help="Interval foto (detik). Default 1.0 = 1 fps")
    p.add_argument("--quality",    type=int,   default=config.JPEG_QUALITY)
    p.add_argument("--no-fc",      action="store_true",
                   help="Tanpa FC (simulasi GPS dummy)")
    p.add_argument("--save-local", action="store_true", default=True,
                   help="Simpan gambar lokal di Raspi")
    p.add_argument("--local-dir",  type=str,   default="/tmp/nova_photos")
    p.add_argument("--no-imu-gate", action="store_true",
                   help="Nonaktifkan IMU gate (foto meski berbelok)")
    return p.parse_args()


def main():
    args = parse_args()

    log.info("=" * 60)
    log.info("PROGRAM-NOVA Photo Capture v1.0")
    log.info(f"  Ground IP   : {args.host}:{args.img_port}")
    log.info(f"  Interval    : {args.interval}s ({1/args.interval:.1f} fps)")
    log.info(f"  FC          : {'BYPASS' if args.no_fc else args.fc_port}")
    log.info(f"  IMU gate    : {'OFF' if args.no_imu_gate else 'ON'}")
    log.info(f"  Simpan lokal: {args.save_local} ({args.local_dir})")
    log.info("=" * 60)

    # Buat direktori lokal
    local_dir = Path(args.local_dir)
    local_dir.mkdir(parents=True, exist_ok=True)

    # Komponen
    gps_state   = GPSState()
    exif_inj    = EXIFInjector()
    img_sender  = UDPSender(args.host, args.img_port)
    meta_sender = MetaSender(args.host, args.meta_port)

    # MAVLink thread
    mav_thread  = None
    if not args.no_fc and MAVLINK_OK:
        conn_str = f"{args.fc_port},{args.fc_baud}" \
                   if "udp" not in args.fc_port.lower() else args.fc_port
        mav_thread = MAVLinkThread(conn_str, gps_state)
        mav_thread.start()
        log.info("Menunggu koneksi FC (5 detik) ...")
        time.sleep(5)
    else:
        # GPS dummy progresif untuk simulasi
        log.warning("Mode --no-fc: GPS dummy digunakan (tidak ada EXIF GPS nyata)")

    # Kamera
    cap = open_camera(args.cam_index)

    photo_count = 0
    skip_count  = 0

    log.info("Mulai mengambil foto. Tekan Ctrl+C untuk berhenti.")
    try:
        while True:
            t_start = time.time()

            # Ambil GPS snapshot
            gps = gps_state.snapshot()

            # Jika GPS dummy
            if args.no_fc:
                gps.update({
                    "lat": -7.123456 + photo_count * 0.00005,
                    "lon": 112.654321 + photo_count * 0.00005,
                    "alt_abs": 80.0,
                    "alt_rel": 75.0,
                    "roll_deg": 2.0,
                    "pitch_deg": 1.5,
                    "yaw_deg": 90.0,
                    "fix_type": 3,
                    "sats": 12,
                    "hdop": 0.9,
                    "is_stable": True,
                    "valid": True,
                    "timestamp": time.time(),
                })

            # ── IMU Gate ─────────────────────────────────────────────────
            if not args.no_imu_gate and not gps["is_stable"]:
                skip_count += 1
                log.warning(
                    f"⚠ TAHAN (berbelok) | "
                    f"Roll={gps['roll_deg']:.1f}° Pitch={gps['pitch_deg']:.1f}°"
                )
                # Flush buffer kamera agar tidak stale
                cap.grab()
                elapsed = time.time() - t_start
                time.sleep(max(0, args.interval - elapsed))
                continue

            # ── Ambil frame dari kamera ───────────────────────────────────
            try:
                frame = grab_latest_frame(cap)
            except Exception as e:
                log.error(f"Gagal ambil frame: {e}")
                time.sleep(0.5)
                continue

            # ── Encode ke JPEG ────────────────────────────────────────────
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
            ret, buf = cv2.imencode(".jpg", frame, encode_params)
            if not ret:
                log.error("Encode JPEG gagal")
                continue
            jpeg_bytes = buf.tobytes()

            # ── Inject EXIF GPS ───────────────────────────────────────────
            if gps["valid"] or args.no_fc:
                jpeg_bytes = exif_inj.inject(jpeg_bytes, gps)
                gps_str    = f"lat={gps['lat']:.6f} lon={gps['lon']:.6f} alt={gps['alt_abs']:.1f}m"
            else:
                gps_str = "GPS NO FIX"

            # ── Nama file ─────────────────────────────────────────────────
            ts       = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")[:19]
            filename = f"NOVA_{ts}_{photo_count:05d}.jpg"

            # ── Simpan lokal ──────────────────────────────────────────────
            if args.save_local:
                save_local(jpeg_bytes, local_dir, filename)

            # ── Kirim via UDP ─────────────────────────────────────────────
            try:
                bytes_sent = img_sender.send(jpeg_bytes)
                meta_sender.send(img_sender.frame_id, filename, gps)
            except Exception as e:
                log.error(f"UDP send error: {e}")
                bytes_sent = 0

            photo_count += 1
            log.info(
                f"📷 Foto #{photo_count} | {filename} | "
                f"{len(jpeg_bytes)/1024:.0f}KB → {bytes_sent/1024:.0f}KB UDP | "
                f"{gps_str} | Sats={gps['sats']} Fix={gps['fix_type']}"
            )

            # ── Throttle ──────────────────────────────────────────────────
            elapsed = time.time() - t_start
            sleep_t = max(0.0, args.interval - elapsed)
            time.sleep(sleep_t)

    except KeyboardInterrupt:
        log.info("\nDihentikan.")
    finally:
        cap.release()
        img_sender.close()
        meta_sender.close()
        log.info(
            f"Selesai. Foto diambil: {photo_count} | "
            f"Ditahan IMU gate: {skip_count} | "
            f"Tersimpan di: {local_dir}"
        )


if __name__ == "__main__":
    main()
