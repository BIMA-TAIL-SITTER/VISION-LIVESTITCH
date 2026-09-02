#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-NOVA | sender.py
=============================================================================
Dijalankan di: Raspberry Pi 5 (di dalam UAV Fixed Wing)

Fungsi:
  1. Membuka kamera DJI Osmo Action 5 Pro (via USB/UVC sebagai webcam)
  2. Membaca attitude (roll/pitch/yaw) dari Flight Controller via MAVLink
  3. Hanya mengirim frame jika pesawat STABIL (tidak sedang berbelok)
  4. Mengirim frame JPEG via UDP ke ground station
  5. Menerima paket Object Detection dari program OD di FC dan meneruskannya
    ke ground via UDP JSON

Cara Jalankan di Raspi:
  python3 sender.py --host 192.168.1.100 --img-port 5600 --od-port 5601

Untuk simulasi tanpa FC (--no-fc):
  python3 sender.py --host 192.168.1.100 --no-fc
=============================================================================
"""

import socket
import struct
import time
import threading
import json
import argparse
import logging
import sys
import os

import cv2
import numpy as np

# Tambahkan path agar bisa import config
sys.path.insert(0, os.path.dirname(__file__))
import config

# ── MAVLink (opsional, skip jika tidak tersedia) ──────────────────────────────
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[SENDER] WARNING: pymavlink tidak tersedia. Gunakan --no-fc untuk simulasi.")

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [SENDER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("sender")


# =============================================================================
# IMU Monitor – membaca attitude dari FC via MAVLink
# =============================================================================
class IMUMonitor(threading.Thread):
    """
    Thread yang terus-menerus polling pesan ATTITUDE dari FC via MAVLink.
    Menyimpan roll/pitch/yaw terbaru.
    Memberikan flag `is_stable` apabila pesawat dalam kondisi aman untuk foto.
    """

    def __init__(self, port: str, baud: int,
                 roll_thresh: float = config.ROLL_THRESHOLD_DEG,
                 pitch_thresh: float = config.PITCH_THRESHOLD_DEG,
                 stable_count: int = config.STABLE_FRAME_COUNT):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.roll_thresh  = roll_thresh
        self.pitch_thresh = pitch_thresh
        self.stable_count = stable_count

        self.roll_deg   = 0.0
        self.pitch_deg  = 0.0
        self.yaw_deg    = 0.0
        self._consecutive_stable = 0
        self.is_stable  = True   # Default true (aman) sampai FC terhubung

        self._lock      = threading.Lock()
        self._connected = False
        self._mav       = None

    def run(self):
        log.info(f"Menghubungkan ke FC di {self.port} @ {self.baud} baud ...")
        try:
            self._mav = mavutil.mavlink_connection(self.port, baud=self.baud)
            self._mav.wait_heartbeat(timeout=10)
            log.info("✓ FC terhubung! Memulai monitoring attitude...")
            self._connected = True
        except Exception as e:
            log.error(f"Gagal terhubung ke FC: {e}")
            log.warning("IMU Monitor berjalan dalam mode BYPASS (semua frame dikirim).")
            return

        # Request stream ATTITUDE dari FC
        self._mav.mav.request_data_stream_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
            10,  # 10 Hz
            1    # Start
        )

        while True:
            try:
                msg = self._mav.recv_match(type='ATTITUDE', blocking=True, timeout=2)
                if msg is None:
                    continue

                r = abs(math_degrees(msg.roll))
                p = abs(math_degrees(msg.pitch))
                y = math_degrees(msg.yaw)

                with self._lock:
                    self.roll_deg  = r
                    self.pitch_deg = p
                    self.yaw_deg   = y

                    # Kondisi stabil: roll dan pitch di bawah threshold
                    currently_stable = (r < self.roll_thresh and p < self.pitch_thresh)

                    if currently_stable:
                        self._consecutive_stable = min(
                            self._consecutive_stable + 1, self.stable_count
                        )
                    else:
                        self._consecutive_stable = 0

                    # Butuh N frame berturut-turut stabil sebelum flag menyala
                    self.is_stable = (self._consecutive_stable >= self.stable_count)

            except Exception as e:
                log.warning(f"IMU read error: {e}")
                time.sleep(0.5)

    @property
    def attitude(self):
        with self._lock:
            return self.roll_deg, self.pitch_deg, self.yaw_deg, self.is_stable

    def connected(self):
        return self._connected


def math_degrees(rad: float) -> float:
    """Konversi radian ke derajat."""
    import math
    return math.degrees(rad)


# =============================================================================
# Frame Sender – kirim gambar via UDP dengan fragmentasi
# =============================================================================
class FrameSender:
    """
    Mengirim frame JPEG melalui UDP.
    Karena UDP max payload 65507 byte, gambar besar di-fragment.

    Format paket fragment:
      [4B frame_id][4B total_chunks][4B chunk_idx][data...]
    """

    HEADER_FMT  = "!III"   # frame_id, total_chunks, chunk_idx
    HEADER_SIZE = struct.calcsize(HEADER_FMT)

    def __init__(self, host: str, port: int, chunk_size: int = config.CHUNK_SIZE):
        self.host       = host
        self.port       = port
        self.chunk_size = chunk_size
        self.sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_id   = 0

    def send(self, jpeg_bytes: bytes) -> int:
        """Kirim gambar. Return jumlah byte terkirim."""
        data        = jpeg_bytes
        total       = len(data)
        chunks      = [data[i:i + self.chunk_size]
                       for i in range(0, total, self.chunk_size)]
        n_chunks    = len(chunks)
        self.frame_id = (self.frame_id + 1) % (2**32)

        bytes_sent = 0
        for idx, chunk in enumerate(chunks):
            header  = struct.pack(self.HEADER_FMT, self.frame_id, n_chunks, idx)
            packet  = header + chunk
            self.sock.sendto(packet, (self.host, self.port))
            bytes_sent += len(packet)

        return bytes_sent

    def close(self):
        self.sock.close()


# =============================================================================
# OD Forwarder – teruskan paket object detection dari port lokal ke ground
# =============================================================================
class ODForwarder(threading.Thread):
    """
    Menerima paket JSON dari proses OD lokal (port LOKAL 5699)
    dan meneruskan ke ground station.

    Dalam skenario nyata, program OD di FC mengirim ke localhost:5699,
    lalu thread ini meneruskan ke ground IP:OD_PORT.
    """

    LOCAL_OD_PORT = 5699  # Port lokal tempat program OD mengirim

    def __init__(self, ground_host: str, ground_port: int):
        super().__init__(daemon=True)
        self.ground_host = ground_host
        self.ground_port = ground_port
        self._sock_in    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock_out   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        try:
            self._sock_in.bind(("0.0.0.0", self.LOCAL_OD_PORT))
            log.info(f"OD Forwarder mendengarkan di port {self.LOCAL_OD_PORT} ...")
        except OSError as e:
            log.warning(f"OD Forwarder tidak bisa bind: {e}")
            return

        while True:
            try:
                data, _ = self._sock_in.recvfrom(65507)
                # Validasi JSON
                try:
                    json.loads(data.decode("utf-8"))
                except Exception:
                    continue
                # Forward ke ground
                self._sock_out.sendto(data, (self.ground_host, self.ground_port))
                log.debug(f"OD paket diteruskan ke {self.ground_host}:{self.ground_port}")
            except Exception as e:
                log.error(f"ODForwarder error: {e}")
                time.sleep(0.1)


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="NOVA Sender – UAV side image streaming dengan IMU gate"
    )
    parser.add_argument("--host",      type=str, default=config.GROUND_IP,
                        help="IP ground station")
    parser.add_argument("--img-port",  type=int, default=config.IMAGE_PORT,
                        help="UDP port untuk stream gambar")
    parser.add_argument("--od-port",   type=int, default=config.OD_PORT,
                        help="UDP port untuk object detection ke ground")
    parser.add_argument("--fc-port",   type=str, default=config.FC_SERIAL_PORT,
                        help="Serial port FC (e.g. /dev/ttyAMA0)")
    parser.add_argument("--fc-baud",   type=int, default=config.FC_BAUD_RATE,
                        help="Baud rate FC")
    parser.add_argument("--no-fc",     action="store_true",
                        help="Nonaktifkan IMU monitoring (kirim semua frame)")
    parser.add_argument("--cam-index", type=int, default=config.CAMERA_INDEX,
                        help="Index kamera OpenCV")
    parser.add_argument("--interval",  type=float, default=config.FRAME_INTERVAL,
                        help="Interval antar frame (detik)")
    parser.add_argument("--quality",   type=int, default=config.JPEG_QUALITY,
                        help="Kualitas JPEG (0-100)")
    return parser.parse_args()


def open_camera(index: int) -> cv2.VideoCapture:
    """Buka kamera dengan pengaturan optimal untuk DJI Osmo Action 5 Pro."""
    log.info(f"Membuka kamera index {index} ...")

    # Coba backend V4L2 dulu (Linux/Raspi), fallback ke default
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        log.warning("V4L2 gagal, mencoba default backend ...")
        cap = cv2.VideoCapture(index)

    if not cap.isOpened():
        raise RuntimeError(f"Tidak bisa membuka kamera index {index}")

    # Set resolusi – DJI Osmo Action 5 Pro sebagai UVC webcam
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    # Disable auto-exposure untuk kestabilan exposure
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual, 3 = auto (driver-dependent)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_f = cap.get(cv2.CAP_PROP_FPS)
    log.info(f"✓ Kamera terbuka: {actual_w}x{actual_h} @ {actual_f:.1f}fps")
    return cap


def encode_frame(frame: np.ndarray, quality: int) -> bytes:
    """Encode frame ke JPEG bytes."""
    params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    ret, buf = cv2.imencode(".jpg", frame, params)
    if not ret:
        raise RuntimeError("Gagal encode JPEG")
    return buf.tobytes()


def main():
    args   = parse_args()
    log.info("=" * 60)
    log.info("PROGRAM-NOVA Sender v1.0")
    log.info(f"  Ground station : {args.host}:{args.img_port}")
    log.info(f"  OD port        : {args.host}:{args.od_port}")
    log.info(f"  FC             : {'BYPASS (--no-fc)' if args.no_fc else args.fc_port}")
    log.info(f"  Interval frame : {args.interval}s")
    log.info("=" * 60)

    # ── 1. Buka kamera ────────────────────────────────────────────────────────
    cap = open_camera(args.cam_index)

    # ── 2. IMU Monitor ────────────────────────────────────────────────────────
    imu = None
    if not args.no_fc and MAVLINK_AVAILABLE:
        imu = IMUMonitor(args.fc_port, args.fc_baud)
        imu.start()
        log.info("IMU Monitor dimulai, menunggu heartbeat FC ...")
        time.sleep(3)  # Beri waktu koneksi
    else:
        log.warning("IMU Monitor DINONAKTIFKAN – semua frame akan dikirim.")

    # ── 3. Frame Sender ───────────────────────────────────────────────────────
    sender = FrameSender(args.host, args.img_port)

    # ── 4. OD Forwarder ───────────────────────────────────────────────────────
    od_fwd = ODForwarder(args.host, args.od_port)
    od_fwd.start()

    # ── 5. Loop utama ─────────────────────────────────────────────────────────
    frame_count   = 0
    skipped_count = 0
    last_frame_t  = 0.0

    log.info("Memulai loop streaming. Tekan Ctrl+C untuk berhenti.")
    try:
        while True:
            loop_start = time.time()

            # Baca frame dari kamera
            ret, frame = cap.read()
            if not ret:
                log.error("Gagal membaca frame dari kamera!")
                time.sleep(0.5)
                continue

            # ── IMU Gate ─────────────────────────────────────────────────────
            if imu is not None:
                roll, pitch, yaw, stable = imu.attitude
                status_str = f"R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°"

                if not stable:
                    skipped_count += 1
                    if skipped_count % 10 == 1:
                        log.warning(
                            f"⚠ BERBELOK – Frame DITAHAN | {status_str} | "
                            f"Threshold Roll>{config.ROLL_THRESHOLD_DEG}° "
                            f"atau Pitch>{config.PITCH_THRESHOLD_DEG}°"
                        )
                    # Tetap baca frame agar buffer kamera tidak penuh
                    elapsed = time.time() - loop_start
                    sleep_t = max(0, args.interval - elapsed)
                    time.sleep(sleep_t)
                    continue
                else:
                    log.debug(f"✓ STABIL | {status_str}")
            # ── End IMU Gate ──────────────────────────────────────────────────

            # Encode frame
            try:
                jpeg_bytes = encode_frame(frame, args.quality)
            except Exception as e:
                log.error(f"Encode error: {e}")
                continue

            # Kirim frame
            try:
                bytes_sent = sender.send(jpeg_bytes)
                frame_count += 1
                log.info(
                    f"[Frame {frame_count}] Terkirim {bytes_sent/1024:.1f} KB "
                    f"({len(jpeg_bytes)/1024:.1f} KB JPEG)"
                )
            except Exception as e:
                log.error(f"Send error: {e}")

            # Throttle ke interval yang ditentukan
            elapsed = time.time() - loop_start
            sleep_t = max(0, args.interval - elapsed)
            time.sleep(sleep_t)

    except KeyboardInterrupt:
        log.info("\nDihentikan oleh pengguna.")
    finally:
        log.info(f"Ringkasan: {frame_count} frame terkirim, {skipped_count} frame ditahan (IMU gate).")
        cap.release()
        sender.close()
        log.info("Selesai.")


if __name__ == "__main__":
    main()
