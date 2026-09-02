#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-SENDER | sender.py
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
import io

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

# ── piexif (opsional, untuk embed GPS ke EXIF gambar) ─────────────────────────
try:
    import piexif
    PIEXIF_AVAILABLE = True
except ImportError:
    PIEXIF_AVAILABLE = False
    print("[SENDER] WARNING: piexif tidak tersedia. GPS tidak akan di-embed ke EXIF. Install: pip install piexif")

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [SENDER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("sender")


# =============================================================================
# Flight Data Monitor – membaca attitude & GPS dari FC via MAVLink
# =============================================================================
class FlightDataMonitor(threading.Thread):
    """
    Thread yang terus-menerus polling pesan ATTITUDE & GLOBAL_POSITION_INT dari FC.
    Menyimpan roll/pitch/yaw serta lat/lon/altitude terbaru.
    Memberikan flag `is_stable` apabila pesawat dalam kondisi aman untuk foto,
    dan `has_gps_fix` apabila FC sudah punya fix GPS yang valid.
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

        # Attitude
        self.roll_deg   = 0.0
        self.pitch_deg  = 0.0
        self.yaw_deg    = 0.0
        self._consecutive_stable = 0
        self.is_stable  = True   # Default true (aman) sampai FC terhubung

        # GPS
        self.lat            = 0.0
        self.lon            = 0.0
        self.alt_msl_m      = 0.0   # Altitude MSL (mean sea level), dari GLOBAL_POSITION_INT.alt
        self.alt_rel_m      = 0.0   # Altitude relatif ke titik takeoff/home, dari .relative_alt
        self.fix_type       = 0     # 0-1 = no fix, 2 = 2D, 3 = 3D, 4+ = DGPS/RTK
        self.satellites_visible = 0
        self.has_gps_fix    = False

        self._lock      = threading.Lock()
        self._connected = False
        self._mav       = None

    def run(self):
        log.info(f"Menghubungkan ke FC di {self.port} @ {self.baud} baud ...")
        try:
            self._mav = mavutil.mavlink_connection(self.port, baud=self.baud)
            self._mav.wait_heartbeat(timeout=10)
            log.info("✓ FC terhubung! Memulai monitoring attitude & GPS...")
            self._connected = True
        except Exception as e:
            log.error(f"Gagal terhubung ke FC: {e}")
            log.warning("Flight Data Monitor berjalan dalam mode BYPASS (semua frame dikirim, tanpa GPS).")
            return

        # Request stream ATTITUDE (EXTRA1) & POSITION (GLOBAL_POSITION_INT, GPS_RAW_INT) dari FC
        for stream_id, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  10),  # ATTITUDE @ 10Hz
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),  # GLOBAL_POSITION_INT @ 5Hz
            (mavutil.mavlink.MAV_DATA_STREAM_ALL,      2),  # cadangan, termasuk GPS_RAW_INT
        ]:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system, self._mav.target_component,
                stream_id, rate, 1
            )

        while True:
            try:
                msg = self._mav.recv_match(
                    type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
                    blocking=True, timeout=2
                )
                if msg is None:
                    continue
                msg_type = msg.get_type()

                if msg_type == 'ATTITUDE':
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

                elif msg_type == 'GLOBAL_POSITION_INT':
                    # lat/lon dalam 1e7 derajat, alt dalam mm
                    with self._lock:
                        self.lat       = msg.lat / 1e7
                        self.lon       = msg.lon / 1e7
                        self.alt_msl_m = msg.alt / 1000.0
                        self.alt_rel_m = msg.relative_alt / 1000.0

                elif msg_type == 'GPS_RAW_INT':
                    with self._lock:
                        self.fix_type          = msg.fix_type
                        self.satellites_visible = msg.satellites_visible
                        self.has_gps_fix       = msg.fix_type >= 3  # 3D fix ke atas

            except Exception as e:
                log.warning(f"Flight data read error: {e}")
                time.sleep(0.5)

    @property
    def attitude(self):
        with self._lock:
            return self.roll_deg, self.pitch_deg, self.yaw_deg, self.is_stable

    @property
    def gps(self):
        """Return dict GPS terkini. altitude = altitude relatif (AGL), dipakai untuk EXIF & footprint."""
        with self._lock:
            return {
                "latitude":  self.lat,
                "longitude": self.lon,
                "altitude":  self.alt_rel_m,
                "altitude_msl": self.alt_msl_m,
                "fix_type":  self.fix_type,
                "satellites": self.satellites_visible,
                "has_fix":   self.has_gps_fix,
            }

    def connected(self):
        return self._connected


# Alias biar backward compatible kalau ada kode lain yang masih merujuk nama lama
IMUMonitor = FlightDataMonitor


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
        description="Sender – UAV side image streaming dengan IMU gate"
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
    parser.add_argument("--roll-thresh",  type=float, default=config.ROLL_THRESHOLD_DEG,
                        help="Batas roll (derajat) sebelum capture ditahan")
    parser.add_argument("--pitch-thresh", type=float, default=config.PITCH_THRESHOLD_DEG,
                        help="Batas pitch (derajat) sebelum capture ditahan")
    parser.add_argument("--no-gps-exif", action="store_true",
                        help="Nonaktifkan embed GPS ke EXIF gambar (default: aktif jika ada fix GPS)")
    parser.add_argument("--status-interval", type=float, default=3.0,
                        help="Interval log status GPS/attitude berkala (detik, default 3.0)")
    parser.add_argument("--require-gps-fix", action="store_true",
                        help="Tahan capture juga kalau FC belum punya GPS fix (default: tetap capture, EXIF tanpa GPS)")
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


def _deg_to_dms_rational(deg_float: float):
    """Konversi derajat desimal ke format DMS rational yang dipakai EXIF GPS."""
    deg_float = abs(deg_float)
    d = int(deg_float)
    m_float = (deg_float - d) * 60
    m = int(m_float)
    s = (m_float - m) * 60
    return ((d, 1), (m, 1), (int(round(s * 100)), 100))


def embed_flight_metadata_exif(jpeg_bytes: bytes, lat: float, lon: float, alt_m: float,
                                roll_deg: float = 0.0, pitch_deg: float = 0.0,
                                yaw_deg: float = 0.0) -> bytes:
    """
    Sisipkan data GPS + attitude (roll/pitch/yaw) ke EXIF gambar JPEG.

    - GPS (lat/lon/altitude) disimpan di GPS IFD standar (GPSLatitude, GPSLongitude,
      GPSAltitude) — dipakai stitcher.py buat GPS threshold filter & georeferencing.
    - Yaw/heading JUGA disimpan di field GPS standar GPSImgDirection.
    - Roll & pitch disimpan sebagai JSON compact di ImageDescription (0th IFD) karena
      EXIF tidak punya field baku untuk attitude pesawat. Dipakai stitcher.py buat
      attitude threshold filter (menyaring ulang gambar yang terlalu miring sebelum
      ikut proses stitching) dan sebagai input orientasi untuk Combiner.

    Semua data ini TIDAK PERLU dikirim terpisah — cukup baca ulang dari file JPEG-nya.
    """
    gps_ifd = {
        piexif.GPSIFD.GPSVersionID:    (2, 0, 0, 0),
        piexif.GPSIFD.GPSLatitudeRef:  'S' if lat < 0 else 'N',
        piexif.GPSIFD.GPSLatitude:     _deg_to_dms_rational(lat),
        piexif.GPSIFD.GPSLongitudeRef: 'W' if lon < 0 else 'E',
        piexif.GPSIFD.GPSLongitude:    _deg_to_dms_rational(lon),
        piexif.GPSIFD.GPSAltitudeRef:  0 if alt_m >= 0 else 1,
        piexif.GPSIFD.GPSAltitude:     (int(round(abs(alt_m) * 100)), 100),
        piexif.GPSIFD.GPSImgDirectionRef: 'T',  # 'T' = true north (bukan magnetic)
        piexif.GPSIFD.GPSImgDirection:    (int(round((yaw_deg % 360) * 100)), 100),
    }

    attitude_json = json.dumps({
        "roll":  round(roll_deg, 2),
        "pitch": round(pitch_deg, 2),
        "yaw":   round(yaw_deg, 2),
    })
    zeroth_ifd = {
        piexif.ImageIFD.ImageDescription: attitude_json.encode("utf-8"),
    }

    exif_bytes = piexif.dump({"GPS": gps_ifd, "0th": zeroth_ifd})

    out = io.BytesIO()
    piexif.insert(exif_bytes, jpeg_bytes, out)
    return out.getvalue()


# Alias biar backward compatible kalau ada kode lain yang masih merujuk nama lama
embed_gps_exif = embed_flight_metadata_exif


def main():
    args   = parse_args()
    embed_gps = (not args.no_gps_exif) and PIEXIF_AVAILABLE
    if not args.no_gps_exif and not PIEXIF_AVAILABLE:
        log.warning("piexif tidak tersedia — GPS EXIF tidak akan di-embed meskipun tidak pakai --no-gps-exif.")

    log.info("=" * 60)
    log.info("PROGRAM-SENDER Sender v1.0")
    log.info(f"  Ground station : {args.host}:{args.img_port}")
    log.info(f"  OD port        : {args.host}:{args.od_port}")
    log.info(f"  FC             : {'BYPASS (--no-fc)' if args.no_fc else args.fc_port}")
    log.info(f"  Attitude gate  : Roll>{args.roll_thresh}° / Pitch>{args.pitch_thresh}°")
    log.info(f"  GPS EXIF       : {'AKTIF' if embed_gps else 'NONAKTIF'}"
              f"{'  (wajib fix)' if args.require_gps_fix else ''}")
    log.info(f"  Interval frame : {args.interval}s")
    log.info("=" * 60)

    # ── 1. Buka kamera ────────────────────────────────────────────────────────
    cap = open_camera(args.cam_index)

    # ── 2. Flight Data Monitor (attitude + GPS) ──────────────────────────────
    imu = None
    if not args.no_fc and MAVLINK_AVAILABLE:
        imu = FlightDataMonitor(args.fc_port, args.fc_baud,
                                 roll_thresh=args.roll_thresh,
                                 pitch_thresh=args.pitch_thresh)
        imu.start()
        log.info("Flight Data Monitor dimulai, menunggu heartbeat FC ...")
        time.sleep(3)  # Beri waktu koneksi
    else:
        log.warning("Flight Data Monitor DINONAKTIFKAN – semua frame akan dikirim, tanpa GPS EXIF.")

    # ── 3. Frame Sender ───────────────────────────────────────────────────────
    sender = FrameSender(args.host, args.img_port)

    # ── 4. OD Forwarder ───────────────────────────────────────────────────────
    od_fwd = ODForwarder(args.host, args.od_port)
    od_fwd.start()

    # ── 5. Loop utama ─────────────────────────────────────────────────────────
    frame_count      = 0
    skipped_count    = 0
    no_gps_warned    = 0
    last_status_log  = 0.0
    STATUS_INTERVAL  = args.status_interval  # detik, independen dari frame gate — selalu tampil

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

            gps = None
            roll = pitch = yaw = 0.0

            # ── IMU Gate (attitude) ─────────────────────────────────────────
            if imu is not None:
                roll, pitch, yaw, stable = imu.attitude
                gps = imu.gps
                status_str = (
                    f"R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}° | "
                    f"GPS={'FIX' if gps['has_fix'] else 'NO-FIX'} "
                    f"({gps['satellites']} sat) "
                    f"lat={gps['latitude']:.6f} lon={gps['longitude']:.6f} alt={gps['altitude']:.1f}m"
                )

                # ── Monitoring GPS/attitude berkala — selalu tampil tiap
                #    STATUS_INTERVAL detik, TERLEPAS dari stabil/skip/fix,
                #    biar bisa dipantau terus tanpa perlu imu_monitor.py terpisah.
                now_t = time.time()
                if now_t - last_status_log >= STATUS_INTERVAL:
                    icon = "🟢" if stable else "🔴"
                    gps_icon = "📡" if gps["has_fix"] else "🛰️ "
                    log.info(f"{icon} {gps_icon} STATUS | {status_str}")
                    last_status_log = now_t

                if not stable:
                    skipped_count += 1
                    if skipped_count % 10 == 1:
                        log.warning(
                            f"⚠ BERBELOK – Frame DITAHAN | {status_str} | "
                            f"Threshold Roll>{args.roll_thresh}° "
                            f"atau Pitch>{args.pitch_thresh}°"
                        )
                    elapsed = time.time() - loop_start
                    sleep_t = max(0, args.interval - elapsed)
                    time.sleep(sleep_t)
                    continue

                if args.require_gps_fix and not gps["has_fix"]:
                    skipped_count += 1
                    if skipped_count % 10 == 1:
                        log.warning(f"⚠ GPS BELUM FIX – Frame DITAHAN (--require-gps-fix aktif) | {status_str}")
                    elapsed = time.time() - loop_start
                    sleep_t = max(0, args.interval - elapsed)
                    time.sleep(sleep_t)
                    continue

                log.debug(f"✓ STABIL | {status_str}")
            # ── End Gate ──────────────────────────────────────────────────────

            # Encode frame
            try:
                jpeg_bytes = encode_frame(frame, args.quality)
            except Exception as e:
                log.error(f"Encode error: {e}")
                continue

            # ── Embed GPS + attitude ke EXIF ─────────────────────────────────
            if embed_gps and gps is not None:
                if gps["has_fix"]:
                    try:
                        jpeg_bytes = embed_flight_metadata_exif(
                            jpeg_bytes, gps["latitude"], gps["longitude"], gps["altitude"],
                            roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw
                        )
                    except Exception as e:
                        log.warning(f"Gagal embed flight metadata EXIF: {e}")
                else:
                    no_gps_warned += 1
                    if no_gps_warned % 10 == 1:
                        log.warning("GPS belum fix — frame dikirim TANPA GPS/attitude EXIF.")

            # Kirim frame
            try:
                bytes_sent = sender.send(jpeg_bytes)
                frame_count += 1
                gps_tag = ""
                if gps is not None and gps["has_fix"]:
                    gps_tag = f" | GPS {gps['latitude']:.6f},{gps['longitude']:.6f} @{gps['altitude']:.1f}m"
                log.info(
                    f"[Frame {frame_count}] Terkirim {bytes_sent/1024:.1f} KB "
                    f"({len(jpeg_bytes)/1024:.1f} KB JPEG){gps_tag}"
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
        log.info(f"Ringkasan: {frame_count} frame terkirim, {skipped_count} frame ditahan (attitude/GPS gate).")
        cap.release()
        sender.close()
        log.info("Selesai.")



if __name__ == "__main__":
    main()