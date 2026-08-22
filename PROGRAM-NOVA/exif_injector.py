#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-NOVA | exif_injector.py
=============================================================================
Dijalankan di: Raspberry Pi 5 (UAV) – bisa standalone maupun diimport

Fungsi STANDALONE:
  Tool untuk batch-inject EXIF GPS ke semua foto JPG yang sudah ada di folder.
  Berguna jika foto sudah diambil tapi belum ada EXIF (misalnya dari kamera
  yang tidak otomatis menyimpan GPS).

  Membaca GPS dari FC secara REALTIME via MAVLink, kemudian saat foto baru
  terdeteksi di folder, langsung inject EXIF ke file tersebut.

Cara Jalankan:
  # Mode watch – inject EXIF ke foto baru secara real-time
  python3 exif_injector.py --watch-dir /tmp/nova_photos

  # Mode batch – inject EXIF ke semua foto yang sudah ada
  python3 exif_injector.py --batch-dir /tmp/nova_photos

  # Mode manual – inject GPS manual ke satu file
  python3 exif_injector.py --file foto.jpg --lat -7.123 --lon 112.654 --alt 80

=============================================================================
"""

import os
import sys
import time
import math
import json
import struct
import socket
import argparse
import logging
import shutil
import threading
from pathlib import Path
from datetime import datetime

import piexif

sys.path.insert(0, os.path.dirname(__file__))
import config

try:
    from pymavlink import mavutil
    MAVLINK_OK = True
except ImportError:
    MAVLINK_OK = False

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [EXIF] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("exif_injector")


# ─────────────────────────────────────────────────────────────────────────────
# Core EXIF Inject Logic
# ─────────────────────────────────────────────────────────────────────────────
def dd_to_dms_rational(dd: float):
    """Decimal Degrees → DMS rational tuple untuk EXIF."""
    d = int(abs(dd))
    m_f = (abs(dd) - d) * 60
    m   = int(m_f)
    s   = (m_f - m) * 60
    return ((d, 1), (m, 1), (int(s * 100000), 100000))


def to_rational(val: float, precision: int = 1000000) -> tuple:
    return (int(abs(val) * precision), precision)


def inject_exif_to_file(
    filepath: Path,
    lat: float,
    lon: float,
    alt_msl: float,
    alt_agl: float   = 0.0,
    roll_deg: float  = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float   = 0.0,
    sats: int        = 0,
    hdop: float      = 99.0,
    dt: datetime     = None,
    overwrite: bool  = True,
) -> bool:
    """
    Inject GPS EXIF ke file JPEG di tempat (in-place).

    Args:
        filepath : Path ke file .jpg
        lat      : Latitude decimal degrees
        lon      : Longitude decimal degrees
        alt_msl  : Altitude MSL (meter) – masuk ke EXIF GPSAltitude
        alt_agl  : Altitude AGL (meter) – masuk ke UserComment JSON
        roll/pitch/yaw : Attitude dari FC (derajat)
        sats     : Jumlah satelit GPS
        hdop     : Horizontal DOP
        dt       : Datetime UTC (default = now)
        overwrite: Timpa file asli (True) atau simpan ke _exif.jpg (False)

    Return: True jika berhasil
    """
    if dt is None:
        dt = datetime.utcnow()

    filepath = Path(filepath)
    if not filepath.exists():
        log.error(f"File tidak ditemukan: {filepath}")
        return False

    try:
        with open(filepath, "rb") as f:
            jpeg_bytes = f.read()
    except Exception as e:
        log.error(f"Gagal membaca file: {e}")
        return False

    lat_ref = "N" if lat >= 0 else "S"
    lon_ref = "E" if lon >= 0 else "W"

    date_str = dt.strftime("%Y:%m:%d")
    time_str = dt.strftime("%H:%M:%S")

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID:      (2, 3, 0, 0),
        piexif.GPSIFD.GPSLatitudeRef:    lat_ref.encode(),
        piexif.GPSIFD.GPSLatitude:       dd_to_dms_rational(lat),
        piexif.GPSIFD.GPSLongitudeRef:   lon_ref.encode(),
        piexif.GPSIFD.GPSLongitude:      dd_to_dms_rational(lon),
        piexif.GPSIFD.GPSAltitudeRef:    0,
        piexif.GPSIFD.GPSAltitude:       to_rational(max(0.0, alt_msl)),
        piexif.GPSIFD.GPSTimeStamp: (
            (dt.hour, 1), (dt.minute, 1), (dt.second, 1)
        ),
        piexif.GPSIFD.GPSDateStamp:      date_str.encode(),
        piexif.GPSIFD.GPSImgDirectionRef: b"T",
        piexif.GPSIFD.GPSImgDirection:   to_rational(yaw_deg % 360),
        piexif.GPSIFD.GPSMeasureMode:    b"3",
        piexif.GPSIFD.GPSDOP:            to_rational(hdop, 100),
        piexif.GPSIFD.GPSSatellites:     str(sats).encode(),
    }

    zeroth_ifd = {
        piexif.ImageIFD.Make:     b"DJI",
        piexif.ImageIFD.Model:    b"Osmo Action 5 Pro",
        piexif.ImageIFD.Software: b"NOVA-ExifInjector-v1.0",
        piexif.ImageIFD.DateTime: f"{date_str} {time_str}".encode(),
    }

    # UserComment: menyimpan data tambahan (roll, pitch, AGL, dsb)
    extra = json.dumps({
        "source":    "NOVA-MAVLink",
        "alt_agl":   round(alt_agl, 3),
        "roll_deg":  round(roll_deg, 3),
        "pitch_deg": round(pitch_deg, 3),
        "yaw_deg":   round(yaw_deg, 3),
        "sats":      sats,
        "hdop":      round(hdop, 2),
        "ts_utc":    dt.isoformat(),
    })

    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal:  f"{date_str} {time_str}".encode(),
        piexif.ExifIFD.DateTimeDigitized: f"{date_str} {time_str}".encode(),
        piexif.ExifIFD.UserComment:       ("UNICODE\x00" + extra).encode("utf-8"),
    }

    exif_dict = {"0th": zeroth_ifd, "Exif": exif_ifd, "GPS": gps_ifd}

    try:
        exif_bytes = piexif.dump(exif_dict)
        new_jpeg   = piexif.insert(exif_bytes, jpeg_bytes)
    except Exception as e:
        log.error(f"piexif error: {e}")
        return False

    # Simpan
    if overwrite:
        out_path = filepath
    else:
        out_path = filepath.with_stem(filepath.stem + "_exif")

    try:
        with open(out_path, "wb") as f:
            f.write(new_jpeg)
        return True
    except Exception as e:
        log.error(f"Gagal menulis file: {e}")
        return False


def read_exif_gps(filepath: Path) -> dict:
    """Baca EXIF GPS dari file JPEG. Return dict atau None."""
    try:
        with open(filepath, "rb") as f:
            exif_data = piexif.load(f.read())
        gps = exif_data.get("GPS", {})
        if not gps:
            return None

        def rational_to_float(val):
            return val[0] / val[1] if val[1] != 0 else 0.0

        def dms_to_dd(dms):
            d = rational_to_float(dms[0])
            m = rational_to_float(dms[1])
            s = rational_to_float(dms[2])
            return d + m / 60 + s / 3600

        lat  = dms_to_dd(gps.get(piexif.GPSIFD.GPSLatitude, ((0,1),(0,1),(0,1))))
        lon  = dms_to_dd(gps.get(piexif.GPSIFD.GPSLongitude, ((0,1),(0,1),(0,1))))
        alt  = rational_to_float(gps.get(piexif.GPSIFD.GPSAltitude, (0, 1)))

        lat_ref = gps.get(piexif.GPSIFD.GPSLatitudeRef, b"N").decode()
        lon_ref = gps.get(piexif.GPSIFD.GPSLongitudeRef, b"E").decode()

        if lat_ref == "S": lat = -lat
        if lon_ref == "W": lon = -lon

        return {"latitude": lat, "longitude": lon, "altitude": alt}
    except Exception:
        return None


# ─────────────────────────────────────────────────────────────────────────────
# MAVLink GPS Reader (standalone – untuk mode watch)
# ─────────────────────────────────────────────────────────────────────────────
class LiveGPS:
    """Membaca GPS dari FC secara realtime untuk mode watch."""

    def __init__(self, conn_str: str):
        self.conn_str  = conn_str
        self._lock     = threading.Lock()
        self._data     = {
            "lat": 0.0, "lon": 0.0, "alt_msl": 0.0, "alt_agl": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "sats": 0, "hdop": 99.0, "fix": 0, "valid": False
        }
        self._thread   = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def get(self) -> dict:
        with self._lock:
            return dict(self._data)

    def _run(self):
        if not MAVLINK_OK:
            log.error("pymavlink tidak tersedia!")
            return
        try:
            mav = mavutil.mavlink_connection(self.conn_str)
            mav.wait_heartbeat(timeout=15)
            log.info("✓ MAVLink GPS connected")

            for sid, rate in [
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),
            ]:
                mav.mav.request_data_stream_send(
                    mav.target_system, mav.target_component, sid, rate, 1
                )

            while True:
                msg = mav.recv_match(
                    type=["GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE"],
                    blocking=True, timeout=2.0
                )
                if msg is None: continue
                t = msg.get_type()

                with self._lock:
                    if t == "GLOBAL_POSITION_INT":
                        self._data["lat"]     = msg.lat / 1e7
                        self._data["lon"]     = msg.lon / 1e7
                        self._data["alt_msl"] = msg.alt / 1000.0
                        self._data["alt_agl"] = msg.relative_alt / 1000.0
                        self._data["valid"]   = True

                    elif t == "GPS_RAW_INT":
                        self._data["fix"]  = msg.fix_type
                        self._data["sats"] = msg.satellites_visible
                        self._data["hdop"] = msg.eph / 100.0

                    elif t == "ATTITUDE":
                        self._data["roll"]  = math.degrees(msg.roll)
                        self._data["pitch"] = math.degrees(msg.pitch)
                        self._data["yaw"]   = math.degrees(msg.yaw) % 360

        except Exception as e:
            log.error(f"LiveGPS error: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Mode: Watch Directory
# ─────────────────────────────────────────────────────────────────────────────
def mode_watch(watch_dir: Path, live_gps: LiveGPS, poll_s: float = 0.5):
    """
    Monitor folder secara real-time.
    Setiap foto baru yang masuk langsung di-inject EXIF GPS.
    """
    processed = set()
    log.info(f"📂 Watching: {watch_dir} (poll {poll_s}s)")

    while True:
        files = sorted(watch_dir.glob("*.jpg")) + sorted(watch_dir.glob("*.jpeg"))
        for f in files:
            if f.name in processed:
                continue
            # Tunggu file stabil (tidak sedang ditulis)
            size_before = f.stat().st_size
            time.sleep(0.2)
            if f.stat().st_size != size_before:
                continue  # Masih ditulis

            gps = live_gps.get()
            ok  = inject_exif_to_file(
                filepath  = f,
                lat       = gps["lat"],
                lon       = gps["lon"],
                alt_msl   = gps["alt_msl"],
                alt_agl   = gps["alt_agl"],
                roll_deg  = gps["roll"],
                pitch_deg = gps["pitch"],
                yaw_deg   = gps["yaw"],
                sats      = gps["sats"],
                hdop      = gps["hdop"],
                overwrite = True,
            )
            processed.add(f.name)
            status = "✓ EXIF injected" if ok else "✗ EXIF gagal"
            log.info(
                f"{status}: {f.name} | "
                f"lat={gps['lat']:.6f} lon={gps['lon']:.6f} "
                f"alt={gps['alt_msl']:.1f}m | sats={gps['sats']}"
            )

        time.sleep(poll_s)


# ─────────────────────────────────────────────────────────────────────────────
# Mode: Batch
# ─────────────────────────────────────────────────────────────────────────────
def mode_batch(batch_dir: Path,
               lat: float, lon: float, alt: float,
               yaw: float = 0.0):
    """Inject EXIF ke semua foto di folder dengan GPS yang sama."""
    files = sorted(batch_dir.glob("*.jpg")) + sorted(batch_dir.glob("*.jpeg"))
    log.info(f"Batch inject {len(files)} file di {batch_dir}")
    log.info(f"GPS: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m")

    ok_count = 0
    for i, f in enumerate(files):
        ok = inject_exif_to_file(f, lat, lon, alt, yaw_deg=yaw)
        if ok:
            ok_count += 1
            log.info(f"[{i+1}/{len(files)}] ✓ {f.name}")
        else:
            log.warning(f"[{i+1}/{len(files)}] ✗ {f.name}")

    log.info(f"Selesai: {ok_count}/{len(files)} berhasil")


# ─────────────────────────────────────────────────────────────────────────────
# Mode: Single File Manual
# ─────────────────────────────────────────────────────────────────────────────
def mode_single(filepath: Path, lat: float, lon: float, alt: float):
    log.info(f"Inject EXIF ke {filepath}")
    ok = inject_exif_to_file(filepath, lat, lon, alt)
    if ok:
        log.info("✓ Berhasil")
        result = read_exif_gps(filepath)
        log.info(f"  Verifikasi: {result}")
    else:
        log.error("✗ Gagal")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="NOVA EXIF Injector – inject GPS MAVLink ke foto JPEG"
    )
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--watch-dir",  type=str, help="Watch folder (real-time inject)")
    group.add_argument("--batch-dir",  type=str, help="Batch inject semua foto di folder")
    group.add_argument("--file",       type=str, help="Inject ke satu file saja")

    # GPS manual (untuk batch & single)
    p.add_argument("--lat", type=float, default=0.0)
    p.add_argument("--lon", type=float, default=0.0)
    p.add_argument("--alt", type=float, default=0.0, help="Altitude MSL (meter)")
    p.add_argument("--yaw", type=float, default=0.0, help="Heading (derajat)")

    # FC connection (untuk watch mode)
    p.add_argument("--fc-port", type=str, default=config.FC_SERIAL_PORT)
    p.add_argument("--fc-baud", type=int, default=config.FC_BAUD_RATE)
    p.add_argument("--no-fc",   action="store_true",
                   help="Watch mode tanpa FC (GPS dummy dari --lat/--lon/--alt)")
    return p.parse_args()


def main():
    args = parse_args()

    if args.watch_dir:
        watch_dir = Path(args.watch_dir)
        watch_dir.mkdir(parents=True, exist_ok=True)

        if args.no_fc:
            # GPS statis dari argumen
            class DummyGPS:
                def get(self):
                    return {
                        "lat": args.lat, "lon": args.lon,
                        "alt_msl": args.alt, "alt_agl": args.alt,
                        "roll": 0.0, "pitch": 0.0, "yaw": args.yaw,
                        "sats": 0, "hdop": 99.0, "fix": 0, "valid": False
                    }
            live_gps = DummyGPS()
        else:
            conn_str = (f"{args.fc_port},{args.fc_baud}"
                        if "udp" not in args.fc_port.lower() else args.fc_port)
            live_gps = LiveGPS(conn_str)
            live_gps.start()
            log.info("Menunggu GPS fix (5 detik) ...")
            time.sleep(5)

        try:
            mode_watch(watch_dir, live_gps)
        except KeyboardInterrupt:
            log.info("Dihentikan.")

    elif args.batch_dir:
        mode_batch(Path(args.batch_dir), args.lat, args.lon, args.alt, args.yaw)

    elif args.file:
        mode_single(Path(args.file), args.lat, args.lon, args.alt)


if __name__ == "__main__":
    main()
