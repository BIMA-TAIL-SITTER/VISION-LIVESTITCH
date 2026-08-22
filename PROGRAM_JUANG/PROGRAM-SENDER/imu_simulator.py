#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-SENDER | imu_simulator.py
=============================================================================
Dijalankan di: Laptop (untuk testing sender.py / imu_monitor.py tanpa FC asli)

Fungsi:
  Berperan sebagai Flight Controller PALSU. Membuka koneksi MAVLink via UDP
  dan mengirimkan pesan HEARTBEAT + ATTITUDE + GLOBAL_POSITION_INT + GPS_RAW_INT
  beneran (bukan data dummy JSON), persis seperti FC asli. sender.py dan
  imu_monitor.py bisa connect ke sini TANPA perlu diubah kodenya sama sekali —
  karena keduanya sudah pakai pymavlink yang mendukung koneksi serial maupun UDP.
  Termasuk simulasi GPS bergerak lurus (garis survey), jadi bisa dipakai buat
  ngetes fitur embed GPS ke EXIF di sender.py juga.

Mode simulasi:
  --mode sine      Roll & pitch berosilasi halus (sine wave) — bagus buat
                    lihat gate nyala-mati berulang.
  --mode scenario  Simulasi misi mapping nyata: terbang lurus (stabil) →
                    belok tajam (roll naik lewati threshold) → lurus lagi →
                    berulang. Ini paling mirip kondisi asli fixed-wing pas
                    survey (lurus - belok di ujung jalur - lurus lagi).

Cara Jalankan:
  # 1. Jalankan simulator (di satu terminal)
  python3 imu_simulator.py
  python3 imu_simulator.py --mode scenario --max-roll 60

  # 2. Jalankan sender.py atau imu_monitor.py di terminal lain, suruh dia LISTEN
  #    di port yang sama (simulator yang aktif ngirim, bukan sebaliknya):
  python3 sender.py --fc-port udpin:0.0.0.0:14550 --host 127.0.0.1
  python3 imu_monitor.py --port udpin:0.0.0.0:14550 --no-send
=============================================================================
"""

import time
import math
import argparse
import logging
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
import config

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [IMU-SIM] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("imu_sim")

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    log.error("pymavlink tidak tersedia! Install: pip install pymavlink")


# =============================================================================
# Generator Attitude
# =============================================================================
class SineAttitude:
    """Roll & pitch berosilasi sinusoidal terus-menerus."""

    def __init__(self, max_roll: float, max_pitch: float, period: float):
        self.max_roll  = max_roll
        self.max_pitch = max_pitch
        self.period    = period
        self.t0        = time.time()

    def get(self):
        t = time.time() - self.t0
        roll  = self.max_roll  * math.sin(2 * math.pi * t / self.period)
        pitch = self.max_pitch * math.sin(2 * math.pi * t / (self.period * 1.7))
        yaw   = (t * 10) % 360
        return roll, pitch, yaw


class ScenarioAttitude:
    """
    Simulasi pola terbang fixed-wing survey:
      LURUS (roll~0) -> BELOK (roll naik ke max_roll & turun lagi) -> LURUS -> ulang
    Cocok buat ngetes IMU gate: capture harus jalan pas LURUS, dan tertahan pas BELOK.
    """

    def __init__(self, max_roll: float, straight_dur: float, turn_dur: float):
        self.max_roll     = max_roll
        self.straight_dur = straight_dur
        self.turn_dur     = turn_dur
        self.cycle        = straight_dur + turn_dur
        self.t0           = time.time()

    def get(self):
        t = (time.time() - self.t0) % self.cycle
        pitch = 2.0 * math.sin(2 * math.pi * (time.time() - self.t0) / 8.0)  # goyangan kecil

        if t < self.straight_dur:
            roll = 0.0
            phase = "LURUS"
        else:
            # ramp naik -> tahan -> ramp turun selama fase belok
            tt = t - self.straight_dur
            half = self.turn_dur / 2
            if tt < half:
                roll = self.max_roll * (tt / half)
            else:
                roll = self.max_roll * (1 - (tt - half) / half)
            phase = "BELOK"

        yaw = (t * 15) % 360
        return roll, pitch, yaw, phase


# =============================================================================
# Generator GPS – simulasi posisi bergerak lurus (garis survey)
# =============================================================================
class GPSTrack:
    """
    Simulasi posisi GPS bergerak lurus dari titik awal dengan heading & speed
    tertentu (meniru satu lintasan survey). Altitude relatif dibuat konstan
    (bisa digoyang dikit biar realistis).
    """

    EARTH_R = 6378137.0  # meter

    def __init__(self, start_lat: float, start_lon: float, start_alt_rel: float,
                 heading_deg: float, speed_mps: float, no_fix: bool = False):
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.alt_rel   = start_alt_rel
        self.heading   = math.radians(heading_deg)
        self.speed     = speed_mps
        self.no_fix    = no_fix
        self.t0        = time.time()

    def get(self):
        t = time.time() - self.t0
        dist = self.speed * t  # meter yang sudah ditempuh

        dlat = (dist * math.cos(self.heading)) / self.EARTH_R
        dlon = (dist * math.sin(self.heading)) / (self.EARTH_R * math.cos(math.radians(self.start_lat)))

        lat = self.start_lat + math.degrees(dlat)
        lon = self.start_lon + math.degrees(dlon)
        alt_rel = self.alt_rel + 1.5 * math.sin(2 * math.pi * t / 20.0)  # goyangan altitude kecil
        alt_msl = alt_rel + 100.0  # asumsi elevasi tanah 100m MSL, cuma buat simulasi

        if self.no_fix:
            fix_type, satellites = 0, 2
        else:
            fix_type, satellites = 3, 14

        return lat, lon, alt_rel, alt_msl, fix_type, satellites


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="IMU Simulator – FC palsu via MAVLink UDP untuk testing"
    )
    parser.add_argument("--port", type=int, default=config.SIM_MAVLINK_PORT,
                        help="Port UDP tujuan (sender.py/imu_monitor.py harus listen di sini, default dari config.py)")
    parser.add_argument("--target-host", type=str, default="127.0.0.1",
                        help="Host tujuan tempat sender.py/imu_monitor.py listen (udpin)")
    parser.add_argument("--mode", type=str, choices=["sine", "scenario"], default="scenario",
                        help="Pola simulasi attitude")
    parser.add_argument("--max-roll",  type=float, default=60.0,
                        help="Roll maksimum saat belok/osilasi (derajat)")
    parser.add_argument("--max-pitch", type=float, default=15.0,
                        help="Pitch maksimum saat osilasi, khusus --mode sine (derajat)")
    parser.add_argument("--straight-dur", type=float, default=8.0,
                        help="[scenario] Lama fase lurus (detik)")
    parser.add_argument("--turn-dur",     type=float, default=4.0,
                        help="[scenario] Lama fase belok, naik+turun (detik)")
    parser.add_argument("--period", type=float, default=6.0,
                        help="[sine] Periode osilasi roll (detik)")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Rate kirim ATTITUDE (Hz)")
    parser.add_argument("--start-lat", type=float, default=-7.123456,
                        help="Titik GPS awal - latitude")
    parser.add_argument("--start-lon", type=float, default=112.654321,
                        help="Titik GPS awal - longitude")
    parser.add_argument("--start-alt", type=float, default=80.0,
                        help="Altitude relatif (AGL, meter) awal")
    parser.add_argument("--heading", type=float, default=90.0,
                        help="Arah terbang simulasi (derajat, 0=utara, 90=timur)")
    parser.add_argument("--speed", type=float, default=15.0,
                        help="Kecepatan simulasi (m/s)")
    parser.add_argument("--no-gps-fix", action="store_true",
                        help="Simulasikan GPS BELUM fix (buat testing --require-gps-fix di sender.py)")
    return parser.parse_args()


def main():
    if not MAVLINK_AVAILABLE:
        log.error("pymavlink tidak terinstall. Jalankan: pip install pymavlink")
        return

    args = parse_args()

    log.info("=" * 60)
    log.info("IMU Simulator (FC palsu via MAVLink UDP)")
    log.info(f"  Kirim ke      : udp {args.target_host}:{args.port}")
    log.info(f"  Mode          : {args.mode}")
    if args.mode == "sine":
        log.info(f"  Max roll/pitch: ±{args.max_roll}° / ±{args.max_pitch}° (periode {args.period}s)")
    else:
        log.info(f"  Max roll      : ±{args.max_roll}° (lurus {args.straight_dur}s, belok {args.turn_dur}s)")
    log.info(f"  Jalankan sender.py / imu_monitor.py dengan LISTEN di port ini:")
    log.info(f"    --fc-port udpin:0.0.0.0:{args.port}   (atau --port untuk imu_monitor.py)")
    log.info(f"  GPS awal      : {args.start_lat:.6f}, {args.start_lon:.6f} @ {args.start_alt}m AGL")
    log.info(f"  GPS gerak     : heading {args.heading}°, speed {args.speed} m/s"
              f"{'  [NO FIX]' if args.no_gps_fix else ''}")
    log.info("=" * 60)

    mav = mavutil.mavlink_connection(f"udpout:{args.target_host}:{args.port}", source_system=1)
    log.info("Mulai streaming HEARTBEAT + ATTITUDE + GPS palsu (Ctrl+C untuk berhenti)...")

    if args.mode == "sine":
        gen = SineAttitude(args.max_roll, args.max_pitch, args.period)
    else:
        gen = ScenarioAttitude(args.max_roll, args.straight_dur, args.turn_dur)

    gps_gen = GPSTrack(args.start_lat, args.start_lon, args.start_alt,
                        args.heading, args.speed, no_fix=args.no_gps_fix)

    interval = 1.0 / args.rate
    last_hb   = 0.0
    last_gps  = 0.0
    prev_phase = None

    try:
        while True:
            now = time.time()

            # Heartbeat tiap 1 detik (wajib biar klien anggap FC masih hidup)
            if now - last_hb >= 1.0:
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                last_hb = now

            if args.mode == "scenario":
                roll, pitch, yaw, phase = gen.get()
                if phase != prev_phase:
                    log.info(f"→ Fase: {phase}  (roll target puncak ±{args.max_roll}°)")
                    prev_phase = phase
            else:
                roll, pitch, yaw = gen.get()

            mav.mav.attitude_send(
                int(now * 1000) & 0xFFFFFFFF,
                math.radians(roll), math.radians(pitch), math.radians(yaw),
                0.0, 0.0, 0.0
            )

            # GPS dikirim @ 5Hz (GLOBAL_POSITION_INT + GPS_RAW_INT)
            if now - last_gps >= 0.2:
                lat, lon, alt_rel, alt_msl, fix_type, satellites = gps_gen.get()

                mav.mav.global_position_int_send(
                    int(now * 1000) & 0xFFFFFFFF,
                    int(lat * 1e7), int(lon * 1e7),
                    int(alt_msl * 1000), int(alt_rel * 1000),
                    0, 0, 0,  # vx, vy, vz (cm/s) - tidak disimulasikan
                    int(yaw * 100) if yaw <= 360 else 0
                )
                mav.mav.gps_raw_int_send(
                    int(now * 1e6) & 0xFFFFFFFFFFFFFFFF,
                    fix_type,
                    int(lat * 1e7), int(lon * 1e7), int(alt_msl * 1000),
                    65535, 65535,  # eph, epv (unknown)
                    0,             # vel
                    65535,         # cog (unknown)
                    satellites
                )
                last_gps = now

                gps_str = f"GPS={'FIX' if fix_type >= 3 else 'NO-FIX'} lat={lat:.6f} lon={lon:.6f} alt={alt_rel:.1f}m"
            else:
                lat = lon = alt_rel = 0.0
                gps_str = ""

            print(f"\r[IMU-SIM] Roll={roll:6.1f}°  Pitch={pitch:6.1f}°  Yaw={yaw:6.1f}°  |  {gps_str}",
                  end="", flush=True)
            time.sleep(interval)

    except KeyboardInterrupt:
        print()
        log.info("Simulator dihentikan.")


if __name__ == "__main__":
    main()