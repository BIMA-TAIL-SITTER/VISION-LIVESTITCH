#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-NOVA | od_simulator.py
=============================================================================
Dijalankan di: Laptop / GCS (untuk simulasi) atau Raspberry Pi (untuk real OD)

Fungsi:
  Mensimulasikan Object Detection yang berjalan di pesawat.
  Setiap 10 detik mengirimkan paket JSON berisi:
    - Posisi GPS pesawat saat ini (latitude, longitude, altitude)
    - Daftar objek terdeteksi dengan bounding box dan koordinat geo

  Paket dikirim via UDP ke receiver.py di ground.

Cara Jalankan (simulasi di laptop):
  python3 od_simulator.py --host 127.0.0.1 --od-port 5601

Cara Jalankan (dari Raspi ke ground):
  python3 od_simulator.py --host 192.168.1.100 --od-port 5601
=============================================================================
"""

import socket
import json
import time
import random
import math
import argparse
import logging
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
import config

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [OD-SIM] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("od_sim")

# =============================================================================
# Label yang disimulasikan
# =============================================================================
OBJECT_LABELS = ["person", "vehicle", "bicycle", "animal", "debris"]

# Waypoint rute simulasi (Fixed Wing – lintasan lurus)
# Ganti dengan koordinat area operasi nyata
SIMULATED_WAYPOINTS = [
    {"lat": -7.123456, "lon": 112.654321, "alt": 80.0},
    {"lat": -7.124000, "lon": 112.655000, "alt": 80.0},
    {"lat": -7.124800, "lon": 112.655800, "alt": 82.0},
    {"lat": -7.125500, "lon": 112.656500, "alt": 82.0},
    {"lat": -7.126200, "lon": 112.657200, "alt": 80.0},
    {"lat": -7.126900, "lon": 112.657900, "alt": 80.0},
    {"lat": -7.127600, "lon": 112.658600, "alt": 78.0},
    {"lat": -7.128300, "lon": 112.659300, "alt": 78.0},
]


def interpolate_position(waypoints: list, t: float) -> dict:
    """
    Interpolasi posisi pesawat berdasarkan waktu t (0.0 - 1.0).
    Simulasikan pesawat bergerak melalui waypoints.
    """
    n = len(waypoints) - 1
    seg_len = 1.0 / n
    seg_idx = min(int(t / seg_len), n - 1)
    seg_t   = (t - seg_idx * seg_len) / seg_len

    wp0 = waypoints[seg_idx]
    wp1 = waypoints[seg_idx + 1]

    lat = wp0["lat"] + (wp1["lat"] - wp0["lat"]) * seg_t
    lon = wp0["lon"] + (wp1["lon"] - wp0["lon"]) * seg_t
    alt = wp0["alt"] + (wp1["alt"] - wp0["alt"]) * seg_t
    return {"lat": lat, "lon": lon, "alt": alt}


def generate_detections(uav_lat: float, uav_lon: float, uav_alt: float,
                        frame_w: int = 1920, frame_h: int = 1080,
                        max_objects: int = config.OD_NUM_OBJECTS_MAX) -> list:
    """
    Generate deteksi objek palsu di sekitar posisi UAV.
    Setiap objek punya bbox di frame dan koordinat geo.

    Konversi posisi piksel → geo (estimasi naif):
      Footprint kamera ≈ 2 * alt * tan(FOV/2)
      Untuk Osmo Action 5 Pro: FOV ≈ 122° (wide)
    """
    fov_deg     = 122.0
    fov_rad     = math.radians(fov_deg)
    footprint_m = 2 * uav_alt * math.tan(fov_rad / 2)

    # meter per piksel
    mpp_x = footprint_m / frame_w
    mpp_y = footprint_m / frame_h

    detections = []
    n_objects = random.randint(0, max_objects)

    for _ in range(n_objects):
        # Random bbox dalam frame
        bx1 = random.randint(0, frame_w - 100)
        by1 = random.randint(0, frame_h - 100)
        bx2 = bx1 + random.randint(40, 150)
        by2 = by1 + random.randint(40, 150)

        # Pusat bbox dalam piksel (relatif ke pusat frame)
        cx_px = ((bx1 + bx2) / 2) - frame_w / 2
        cy_px = ((by1 + by2) / 2) - frame_h / 2

        # Offset dalam meter
        dx_m = cx_px * mpp_x
        dy_m = -cy_px * mpp_y   # Y terbalik (piksel turun = selatan)

        # Offset ke koordinat geo
        det_lat = uav_lat + dy_m / 110540
        det_lon = uav_lon + dx_m / (111320 * math.cos(math.radians(uav_lat)))

        label = random.choice(OBJECT_LABELS)
        conf  = round(random.uniform(0.65, 0.98), 3)

        detections.append({
            "label":      label,
            "confidence": conf,
            "bbox":       [bx1, by1, bx2, by2],
            "geo": {
                "lat": round(det_lat, 7),
                "lon": round(det_lon, 7),
                "alt": round(uav_alt, 2)
            }
        })

    return detections


def build_packet(uav_pos: dict, detections: list) -> dict:
    """Buat paket JSON lengkap untuk dikirim ke ground."""
    return {
        "timestamp":  time.time(),
        "source":     "od_simulator",
        "latitude":   round(uav_pos["lat"], 7),
        "longitude":  round(uav_pos["lon"], 7),
        "altitude":   round(uav_pos["alt"], 2),
        "detections": detections
    }


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="NOVA OD Simulator – kirim deteksi palsu ke ground"
    )
    parser.add_argument("--host",     type=str, default=config.GROUND_IP,
                        help="IP ground station")
    parser.add_argument("--od-port",  type=int, default=config.OD_PORT,
                        help="UDP port OD receiver di ground")
    parser.add_argument("--interval", type=float, default=config.OD_SEND_INTERVAL,
                        help="Interval kirim paket OD (detik)")
    parser.add_argument("--duration", type=float, default=120.0,
                        help="Durasi simulasi (detik). 0 = selamanya")
    return parser.parse_args()


def main():
    args = parse_args()

    log.info("=" * 60)
    log.info("PROGRAM-NOVA OD Simulator v1.0")
    log.info(f"  Target         : {args.host}:{args.od_port}")
    log.info(f"  Interval       : {args.interval}s")
    log.info(f"  Durasi         : {'∞' if args.duration == 0 else f'{args.duration}s'}")
    log.info(f"  Waypoints      : {len(SIMULATED_WAYPOINTS)} titik")
    log.info("=" * 60)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    start_time = time.time()
    pkt_count  = 0
    total_dur  = args.duration if args.duration > 0 else float("inf")
    loop_dur   = (len(SIMULATED_WAYPOINTS) - 1) * 30  # 30 detik per segmen

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed >= total_dur:
                log.info("Durasi simulasi selesai.")
                break

            # Posisi UAV saat ini (loop kembali ke awal jika habis)
            t_norm = (elapsed % loop_dur) / loop_dur
            uav_pos = interpolate_position(SIMULATED_WAYPOINTS, t_norm)

            # Generate deteksi
            detections = generate_detections(
                uav_pos["lat"], uav_pos["lon"], uav_pos["alt"]
            )

            # Buat dan kirim paket
            packet      = build_packet(uav_pos, detections)
            packet_json = json.dumps(packet).encode("utf-8")

            try:
                sock.sendto(packet_json, (args.host, args.od_port))
                pkt_count += 1

                log.info(
                    f"📦 Paket #{pkt_count} terkirim | "
                    f"Lat={uav_pos['lat']:.6f} Lon={uav_pos['lon']:.6f} "
                    f"Alt={uav_pos['alt']:.1f}m | "
                    f"{len(detections)} objek: "
                    + ", ".join(
                        f"{d['label']}({d['confidence']:.0%})"
                        for d in detections
                    ) if detections else "0 objek"
                )

                # Print detail deteksi
                for i, det in enumerate(detections):
                    log.info(
                        f"  [{i+1}] {det['label']:10s} conf={det['confidence']:.0%} "
                        f"bbox={det['bbox']} "
                        f"@ lat={det['geo']['lat']:.6f} lon={det['geo']['lon']:.6f}"
                    )

            except Exception as e:
                log.error(f"Send error: {e}")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        log.info("\nDihentikan oleh pengguna.")
    finally:
        sock.close()
        log.info(f"Total paket terkirim: {pkt_count}")


if __name__ == "__main__":
    main()
