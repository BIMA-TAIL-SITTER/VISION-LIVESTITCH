#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-NOVA | run_ground.py
=============================================================================
Script launcher untuk menjalankan receiver + stitcher + OD simulator
sekaligus di ground station dengan satu perintah.

Cara Jalankan:
  python3 run_ground.py --session nova_flight_1
  python3 run_ground.py --session test1 --simulate-od   # dengan OD simulator
=============================================================================
"""

import subprocess
import argparse
import sys
import os
import time
import signal
import logging

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [LAUNCHER] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("launcher")

NOVA_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON   = sys.executable


def parse_args():
    parser = argparse.ArgumentParser(
        description="NOVA Ground Launcher – jalankan receiver + stitcher sekaligus"
    )
    parser.add_argument("--session",     type=str,  default="nova_session",
                        help="ID sesi")
    parser.add_argument("--img-port",    type=int,  default=5600)
    parser.add_argument("--od-port",     type=int,  default=5601)
    parser.add_argument("--batch",       type=int,  default=5,
                        help="Batch size stitching")
    parser.add_argument("--gps-thresh",  type=float, default=3.0,
                        help="GPS threshold (meter)")
    parser.add_argument("--simulate-od", action="store_true",
                        help="Jalankan OD simulator lokal (untuk testing)")
    parser.add_argument("--od-host",     type=str,  default="127.0.0.1",
                        help="Host OD simulator (hanya jika --simulate-od)")
    return parser.parse_args()


def main():
    args     = parse_args()
    procs    = []

    log.info("=" * 60)
    log.info("PROGRAM-NOVA Ground Station Launcher")
    log.info(f"  Sesi     : {args.session}")
    log.info(f"  Batch    : {args.batch} gambar")
    log.info(f"  GPS thr. : {args.gps_thresh} m")
    log.info("=" * 60)

    try:
        # 1. Receiver
        log.info("▶ Memulai receiver.py ...")
        p_recv = subprocess.Popen([
            PYTHON, os.path.join(NOVA_DIR, "receiver.py"),
            "--session",  args.session,
            "--img-port", str(args.img_port),
            "--od-port",  str(args.od_port),
        ])
        procs.append(("receiver", p_recv))
        time.sleep(1)

        # 2. Stitcher
        log.info("▶ Memulai stitcher.py ...")
        p_stitch = subprocess.Popen([
            PYTHON, os.path.join(NOVA_DIR, "stitcher.py"),
            "--session",       args.session,
            "--batch",         str(args.batch),
            "--gps-threshold", str(args.gps_thresh),
        ])
        procs.append(("stitcher", p_stitch))
        time.sleep(0.5)

        # 3. OD Simulator (opsional)
        if args.simulate_od:
            log.info("▶ Memulai od_simulator.py (mode simulasi lokal) ...")
            p_od = subprocess.Popen([
                PYTHON, os.path.join(NOVA_DIR, "od_simulator.py"),
                "--host",     args.od_host,
                "--od-port",  str(args.od_port),
                "--interval", "10",
            ])
            procs.append(("od_simulator", p_od))

        log.info("")
        log.info("✓ Semua proses berjalan. Tekan Ctrl+C untuk menghentikan semua.")
        log.info(f"  Output mosaic: sessions/{args.session}/output/")
        log.info("")

        # Monitor proses
        while True:
            for name, p in procs:
                if p.poll() is not None:
                    log.warning(f"Proses '{name}' berhenti dengan kode {p.returncode}")
            time.sleep(5)

    except KeyboardInterrupt:
        log.info("\nMenghentikan semua proses ...")
    finally:
        for name, p in procs:
            try:
                p.terminate()
                p.wait(timeout=3)
                log.info(f"  ✓ {name} dihentikan")
            except Exception:
                p.kill()
        log.info("Selesai.")


if __name__ == "__main__":
    main()
