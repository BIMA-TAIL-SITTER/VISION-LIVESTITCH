#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-SENDER | fc_router.py
=============================================================================
Dijalankan di: Perangkat yang fisik terhubung ke FC (Raspberry Pi via UART,
               atau laptop via USB/ELRS COM port)

Fungsi:
  Membaca SATU koneksi serial fisik ke Flight Controller lalu meneruskan
  (relay) semua byte MAVLink mentah ke BEBERAPA endpoint UDP sekaligus.

  Masalah yang diselesaikan: serial/COM port cuma bisa dibuka oleh SATU
  proses dalam satu waktu (PermissionError: Access is denied kalau dua
  program coba buka port yang sama). Dengan fc_router.py, hanya router INI
  yang pegang port serial-nya; sender.py, imu_monitor.py, Mission Planner,
  dst tinggal dengarkan (listen) di port UDP masing-masing — bisa jalan
  BERSAMAAN tanpa rebutan.

  Pola koneksinya sama seperti imu_simulator.py: router = udpout (aktif
  push data), konsumer (sender.py/imu_monitor.py) = udpin (listen).

Cara Jalankan:
  # Router baca dari FC fisik, forward ke 2 listener sekaligus
  python3 fc_router.py --port COM4 --baud 460800 \
      --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

  # Di terminal lain — masing-masing consumer listen di port berbeda:
  python3 sender.py --fc-port udpin:0.0.0.0:14552 --host 127.0.0.1
  python3 imu_monitor.py --port udpin:0.0.0.0:14553 --no-send

  # Bisa juga forward ke Mission Planner/QGroundControl di port standar 14550
  python3 fc_router.py --port /dev/ttyAMA0 --baud 57600 \
      --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553 --out udp:192.168.1.50:14550
=============================================================================
"""

import argparse
import logging
import socket
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [FC-ROUTER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("fc_router")

try:
    import serial
    PYSERIAL_AVAILABLE = True
except ImportError:
    PYSERIAL_AVAILABLE = False
    log.error("pyserial tidak tersedia! Install: pip install pyserial")


def parse_out_target(spec: str):
    """Parse 'udp:HOST:PORT' (prefix 'udp:' opsional) -> (host, port)."""
    s = spec.strip()
    if s.lower().startswith("udp:"):
        s = s[4:]
    try:
        host, port_str = s.rsplit(":", 1)
        return host, int(port_str)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Format --out tidak valid: '{spec}'. Gunakan udp:HOST:PORT, contoh udp:127.0.0.1:14552"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="FC Router - relay 1 koneksi serial FC ke banyak listener UDP sekaligus"
    )
    parser.add_argument("--port", type=str, required=True,
                        help="Serial port fisik ke FC (contoh: COM4, /dev/ttyAMA0, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=460800,
                        help="Baud rate serial ke FC (default 460800; ELRS/USB umumnya 115200-460800, UART Raspi 57600)")
    parser.add_argument("--out", action="append", required=True, metavar="udp:HOST:PORT",
                        help="Target UDP output, format udp:HOST:PORT. Ulangi --out untuk beberapa target sekaligus.")
    parser.add_argument("--chunk-size", type=int, default=1024,
                        help="Ukuran maksimum baca per siklus dari serial (bytes)")
    return parser.parse_args()


def main():
    if not PYSERIAL_AVAILABLE:
        sys.exit(1)

    args = parse_args()
    targets = [parse_out_target(o) for o in args.out]

    log.info("=" * 60)
    log.info("FC Router — MAVLink UDP fan-out")
    log.info(f"  Serial in : {args.port} @ {args.baud} baud")
    for host, port in targets:
        log.info(f"  Out       : udp:{host}:{port}")
    log.info("=" * 60)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        log.error(f"Gagal membuka serial {args.port}: {e}")
        log.error("Tips: pastikan tidak ada program lain (Mission Planner/QGC/router lain) yang masih memegang port ini.")
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    log.info("Router aktif, meneruskan data FC ke semua target ... (Ctrl+C untuk berhenti)")
    byte_count   = 0
    packet_count = 0
    t_last_log   = time.time()

    try:
        while True:
            data = ser.read(args.chunk_size)
            if data:
                for host, port in targets:
                    try:
                        sock.sendto(data, (host, port))
                    except OSError as e:
                        log.warning(f"Gagal kirim ke udp:{host}:{port}: {e}")
                byte_count   += len(data)
                packet_count += 1

            now = time.time()
            if now - t_last_log >= 5.0:
                log.info(f"Relay aktif: {byte_count} bytes / {packet_count} chunk diteruskan (5 detik terakhir)")
                byte_count   = 0
                packet_count = 0
                t_last_log   = now

    except KeyboardInterrupt:
        print()
        log.info("Router dihentikan oleh pengguna.")
    except serial.SerialException as e:
        log.error(f"Serial error: {e} — FC mungkin terputus/dicabut.")
    finally:
        ser.close()
        sock.close()
        log.info("Selesai.")


if __name__ == "__main__":
    main()
