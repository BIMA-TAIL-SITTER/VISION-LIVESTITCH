#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-SENDER | receiver.py
=============================================================================
Dijalankan di: Laptop / Ground Control Station (GCS)

Fungsi:
  1. Menerima fragment UDP dari sender.py (UAV)
  2. Mereassemble fragment menjadi frame JPEG utuh
  3. Menyimpan frame ke folder sesi
  4. Menerima paket Object Detection (UDP JSON) dan menyimpannya
  5. Memberitahu stitcher.py via shared queue / file bahwa gambar baru tersedia

Cara Jalankan:
  python3 receiver.py --session flight_1
  python3 receiver.py --session flight_1 --img-port 5600 --od-port 5601
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
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))
import config

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [RECEIVER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("receiver")


# =============================================================================
# Frame Reassembler
# =============================================================================
class FrameReassembler:
    """
    Mereassemble fragment UDP menjadi frame JPEG lengkap.

    Format header setiap paket:
    [4B frame_id][4B total_chunks][4B chunk_idx][data...]
    """

    HEADER_FMT  = "!III"
    HEADER_SIZE = struct.calcsize(HEADER_FMT)

    def __init__(self, timeout_s: float = 5.0):
        self.timeout_s = timeout_s
        # {frame_id: {"chunks": {idx: bytes}, "total": int, "first_seen": float}}
        self._buffer: dict = defaultdict(lambda: {"chunks": {}, "total": 0, "first_seen": time.time()})
        self._lock   = threading.Lock()

    def feed(self, packet: bytes):
        """
        Masukkan paket UDP ke assembler.
        Return bytes frame jika lengkap, atau None jika belum.
        """
        if len(packet) < self.HEADER_SIZE:
            return None

        frame_id, total_chunks, chunk_idx = struct.unpack(
            self.HEADER_FMT, packet[:self.HEADER_SIZE]
        )
        payload = packet[self.HEADER_SIZE:]

        with self._lock:
            entry = self._buffer[frame_id]
            entry["total"] = total_chunks
            entry["chunks"][chunk_idx] = payload
            if not entry["first_seen"]:
                entry["first_seen"] = time.time()

            # Cek apakah semua chunk sudah terkumpul
            if len(entry["chunks"]) == total_chunks:
                # Reassemble in order
                assembled = b"".join(
                    entry["chunks"][i] for i in range(total_chunks)
                )
                del self._buffer[frame_id]
                return assembled

        return None

    def cleanup_stale(self):
        """Hapus frame yang sudah timeout (tidak lengkap)."""
        now = time.time()
        with self._lock:
            stale = [
                fid for fid, entry in self._buffer.items()
                if now - entry.get("first_seen", now) > self.timeout_s
            ]
            for fid in stale:
                log.warning(f"Frame {fid} timeout – dihapus (tidak lengkap)")
                del self._buffer[fid]


# =============================================================================
# Image Receiver Thread
# =============================================================================
class ImageReceiverThread(threading.Thread):
    """
    Thread yang mendengarkan UDP untuk menerima frame dari UAV.
    Frame lengkap disimpan ke folder sesi.
    """

    def __init__(self, port: int, save_dir: Path, od_store: "ODStore"):
        super().__init__(daemon=True)
        self.port       = port
        self.save_dir   = save_dir
        self.od_store   = od_store
        self._assembler = FrameReassembler()
        self._sock      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)  # 4 MB buffer

        self.img_count   = 0
        self.total_bytes = 0
        self._running    = True

    def run(self):
        self._sock.bind(("0.0.0.0", self.port))
        log.info(f"Image Receiver mendengarkan UDP di port {self.port} ...")

        cleanup_timer = time.time()

        while self._running:
            try:
                self._sock.settimeout(1.0)
                packet, addr = self._sock.recvfrom(config.UDP_MAX_PACKET)

                frame_bytes = self._assembler.feed(packet)
                if frame_bytes is not None:
                    self._save_frame(frame_bytes)

                # Cleanup stale setiap 10 detik
                if time.time() - cleanup_timer > 10:
                    self._assembler.cleanup_stale()
                    cleanup_timer = time.time()

            except socket.timeout:
                continue
            except Exception as e:
                log.error(f"Image receive error: {e}")

    def _save_frame(self, jpeg_bytes: bytes):
        """Simpan frame JPEG ke disk."""
        ts    = time.strftime("%Y%m%d_%H%M%S")
        ms    = int((time.time() % 1) * 1000)
        fname = f"{ts}_{ms:03d}_{self.img_count:05d}.jpg"
        path  = self.save_dir / fname

        with open(path, "wb") as f:
            f.write(jpeg_bytes)

        self.img_count   += 1
        self.total_bytes += len(jpeg_bytes)
        log.info(
            f"✓ Frame #{self.img_count} disimpan: {fname} "
            f"({len(jpeg_bytes)/1024:.1f} KB)"
        )

    def stop(self):
        self._running = False
        self._sock.close()


# =============================================================================
# Object Detection Store
# =============================================================================
class ODStore:
    """
    Menerima dan menyimpan paket object detection dari UAV.
    Paket disimpan sebagai JSONL agar stitcher bisa membacanya.

    Format JSON paket OD yang diharapkan:
    {
        "timestamp": 1234567890.123,
        "latitude":  -7.123456,
        "longitude": 112.654321,
        "altitude":  80.5,
        "detections": [
            {
                "label":      "person",
                "confidence": 0.92,
                "bbox":       [x1, y1, x2, y2],   // piksel di frame asli
                "geo":        {"lat": -7.123, "lon": 112.654, "alt": 80.5}
            }
        ]
    }
    """

    def __init__(self, save_dir: Path):
        self.save_dir    = save_dir
        self._jsonl_path = save_dir / "detections.jsonl"
        self._lock       = threading.Lock()
        self._records    = []

    def add(self, packet: dict):
        with self._lock:
            self._records.append(packet)
            # Append ke JSONL file
            with open(self._jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(packet) + "\n")
        log.info(
            f"🎯 OD diterima: {len(packet.get('detections', []))} objek "
            f"@ lat={packet.get('latitude', '?'):.6f} "
            f"lon={packet.get('longitude', '?'):.6f}"
        )

    def get_all(self) -> list:
        with self._lock:
            return list(self._records)


# =============================================================================
# OD Receiver Thread
# =============================================================================
class ODReceiverThread(threading.Thread):
    """
    Thread yang mendengarkan UDP untuk paket object detection JSON dari UAV.
    """

    def __init__(self, port: int, od_store: ODStore):
        super().__init__(daemon=True)
        self.port     = port
        self.od_store = od_store
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = True
        self.pkt_count = 0

    def run(self):
        self._sock.bind(("0.0.0.0", self.port))
        log.info(f"OD Receiver mendengarkan UDP di port {self.port} ...")

        while self._running:
            try:
                self._sock.settimeout(1.0)
                data, addr = self._sock.recvfrom(65507)
                packet = json.loads(data.decode("utf-8"))
                self.od_store.add(packet)
                self.pkt_count += 1
            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                log.warning(f"JSON parse error: {e}")
            except Exception as e:
                log.error(f"OD receive error: {e}")

    def stop(self):
        self._running = False
        self._sock.close()


# =============================================================================
# Status Printer
# =============================================================================
def status_printer(img_receiver: ImageReceiverThread,
                   od_receiver: ODReceiverThread,
                   interval: float = 10.0):
    """Print statistik penerimaan setiap N detik."""
    while True:
        time.sleep(interval)
        log.info(
            f"STATUS | Gambar diterima: {img_receiver.img_count} | "
            f"Data OD: {od_receiver.pkt_count} paket | "
            f"Total data: {img_receiver.total_bytes / 1024 / 1024:.2f} MB"
        )


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="Receiver – Ground station image & OD receiver"
    )
    parser.add_argument("--session",  type=str, default=config.DEFAULT_SESSION_ID,
                        help="ID sesi (nama folder penyimpanan)")
    parser.add_argument("--img-port", type=int, default=config.IMAGE_PORT,
                        help="UDP port gambar dari UAV")
    parser.add_argument("--od-port",  type=int, default=config.OD_PORT,
                        help="UDP port object detection dari UAV")
    parser.add_argument("--base-dir", type=str, default=config.SESSION_DIR,
                        help="Direktori root sesi")
    return parser.parse_args()


def main():
    args = parse_args()

    log.info("=" * 60)
    log.info("PROGRAM-SENDER Receiver v1.0")
    log.info(f"  Sesi        : {args.session}")
    log.info(f"  Image port  : {args.img_port}")
    log.info(f"  OD port     : {args.od_port}")
    log.info("=" * 60)

    # Buat folder sesi
    session_root = Path(args.base_dir) / args.session
    img_dir      = session_root / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    log.info(f"Menyimpan gambar ke: {img_dir}")

    # Inisialisasi komponen
    od_store     = ODStore(session_root)
    img_receiver = ImageReceiverThread(args.img_port, img_dir, od_store)
    od_receiver  = ODReceiverThread(args.od_port, od_store)

    # Mulai thread
    img_receiver.start()
    od_receiver.start()

    # Status printer (background)
    stat_thread = threading.Thread(
        target=status_printer,
        args=(img_receiver, od_receiver, 15.0),
        daemon=True
    )
    stat_thread.start()

    log.info("Receiver berjalan. Tekan Ctrl+C untuk berhenti.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("\nMenghentikan receiver ...")
    finally:
        img_receiver.stop()
        od_receiver.stop()
        log.info(
            f"Selesai. Total: {img_receiver.img_count} gambar diterima, "
            f"{od_receiver.pkt_count} paket OD diterima."
        )


if __name__ == "__main__":
    main()
