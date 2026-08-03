# sim_bbox_sender.py
# Simulasi Pengiriman Gambar, Metadata GPS, dan Bounding Box secara Berkala via Socket

import socket
import struct
import json
import os
import sys
import time
import argparse
import random
from pathlib import Path
import glob

# Ensure UTF-8 output on Windows terminal
if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass


def generate_simulated_metadata(frame_id, img_w, img_h):
    """
    Generate GPS telemetry dan bounding box acak / terdeteksi pada frame.
    """
    # Simulasi GPS Drone (Lintasan lurus kecil di ITS Surabaya)
    base_lat = -7.27581 + (frame_id * 0.00005)
    base_lon = 112.79832 + (frame_id * 0.00005)
    altitude = 50.0 + random.uniform(-0.5, 0.5)

    # Buat 1 atau 2 bounding box acak pada frame gambar
    bboxes = []
    num_objects = random.choice([1, 2])
    
    classes = ["car", "person", "drone_target", "boat"]
    for i in range(num_objects):
        bw = random.randint(60, 120)
        bh = random.randint(60, 120)
        x1 = random.randint(50, max(51, img_w - bw - 50))
        y1 = random.randint(50, max(51, img_h - bh - 50))
        x2 = x1 + bw
        y2 = y1 + bh

        target_lat = base_lat + random.uniform(-0.00002, 0.00002)
        target_lon = base_lon + random.uniform(-0.00002, 0.00002)

        bboxes.append({
            "id": i + 1,
            "label": random.choice(classes),
            "confidence": round(random.uniform(0.82, 0.98), 2),
            "bbox": [x1, y1, x2, y2],  # [x1, y1, x2, y2] pada frame lokal
            "lat": target_lat,
            "lon": target_lon
        })

    metadata = {
        "frame_id": frame_id,
        "timestamp": time.time(),
        "gps": {
            "latitude": base_lat,
            "longitude": base_lon,
            "altitude": altitude
        },
        "bboxes": bboxes
    }
    return metadata

def send_packet(sock, img_path, frame_id):
    """
    Kirim satu paket data: [4-byte header length] + [JSON metadata] + [JPEG bytes]
    """
    with open(img_path, "rb") as f:
        img_bytes = f.read()

    # Dapatkan dimensi gambar kasar dari OpenCV / estimasi
    import cv2
    import numpy as np
    nparr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    h, w = img.shape[:2]

    # Generate metadata
    meta = generate_simulated_metadata(frame_id, w, h)
    meta_json_str = json.dumps(meta)
    meta_bytes = meta_json_str.encode('utf-8')

    # Struktur Paket Protocol:
    # 4 byte: Meta JSON Length
    # 4 byte: Image Bytes Length
    # Payload 1: Meta JSON Bytes
    # Payload 2: Image Bytes
    header = struct.pack("!II", len(meta_bytes), len(img_bytes))
    sock.sendall(header + meta_bytes + img_bytes)
    return len(meta_bytes) + len(img_bytes), meta

def main():
    parser = argparse.ArgumentParser(description="Simulator Pengirim Bounding Box & Gambar live")
    parser.add_argument(
        '--dataset-dir', type=str,
        default=os.path.join(os.path.dirname(__file__), '..', 'dataset', '100GOPRO'),
        help='Folder dataset gambar'
    )
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Host receiver')
    parser.add_argument('--port', type=int, default=5050, help='Port receiver')
    parser.add_argument('--delay', type=float, default=0.5, help='Delay antar frame (detik)')
    parser.add_argument('--max-images', type=int, default=0, help='Maksimum gambar (0 = semua)')
    args = parser.parse_args()

    dataset_dir = Path(args.dataset_dir).resolve()
    if not dataset_dir.exists():
        print(f"[SENDER] Error: Directory {dataset_dir} tidak ditemukan.")
        return

    image_files = []
    for ext in ["*.jpg", "*.jpeg", "*.png", "*.JPG", "*.PNG"]:
        image_files.extend(glob.glob(str(dataset_dir / ext)))
    image_files = sorted(list(set(image_files)))

    if not image_files:
        print(f"[SENDER] tidak ada gambar di {dataset_dir}")
        return

    if args.max_images > 0:
        image_files = image_files[:args.max_images]

    print("=" * 60)
    print("  SIMULATOR SENDER BBOX & TELEMETRI GPS LIVE")
    print("=" * 60)
    print(f"  Target: {args.host}:{args.port}")
    print(f"  Jumlah Gambar: {len(image_files)}")
    print(f"  Delay Pengiriman: {args.delay}s")
    print("=" * 60)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((args.host, args.port))
        print("[SENDER] [OK] Terhubung ke Server Receiver!")
    except Exception as e:
        print(f"[SENDER] [FAIL] Gagal Terhubung: {e}")
        return

    try:
        for i, img_path in enumerate(image_files, 1):
            fname = os.path.basename(img_path)
            total_sent_bytes, meta = send_packet(sock, img_path, i)
            
            print(f"[SENDER] [{i}/{len(image_files)}] {fname} sent ({total_sent_bytes/1024:.1f} KB)")
            print(f"         GPS: Lat={meta['gps']['latitude']:.5f}, Lon={meta['gps']['longitude']:.5f}")
            print(f"         BBoxes Generated: {len(meta['bboxes'])} target(s)")

            if i < len(image_files):
                time.sleep(args.delay)

    except KeyboardInterrupt:
        print("\n[SENDER] Dihentikan oleh user.")
    finally:
        sock.close()
        print("[SENDER] Selesai & Koneksi ditutup.")

if __name__ == "__main__":
    main()
