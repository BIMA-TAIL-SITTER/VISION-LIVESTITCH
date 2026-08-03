# sender.py
# Mengirim gambar dari folder dataset via TCP socket
# Simulasi pengiriman gambar dari drone/kamera

import socket
import struct
import os
import time
import argparse
from pathlib import Path
import glob


def send_image(sock, img_path):
    """Kirim satu gambar melalui socket."""
    with open(img_path, "rb") as f:
        img_data = f.read()
    
    length = len(img_data)
    sock.sendall(struct.pack("!I", length))  # Kirim panjang data (4 bytes)
    sock.sendall(img_data)                   # Kirim data gambar
    return length


def main():
    parser = argparse.ArgumentParser(
        description='Simulasi pengiriman gambar dari dataset via socket'
    )
    parser.add_argument(
        '--dataset-dir', type=str,
        default=os.path.join(os.path.dirname(__file__), '..', 'dataset', '100GOPRO'),
        help='Direktori berisi gambar untuk dikirim (default: ../dataset/100GOPRO)'
    )
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help='Host tujuan (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=5050,
                        help='Port tujuan (default: 5050)')
    parser.add_argument('--delay', type=float, default=0.5,
                        help='Delay antar pengiriman gambar dalam detik (default: 0.5)')
    parser.add_argument('--max-images', type=int, default=0,
                        help='Batasi jumlah gambar yang dikirim (0 = semua)')
    args = parser.parse_args()

    dataset_dir = Path(args.dataset_dir).resolve()
    if not dataset_dir.exists() or not dataset_dir.is_dir():
        print(f"[SENDER] ✗ Error: Direktori {dataset_dir} tidak ditemukan!")
        return

    # Cari semua file gambar
    image_files = []
    for ext in ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.JPG", "*.JPEG", "*.PNG", "*.BMP"]:
        image_files.extend(glob.glob(str(dataset_dir / ext)))
    
    image_files = sorted(list(set(image_files)))
    if not image_files:
        print(f"[SENDER] ✗ Tidak ada gambar ditemukan di {dataset_dir}")
        return

    # Batasi jumlah gambar jika diminta
    if args.max_images > 0:
        image_files = image_files[:args.max_images]

    print("")
    print("=" * 60)
    print("  SIMULASI LIVE STITCHING — SENDER")
    print("=" * 60)
    print(f"  Dataset    : {dataset_dir}")
    print(f"  Jumlah     : {len(image_files)} gambar")
    print(f"  Tujuan     : {args.host}:{args.port}")
    print(f"  Delay      : {args.delay}s antar gambar")
    print("=" * 60)
    print("")

    # Koneksi ke receiver
    print(f"[SENDER] Menghubungi {args.host}:{args.port} ...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((args.host, args.port))
        print("[SENDER] ✓ Terhubung ke receiver!")
        print("")
    except Exception as e:
        print(f"[SENDER] ✗ Gagal terhubung: {e}")
        print("[SENDER]   Pastikan receiver sudah berjalan terlebih dahulu!")
        return

    try:
        sent_count = 0
        total_bytes = 0
        start_time_total = time.time()

        for i, img_path in enumerate(image_files, 1):
            filename = os.path.basename(img_path)
            print(f"[SENDER] Mengirim gambar {i}/{len(image_files)}: {filename}", end=" ")

            start_time = time.time()
            img_size = send_image(sock, img_path)
            elapsed = time.time() - start_time

            total_bytes += img_size
            print(f"✓ {img_size/1024:.1f} KB ({elapsed:.3f}s)")
            sent_count += 1

            # Delay antar gambar (kecuali gambar terakhir)
            if i < len(image_files):
                time.sleep(args.delay)

        elapsed_total = time.time() - start_time_total
        print("")
        print("=" * 60)
        print(f"  [SENDER] ✓ SELESAI!")
        print(f"  Gambar terkirim : {sent_count}")
        print(f"  Total data      : {total_bytes/1024/1024:.2f} MB")
        print(f"  Waktu total     : {elapsed_total:.2f}s")
        print(f"  Rata-rata       : {elapsed_total/sent_count:.2f}s per gambar")
        print("=" * 60)

    except KeyboardInterrupt:
        print(f"\n[SENDER] Dihentikan oleh user. {sent_count}/{len(image_files)} gambar terkirim.")
    except BrokenPipeError:
        print(f"\n[SENDER] ✗ Koneksi terputus (receiver disconnect)")
        print(f"[SENDER] {sent_count}/{len(image_files)} gambar terkirim sebelum terputus.")
    except Exception as e:
        print(f"[SENDER] ✗ Error: {e}")
    finally:
        sock.close()
        print("[SENDER] Koneksi ditutup.")


if __name__ == "__main__":
    main()
