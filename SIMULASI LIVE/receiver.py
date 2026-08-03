# receiver.py
# Menerima gambar via TCP socket dan langsung melakukan incremental stitching
# Simulasi live stitching receiver

import socket
import struct
import os
import time
import argparse
import numpy as np
import cv2
from pathlib import Path
from stitcher import LiveStitcher

# Disable OpenCL untuk stabilitas
cv2.ocl.setUseOpenCL(False)


def recvall(conn, n):
    """Baca tepat n byte dari socket, return None jika koneksi putus."""
    data = b""
    while len(data) < n:
        chunk = conn.recv(min(n - len(data), 65536))
        if not chunk:
            return None
        data += chunk
    return data


def main():
    parser = argparse.ArgumentParser(
        description='Simulasi receiver + live stitching'
    )
    parser.add_argument('--port', type=int, default=5050,
                        help='Port untuk menerima gambar (default: 5050)')
    parser.add_argument('--output-dir', type=str, default='output',
                        help='Direktori output untuk hasil stitching (default: output)')
    parser.add_argument('--downsample', type=int, default=5,
                        help='Faktor downsample gambar (default: 5)')
    parser.add_argument('--save-received', action='store_true',
                        help='Simpan juga gambar yang diterima ke folder received/')
    args = parser.parse_args()

    host = "0.0.0.0"
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Direktori untuk menyimpan gambar yang diterima (opsional)
    received_dir = None
    if args.save_received:
        received_dir = Path("received")
        received_dir.mkdir(parents=True, exist_ok=True)

    print("")
    print("=" * 60)
    print("  SIMULASI LIVE STITCHING — RECEIVER")
    print("=" * 60)
    print(f"  Listening   : {host}:{args.port}")
    print(f"  Output dir  : {output_dir.resolve()}")
    print(f"  Downsample  : {args.downsample}x")
    if args.save_received:
        print(f"  Received dir: {received_dir.resolve()}")
    print("=" * 60)
    print("")

    # Buat socket TCP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, args.port))
    sock.listen(1)

    print(f"[RECEIVER] Menunggu koneksi di port {args.port}...")
    print(f"[RECEIVER] Jalankan sender.py di terminal lain untuk mulai\n")

    try:
        while True:
            conn, addr = sock.accept()
            print(f"[RECEIVER] ✓ Terhubung dari {addr}")
            print("")

            # Buat stitcher baru untuk setiap koneksi
            stitcher = LiveStitcher(
                output_dir=str(output_dir),
                downsample_factor=args.downsample
            )

            img_counter = 0
            total_received_bytes = 0
            session_start = time.time()

            try:
                while True:
                    # 1. Baca 4 byte header (panjang data)
                    header = recvall(conn, 4)
                    if not header:
                        print("\n[RECEIVER] Sender terputus (tidak ada header).")
                        break

                    # 2. Unpack panjang data
                    length = struct.unpack("!I", header)[0]
                    if length == 0:
                        print("[RECEIVER] Menerima length 0, skip.")
                        continue

                    # 3. Baca data gambar
                    t_recv = time.time()
                    img_data = recvall(conn, length)
                    if img_data is None:
                        print("\n[RECEIVER] Sender terputus (data tidak lengkap).")
                        break
                    recv_time = time.time() - t_recv

                    img_counter += 1
                    total_received_bytes += length

                    print(f"\n[RECEIVER] ── Gambar #{img_counter} diterima ──")
                    print(f"  Ukuran: {length/1024:.1f} KB | Waktu terima: {recv_time:.3f}s")

                    # 4. Simpan gambar yang diterima (opsional)
                    if received_dir is not None:
                        save_path = received_dir / f"received_{img_counter:04d}.jpg"
                        with open(save_path, "wb") as f:
                            f.write(img_data)

                    # 5. Decode gambar
                    img_array = np.frombuffer(img_data, dtype=np.uint8)
                    image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if image is None:
                        print(f"  ⚠️  Gagal decode gambar #{img_counter}. Skip.")
                        continue

                    print(f"  Resolusi asli: {image.shape[1]}x{image.shape[0]}")

                    # 6. Stitch ke mosaic
                    t_stitch = time.time()
                    success, stitch_path = stitcher.add_image(image)
                    stitch_time = time.time() - t_stitch

                    if success and stitch_path:
                        print(f"  ✓ Stitch #{stitcher.stitch_count} selesai ({stitch_time:.2f}s)")
                    elif success and stitcher.image_count <= stitcher.stitch_start_threshold:
                        print(
                            f"  ⏳ Menunggu threshold: {stitcher.image_count}/"
                            f"{stitcher.stitch_start_threshold + 1} gambar"
                        )
                    elif success and not stitch_path:
                        print(f"  ✓ Threshold terlewati — mosaic diinisialisasi")
                    else:
                        print(f"  ⚠️  Stitch gagal — gambar diabaikan")

                    # Free memory
                    del img_data, img_array, image

            except KeyboardInterrupt:
                print("\n[RECEIVER] Dihentikan oleh user.")
            except Exception as e:
                print(f"\n[RECEIVER] Error: {e}")
                import traceback
                traceback.print_exc()
            finally:
                conn.close()
                session_elapsed = time.time() - session_start

                # Print ringkasan sesi
                print("")
                print("=" * 60)
                print("  RINGKASAN SESI")
                print("=" * 60)
                print(f"  Gambar diterima   : {img_counter}")
                print(f"  Stitch berhasil   : {stitcher.stitch_count}")
                print(f"  Total data        : {total_received_bytes/1024/1024:.2f} MB")
                print(f"  Waktu sesi        : {session_elapsed:.2f}s")
                
                # Simpan final result
                if stitcher.get_mosaic() is not None:
                    final_path = os.path.join(str(output_dir), "finalResult.png")
                    cv2.imwrite(final_path, stitcher.get_mosaic())
                    print(f"  Hasil akhir       : {final_path}")
                
                stitcher.print_summary()
                print("")
                print("[RECEIVER] Menunggu koneksi baru...")
                print("")

    except KeyboardInterrupt:
        print("\n[RECEIVER] Server dihentikan.")
    finally:
        sock.close()
        print("[RECEIVER] Socket ditutup.")


if __name__ == "__main__":
    main()
