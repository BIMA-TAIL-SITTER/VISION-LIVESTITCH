# sim_bbox_receiver.py
# Server Receiver yang menerima paket socket, mengekstrak data Bounding Box & GPS,
# dan melakukan Live Stitching dengan overlay Bounding Box.

import socket
import struct
import json
import cv2
import numpy as np
import time
import sys
import argparse
from pathlib import Path
from bbox_stitcher import BBoxStitcher

# Ensure UTF-8 output on Windows terminal
if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass


def recvall(conn, n):
    """Membaca persis n bytes dari socket TCP."""
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def main():
    parser = argparse.ArgumentParser(description="Server Receiver Live Stitching dengan Bounding Box Projection")
    parser.add_argument('--port', type=int, default=5050, help='Port TCP server (default 5050)')
    parser.add_argument('--downsample', type=float, default=2.0, help='Faktor downsample gambar untuk performa')
    parser.add_argument('--output-dir', type=str, default='output', help='Folder tempat menyimpan hasil')
    args = parser.parse_args()

    out_dir = Path(__file__).parent / args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("  LIVE STITCHER RECEIVER WITH BBOX PROJECTION")
    print("=" * 60)
    print(f"  Listening Port : {args.port}")
    print(f"  Output Folder  : {out_dir}")
    print(f"  Downsample     : {args.downsample}x")
    print("=" * 60)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", args.port))
    server_sock.listen(1)

    stitcher = BBoxStitcher(downsample=args.downsample)
    frame_count = 0

    print("[SERVER] Menunggu koneksi pengirim (sender)...")

    while True:
        conn, addr = server_sock.accept()
        print(f"[SERVER] [OK] Terhubung dari {addr}")

        try:
            while True:
                # 1. Baca 8 bytes header (4 byte meta len, 4 byte img len)
                header = recvall(conn, 8)
                if not header:
                    print("[SERVER] Connection closed by sender.")
                    break

                meta_len, img_len = struct.unpack("!II", header)

                # 2. Baca payload metadata JSON
                meta_bytes = recvall(conn, meta_len)
                if not meta_bytes:
                    break
                metadata = json.loads(meta_bytes.decode('utf-8'))

                # 3. Baca payload gambar JPEG
                img_bytes = recvall(conn, img_len)
                if not img_bytes:
                    break

                nparr = np.frombuffer(img_bytes, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                frame_count += 1
                print(f"\n[SERVER] Frame #{frame_count} Diterima.")
                print(f"         Metadata GPS: {metadata.get('gps')}")
                print(f"         Jumlah BBox: {len(metadata.get('bboxes', []))}")

                # 4. Ingest ke Live Stitcher Engine
                start_t = time.time()
                stitched_result = stitcher.add_frame(
                    img=img,
                    bboxes=metadata.get('bboxes', []),
                    gps_data=metadata.get('gps', {})
                )
                elapsed = time.time() - start_t
                print(f"         [OK] Live Stitching & BBox Warping selesai dalam {elapsed:.3f}s")

                # 5. Simpan Hasil Stitching
                step_file = out_dir / f"stitch_bbox_{frame_count:04d}.png"
                cv2.imwrite(str(step_file), stitched_result)

                final_file = out_dir / "final_stitched_bbox.png"
                cv2.imwrite(str(final_file), stitched_result)

                print(f"         Saved: {step_file.name} & final_stitched_bbox.png")

        except KeyboardInterrupt:
            print("\n[SERVER] Server dihentikan oleh user.")
            break
        except Exception as e:
            print(f"[SERVER] Error: {e}")
        finally:
            conn.close()
            print("[SERVER] Menunggu koneksi pengirim baru...")

if __name__ == "__main__":
    main()
