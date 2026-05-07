# receiver_socket.py
import socket
import struct
import os
import time
import argparse
from pathlib import Path

HOST = "0.0.0.0"  # listen di semua interface
PORT = 5001       # bebas, asal sama dengan sender

def recvall(conn, n):
    """Baca tepat n byte dari socket, atau return None kalau koneksi putus."""
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def main():
    parser = argparse.ArgumentParser(description='Receive images from drone via socket')
    parser.add_argument('--session-id', type=str, default='session_1',
                        help='Session ID to save images to')
    parser.add_argument('--port', type=int, default=5001,
                        help='Port to listen on')
    args = parser.parse_args()
    
    # Use session_id from argument
    SAVE_DIR = Path(f"sessions/{args.session_id}/images")
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    
    print(f"[SERVER] Saving images to: {SAVE_DIR}")
    print(f"[SERVER] Listening on {HOST}:{args.port} ...")
    
    # buat socket TCP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow quick restart
    sock.bind((HOST, args.port))
    sock.listen(1)

    while True:
        conn, addr = sock.accept()
        print(f"[SERVER] Connected from {addr}")

        img_counter = 0

        try:
            while True:
                # 1) baca 4 byte panjang data
                header = recvall(conn, 4)
                if not header:
                    print("[SERVER] Client disconnected (no header).")
                    break

                # 2) unpack jadi integer (big-endian)
                length = struct.unpack("!I", header)[0]
                if length == 0:
                    print("[SERVER] Got length 0, skip.")
                    continue

                # 3) baca data gambar sepanjang length
                img_data = recvall(conn, length)
                if img_data is None:
                    print("[SERVER] Client disconnected (no data).")
                    break

                # 4) buat nama file: timestamp + counter
                ts = time.strftime("%Y%m%d_%H%M%S")
                fname = f"{ts}_{img_counter:04d}.jpg"
                save_path = SAVE_DIR / fname

                with open(save_path, "wb") as f:
                    f.write(img_data)

                print(f"[SERVER] Saved: {save_path} ({len(img_data)} bytes)")
                img_counter += 1

        except KeyboardInterrupt:
            print("\n[SERVER] Shutting down...")
            break
        except Exception as e:
            print(f"[SERVER] Error: {e}")
        finally:
            conn.close()
            print("[SERVER] Connection closed, waiting for new client...")

if __name__ == "__main__":
    main()