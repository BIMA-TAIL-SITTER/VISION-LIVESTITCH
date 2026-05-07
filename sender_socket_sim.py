import socket
import struct
import os
import time
import argparse
from pathlib import Path
import glob

def send_image(sock, img_path):
    """ send image over socket """
    with open(img_path, "rb") as f:
        img_data = f.read()
    
    length = len(img_data)
    sock.sendall(struct.pack("!I", length))  # send length
    sock.sendall(img_data)                    # send image data
    return length

def main():
    parser = argparse.ArgumentParser(description='Simulate drone sending images via socket')
    parser.add_argument('--dataset-dir', type=str, required=True,
                        help='Directory containing images to send')
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Host to connect to')
    parser.add_argument('--port', type=int, default=5001, help='Port to connect to')
    parser.add_argument('--delay', type=float, default=0.2, help='Delay between sending images (seconds)')
    args = parser.parse_args()
    
    dataset_dir = Path(args.dataset_dir)
    if not dataset_dir.exists() or not dataset_dir.is_dir():
        print(f"[SENDER] Error: Dataset directory {dataset_dir} does not exist or is not a directory.")
        return
    
    image_files = []
    for extension in ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.JPG", "*.JPEG", "*.PNG", "*.BMP"]:
        image_files.extend(glob.glob(str(dataset_dir / extension)))
    
    image_files = sorted(list(set(image_files)))  # remove duplicates and sort
    if not image_files:
        print(f"[SENDER] No images found in {dataset_dir}")
        return
    
    print(f"[SENDER] Found {len(image_files)} images in {dataset_dir}")
    print(f"[SENDER] Connecting to {args.host}:{args.port} ...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((args.host, args.port))
        print("[SENDER] ✓ Connected to server!")
        print(f"[SENDER] Delay between images: {args.delay}s")
        print("")
    except Exception as e:
        print(f"[SENDER] ✗ Connection error: {e}")
        return
    
    try:
        sent_count = 0
        total_bytes = 0
        start_time_total = time.time() 

        
        for i, img_path in enumerate(image_files, 1):
            print(f"[SENDER] Sending image {i}/{len(image_files)}: {img_path}")
            
            start_time = time.time()
            img_size = send_image(sock, img_path)
            elapsed = time.time() - start_time
            
            total_bytes += img_size
            print(f"✓ {img_size/1024:.1f} KB ({elapsed:.2f}s)")
            sent_count += 1
            
            if i < len(image_files):
                time.sleep(args.delay)
            
        elapsed_total = time.time() - start_time_total
        print("")
        print("=" * 60)
        print(f"[SENDER] ✓ Complete!")
        print(f"[SENDER] Sent: {sent_count} images")
        print(f"[SENDER] Total: {total_bytes/1024/1024:.2f} MB")
        print(f"[SENDER] Time: {elapsed_total:.2f}s")
        print(f"[SENDER] Avg: {elapsed_total/sent_count:.2f}s per image")
        print("=" * 60)
            
    except KeyboardInterrupt:
        print("\n[SENDER] Interrupted by user.")
        print(f"[SENDER] Sent {sent_count}/{len(image_files)} images before interruption.")
    except BrokenPipeError:
        print("\n[SENDER] ✗ Connection lost (receiver disconnected)")
        print(f"[SENDER] Sent {sent_count}/{len(image_files)} images before disconnection.")
    except Exception as e:
        print(f"[SENDER] Error during sending: {e}")
    finally:
        print(f"[SENDER] Sent {sent_count} images. Closing connection.")
        sock.close()

if __name__ == "__main__":
    main()
