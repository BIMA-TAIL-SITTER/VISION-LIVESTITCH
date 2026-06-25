#!/usr/bin/env python3
"""
Send images from a Raspberry Pi folder to Ground Control Station (GCS)
via TCP socket.

Batch mode:
Reads all images from a folder and sends them sequentially via TCP socket.

Usage:
    python3 sender_gcs.py --dataset-dir <folder> --host <gcs_ip> --port <port> \
        [--delay <seconds>]

Example:
    python3 sender_gcs.py \
        --dataset-dir /home/pi/images \
        --host 192.168.1.100 \
        --port 5001 \
        --delay 0.2
"""

import argparse
import glob
import socket
import struct
import time
from pathlib import Path
from typing import List


class ImageSenderSocket:
    """Send images to Ground Control Station via TCP socket."""

    def __init__(self, host: str, port: int, timeout: int = 60):
        """
        Initialize TCP socket connection to GCS.

        Args:
            host: IP address of Ground Control Station.
            port: Port to connect to.
            timeout: Socket timeout in seconds.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self._connect()

    def _connect(self):
        """Establish TCP connection to GCS."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)

            print(f"[SOCKET] Connecting to {self.host}:{self.port} ...")
            self.sock.connect((self.host, self.port))
            print(f"[SOCKET] ✓ Connected to GCS: {self.host}:{self.port}")

        except Exception as e:
            print(f"[SOCKET] ✗ Connection failed: {e}")
            raise

    def send_image(self, img_path: str) -> bool:
        """
        Send a single image via TCP socket.

        Args:
            img_path: Path to local image file.

        Returns:
            True if successful, False otherwise.
        """
        try:
            with open(img_path, "rb") as file:
                img_data = file.read()

            # Send 4-byte length header, big-endian.
            length = len(img_data)
            self.sock.sendall(struct.pack("!I", length))

            # Send image data.
            self.sock.sendall(img_data)

            file_size = Path(img_path).stat().st_size
            print(f"✓ Sent {Path(img_path).name} ({file_size / 1024:.1f} KB)")
            return True

        except Exception as e:
            print(f"✗ Failed to send {img_path}: {e}")
            return False

    def send_batch(self, image_files: List[str], delay: float = 0.2) -> dict:
        """
        Send multiple images to GCS via TCP socket.

        Args:
            image_files: List of local image file paths.
            delay: Delay between sends in seconds.

        Returns:
            Dictionary with send statistics.
        """
        stats = {
            "total": len(image_files),
            "successful": 0,
            "failed": 0,
            "total_bytes": 0,
            "failed_files": [],
        }

        print(f"[SOCKET] Starting batch send: {len(image_files)} images")
        print(f"[SOCKET] Delay between sends: {delay}s")
        print("-" * 60)

        start_time_total = time.time()

        for index, img_path in enumerate(image_files, 1):
            print(f"[{index}/{len(image_files)}] ", end="")

            start_time = time.time()
            success = self.send_image(img_path)
            elapsed = time.time() - start_time

            if success:
                stats["successful"] += 1
                file_size = Path(img_path).stat().st_size
                stats["total_bytes"] += file_size
                print(f"    ({elapsed:.2f}s)")
            else:
                stats["failed"] += 1
                stats["failed_files"].append(img_path)

            if index < len(image_files):
                time.sleep(delay)

        elapsed_total = time.time() - start_time_total

        print("")
        print("=" * 60)
        print("[SOCKET] ✓ Batch send complete!")
        print(f"[SOCKET] Successful: {stats['successful']}/{stats['total']}")
        print(f"[SOCKET] Failed: {stats['failed']}/{stats['total']}")
        print(f"[SOCKET] Total size: {stats['total_bytes'] / 1024 / 1024:.2f} MB")
        print(f"[SOCKET] Total time: {elapsed_total:.2f}s")

        if stats["total"] > 0:
            print(f"[SOCKET] Average: {elapsed_total / stats['total']:.2f}s per image")

        print("=" * 60)

        if stats["failed_files"]:
            print("\n[SOCKET] Failed files:")
            for failed_file in stats["failed_files"]:
                print(f"  - {failed_file}")

        return stats

    def close(self):
        """Close TCP connection."""
        if self.sock:
            self.sock.close()
            print("[SOCKET] Connection closed.")


def get_image_files(dataset_dir: Path) -> List[str]:
    """
    Get all image files from directory.

    Supports:
    jpg, jpeg, png, bmp, JPG, JPEG, PNG, BMP.

    Args:
        dataset_dir: Path to dataset directory.

    Returns:
        Sorted list of image file paths.
    """
    if not dataset_dir.exists() or not dataset_dir.is_dir():
        print(f"[ERROR] Directory not found: {dataset_dir}")
        return []

    image_files = []
    extensions = [
        "*.jpg",
        "*.jpeg",
        "*.png",
        "*.bmp",
        "*.JPG",
        "*.JPEG",
        "*.PNG",
        "*.BMP",
    ]

    for extension in extensions:
        image_files.extend(glob.glob(str(dataset_dir / extension)))

    image_files = sorted(list(set(image_files)))

    if not image_files:
        print(f"[ERROR] No images found in {dataset_dir}")
        return []

    print(f"[LOADER] Found {len(image_files)} images")
    return image_files


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Send images from Raspberry Pi folder to Ground Control Station "
            "(GCS) via TCP socket"
        )
    )

    parser.add_argument(
        "--dataset-dir",
        type=str,
        required=True,
        help="Directory containing images to send, local path on Raspberry Pi",
    )

    parser.add_argument(
        "--host",
        type=str,
        required=True,
        help="IP address or hostname of Ground Control Station",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=5001,
        help="Port to connect to, default: 5001",
    )

    parser.add_argument(
        "--delay",
        type=float,
        default=0.2,
        help="Delay between sends in seconds, default: 0.2",
    )

    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="Socket timeout in seconds, default: 60",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print files to be sent without actually sending",
    )

    args = parser.parse_args()

    dataset_dir = Path(args.dataset_dir)
    image_files = get_image_files(dataset_dir)

    if not image_files:
        return

    print("[INFO] Images to send:")
    for index, file_path in enumerate(image_files, 1):
        print(f"  {index}. {Path(file_path).name}")

    if args.dry_run:
        print(
            f"\n[DRY-RUN] Would send {len(image_files)} images "
            f"to {args.host}:{args.port}"
        )
        return

    sender = None

    try:
        sender = ImageSenderSocket(
            host=args.host,
            port=args.port,
            timeout=args.timeout,
        )

        stats = sender.send_batch(
            image_files=image_files,
            delay=args.delay,
        )

        if stats["failed"] > 0:
            exit(1)

    except ConnectionRefusedError:
        print(
            f"[ERROR] Connection refused. "
            f"Check if GCS receiver is running at {args.host}:{args.port}"
        )
        exit(1)

    except TimeoutError:
        print(
            f"[ERROR] Connection timeout. "
            f"Check IP address, port, firewall, and receiver status."
        )
        exit(1)

    except Exception as e:
        print(f"[ERROR] {e}")
        exit(1)

    finally:
        if sender is not None:
            sender.close()


if __name__ == "__main__":
    main()