#!/usr/bin/env python3
"""
=============================================================================
Fake UAV Image Sender | fake_uav_sender.py
=============================================================================
Dev-only test utility -- NOT part of the production backend. Sends
fragmented UDP frames matching the wire protocol UAVImageReceiver expects
(app/services/image/receiver.py), so the receiver can be exercised
end-to-end without a real UAV/edge-node.

Wire format per packet (matches _FrameReassembler):
    [4B frame_id][4B total_chunks][4B chunk_idx][chunk payload bytes]
    struct.pack("!III", frame_id, total_chunks, chunk_idx) + payload slice

EXIF written per frame (matches flight_metadata.extract_flight_metadata):
    - GPS IFD: GPSLatitude/GPSLongitude (DMS + N/S/E/W ref), GPSAltitude,
      GPSImgDirection (yaw)
    - 0th IFD ImageDescription: JSON string {"roll": .., "pitch": ..}

Requires: pip install piexif pillow

Usage:
    python3 fake_uav_sender.py --port 6001
    python3 fake_uav_sender.py --port 6001 --image real_sample.jpg --count 5
    python3 fake_uav_sender.py --port 6001 --shuffle
    python3 fake_uav_sender.py --port 6001 --drop-chunk-idx 2
=============================================================================
"""

import argparse
import io
import json
from pathlib import Path
import random
import socket
import struct
import time

import piexif
from PIL import Image, ImageDraw

_HEADER_FMT = "!III"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


# =============================================================================
# EXIF embedding
# =============================================================================
def _deg_to_dms_rational(value: float) -> tuple:
    """Decimal degrees -> ((deg,1),(min,1),(sec*1000,1000)) for piexif GPS tags."""
    value = abs(value)
    deg = int(value)
    minutes_float = (value - deg) * 60
    minutes = int(minutes_float)
    seconds = (minutes_float - minutes) * 60
    return ((deg, 1), (minutes, 1), (int(seconds * 1000), 1000))


def embed_flight_exif(
    jpeg_bytes: bytes,
    lat: float,
    lon: float,
    alt: float,
    yaw: float,
    roll: float,
    pitch: float,
) -> bytes:
    """Write GPS + ImageDescription EXIF matching flight_metadata.py's expectations."""
    gps_ifd = {
        piexif.GPSIFD.GPSLatitudeRef: "N" if lat >= 0 else "S",
        piexif.GPSIFD.GPSLatitude: _deg_to_dms_rational(lat),
        piexif.GPSIFD.GPSLongitudeRef: "E" if lon >= 0 else "W",
        piexif.GPSIFD.GPSLongitude: _deg_to_dms_rational(lon),
        piexif.GPSIFD.GPSAltitude: (int(alt * 100), 100),
        piexif.GPSIFD.GPSImgDirection: (int(yaw * 100) % 36000, 100),
    }
    zeroth_ifd = {
        piexif.ImageIFD.ImageDescription: json.dumps(
            {"roll": roll, "pitch": pitch}
        ).encode("utf-8"),
    }
    exif_dict = {"0th": zeroth_ifd, "GPS": gps_ifd}
    exif_bytes = piexif.dump(exif_dict)
    return piexif.insert(exif_bytes, jpeg_bytes)


# =============================================================================
# Frame source
# =============================================================================
def load_or_generate_jpeg(image_path: str | None, frame_index: int) -> bytes:
    """Use a real JPEG if given, otherwise generate a synthetic one so
    reassembled frames are visually distinguishable by frame number."""
    if image_path:
        with open(image_path, "rb") as f:
            return f.read()

    img = Image.new("RGB", (640, 480), color=(30, 30, 30))
    draw = ImageDraw.Draw(img)
    draw.text((20, 20), f"FAKE FRAME #{frame_index}", fill=(0, 255, 120))
    draw.text((20, 60), time.strftime("%Y-%m-%d %H:%M:%S"), fill=(200, 200, 200))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()

def load_dataset_sequence(image_dir: str) -> list[Path]:
    """Load a sequence of JPEG images from a directory, sorted by filename."""
    image_paths = sorted(Path(image_dir).glob("*.jpg")) + sorted(Path(image_dir).glob("*.jpeg")) 
    if not image_paths:
        raise ValueError(f"No JPEG images found in {image_dir}")
    return image_paths


# =============================================================================
# Fragmentation + send
# =============================================================================
def fragment(frame_bytes: bytes, chunk_size: int, frame_id: int) -> list[bytes]:
    """Split frame_bytes into chunk_size pieces, each prefixed with the
    12-byte !III header. Returns the list of ready-to-send packets."""
    total_chunks = (len(frame_bytes) + chunk_size - 1) // chunk_size
    packets = []
    for idx in range(total_chunks):
        start = idx * chunk_size
        payload = frame_bytes[start : start + chunk_size]
        header = struct.pack(_HEADER_FMT, frame_id, total_chunks, idx)
        packets.append(header + payload)
    return packets


def send_frame(
    sock: socket.socket,
    addr: tuple,
    frame_bytes: bytes,
    chunk_size: int,
    frame_id: int,
    shuffle: bool,
    drop_chunk_idx: int | None,
) -> None:
    packets = fragment(frame_bytes, chunk_size, frame_id)
    total_chunks = len(packets)

    order = list(range(total_chunks))
    if shuffle:
        random.shuffle(order)

    for idx in order:
        if drop_chunk_idx is not None and idx == drop_chunk_idx:
            print(f"  [frame {frame_id}] dropping chunk {idx}/{total_chunks - 1} on purpose")
            continue
        sock.sendto(packets[idx], addr)

    print(
        f"[frame {frame_id}] sent {total_chunks} chunks "
        f"({len(frame_bytes) / 1024:.1f} KB) -> {addr[0]}:{addr[1]}"
        f"{' (shuffled)' if shuffle else ''}"
    )


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(description="Fake UAV sender for testing UAVImageReceiver")
    parser.add_argument("--host", default="127.0.0.1", help="Receiver host")
    parser.add_argument("--port", type=int, required=True, help="Receiver UDP port")
    parser.add_argument("--image", default=None, help="Real JPEG to send (default: synthetic generated frame)")
    parser.add_argument("--image-dir", default=None, help="Directory containing real JPEG images to send")

    parser.add_argument("--chunk-size", type=int, default=1024, help="Bytes per UDP fragment")
    parser.add_argument("--count", type=int, default=1, help="Number of frames to send (loops back to the start of --image-dir if count exceeds file count)")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between frames")
    parser.add_argument("--shuffle", action="store_true", help="Send chunks out of order (tests reassembler)")
    parser.add_argument("--drop-chunk-idx", type=int, default=None, help="Deliberately omit this chunk index (tests cleanup_stale)")

    parser.add_argument("--lat", type=float, default=-7.782, help="Starting latitude")
    parser.add_argument("--lon", type=float, default=110.367, help="Starting longitude")
    parser.add_argument("--alt", type=float, default=80.0, help="Altitude (m)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw / GPSImgDirection (deg)")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll (deg)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch (deg)")
    parser.add_argument("--lat-step", type=float, default=0.0001, help="Latitude drift per frame (simulates forward flight)")
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    dataset_files = load_dataset_sequence(args.image_dir) if args.image_dir else None

    print(f"Sending {args.count} frame(s) to {addr[0]}:{addr[1]} ...")

    for i in range(args.count):
        frame_id = int(time.time() * 1000) & 0xFFFFFFFF
        if dataset_files:
            source_path = dataset_files[i % len(dataset_files)]
            jpeg_bytes = load_or_generate_jpeg(str(source_path), i)
            print(f"  [frame {frame_id}] using image {source_path.name}")
        else:
            jpeg_bytes = load_or_generate_jpeg(args.image, i)
            print(f"  [frame {frame_id}] using {'synthetic' if args.image is None else args.image}")
        # jpeg_bytes = embed_flight_exif(
        #     jpeg_bytes,
        #     lat=args.lat + i * args.lat_step,
        #     lon=args.lon,
        #     alt=args.alt,
        #     yaw=args.yaw,
        #     roll=args.roll,
        #     pitch=args.pitch,
        # )
        send_frame(
            sock, addr, jpeg_bytes, args.chunk_size, frame_id,
            shuffle=args.shuffle, drop_chunk_idx=args.drop_chunk_idx,
        )
        if i < args.count - 1:
            time.sleep(args.interval)

    print("Done.")


if __name__ == "__main__":
    main()
