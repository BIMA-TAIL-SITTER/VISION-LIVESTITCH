"""
Live Stitcher — TCP Image Receiver.

Asyncio-based TCP server that receives images from drone senders
(compatible with sender_gcs.py and sender_socket_sim.py protocol).

Protocol: [4 bytes big-endian length][raw image bytes]

Images are received, GPS is extracted from EXIF in-memory,
and frames are pushed to a thread-safe queue for the stitcher.

Author: BIMA SWARM — VISION-LIVESTITCH
"""

import asyncio
import logging
import struct
import time
import os
from pathlib import Path
from typing import Optional, Callable
from queue import Queue
from dataclasses import dataclass

import cv2
import numpy as np

from .gps_utils import extract_gps_exif

logger = logging.getLogger(__name__)

# Header size: 4 bytes unsigned int (big-endian)
HEADER_SIZE = 4
HEADER_FORMAT = "!I"


@dataclass
class ReceivedFrame:
    """Container for a received image frame with metadata."""

    image: np.ndarray
    gps: dict
    filename: str
    timestamp: float
    size_bytes: int


class ImageReceiver:
    """Asyncio TCP server for receiving drone images.

    Compatible with sender_gcs.py and sender_socket_sim.py protocol:
    [4-byte big-endian length header][raw JPEG bytes]

    Features:
    - In-memory EXIF GPS extraction (no disk write required)
    - Optional disk saving for archival
    - Thread-safe queue output for consumer threads
    - Multi-client support (one UAV per connection)
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 5001,
        output_queue: Optional[Queue] = None,
        save_dir: Optional[str] = None,
        on_frame_callback: Optional[Callable] = None,
    ):
        """Initialize image receiver.

        Args:
            host: Bind address.
            port: TCP port to listen on.
            output_queue: Thread-safe queue to push ReceivedFrame objects to.
            save_dir: Optional directory to save received images to disk.
            on_frame_callback: Optional callback called with ReceivedFrame.
        """
        self.host = host
        self.port = port
        self.queue = output_queue or Queue()
        self.save_dir = Path(save_dir) if save_dir else None
        self.on_frame_callback = on_frame_callback

        self._frame_count = 0
        self._total_bytes = 0
        self._start_time: Optional[float] = None
        self._server: Optional[asyncio.Server] = None
        self._running = False

        if self.save_dir:
            self.save_dir.mkdir(parents=True, exist_ok=True)
            logger.info(f"Saving received images to: {self.save_dir}")

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ):
        """Handle a single client connection.

        Reads length-prefixed image frames from the TCP stream.
        """
        addr = writer.get_extra_info("peername")
        logger.info(f"Client connected: {addr}")

        try:
            while self._running:
                # Read 4-byte length header
                header = await reader.readexactly(HEADER_SIZE)
                length = struct.unpack(HEADER_FORMAT, header)[0]

                if length == 0 or length > 50_000_000:  # Max 50MB sanity check
                    logger.warning(
                        f"Invalid frame length: {length} bytes, skipping"
                    )
                    continue

                # Read image payload
                payload = await reader.readexactly(length)
                self._total_bytes += length

                # Decode image in memory
                img_array = np.frombuffer(payload, dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                if image is None:
                    logger.warning(
                        f"Failed to decode image ({length} bytes), skipping"
                    )
                    continue

                self._frame_count += 1

                # Extract GPS from EXIF in memory
                try:
                    gps = extract_gps_exif(bytes(payload))
                except (ValueError, Exception) as e:
                    logger.warning(
                        f"Frame {self._frame_count}: No GPS EXIF data: {e}"
                    )
                    gps = None

                # Generate filename
                ts = time.strftime("%Y%m%d_%H%M%S")
                filename = f"{ts}_{self._frame_count:04d}.jpg"

                # Optional: save to disk
                if self.save_dir and gps is not None:
                    save_path = self.save_dir / filename
                    with open(save_path, "wb") as f:
                        f.write(payload)

                # Create frame object
                frame = ReceivedFrame(
                    image=image,
                    gps=gps if gps else {},
                    filename=filename,
                    timestamp=time.time(),
                    size_bytes=length,
                )

                # Push to queue
                if gps is not None:
                    self.queue.put(frame)

                    # Callback
                    if self.on_frame_callback:
                        try:
                            self.on_frame_callback(frame)
                        except Exception as e:
                            logger.error(f"Frame callback error: {e}")

                    logger.debug(
                        f"Frame {self._frame_count} received: "
                        f"{image.shape[1]}x{image.shape[0]} "
                        f"({length / 1024:.1f} KB) "
                        f"GPS=({gps['latitude']:.6f}, {gps['longitude']:.6f})"
                    )
                else:
                    logger.debug(
                        f"Frame {self._frame_count} received without GPS, "
                        f"skipping stitcher queue"
                    )

        except asyncio.IncompleteReadError:
            logger.info(f"Client disconnected: {addr}")
        except ConnectionResetError:
            logger.info(f"Connection reset: {addr}")
        except Exception as e:
            logger.error(f"Error handling client {addr}: {e}")
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass
            logger.info(f"Client session ended: {addr}")

    async def start(self):
        """Start the TCP server."""
        self._running = True
        self._start_time = time.time()

        self._server = await asyncio.start_server(
            self._handle_client,
            self.host,
            self.port,
        )

        addrs = ", ".join(str(s.getsockname()) for s in self._server.sockets)
        logger.info(f"Image receiver listening on {addrs}")

        async with self._server:
            await self._server.serve_forever()

    async def stop(self):
        """Stop the TCP server."""
        self._running = False
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            logger.info("Image receiver stopped")

    @property
    def stats(self) -> dict:
        """Return receiver statistics."""
        elapsed = time.time() - self._start_time if self._start_time else 0
        return {
            "frames_received": self._frame_count,
            "total_bytes": self._total_bytes,
            "total_mb": self._total_bytes / (1024 * 1024),
            "elapsed_s": elapsed,
            "avg_fps": self._frame_count / max(1, elapsed),
            "avg_mbps": (self._total_bytes / (1024 * 1024)) / max(1, elapsed),
        }


def run_receiver_thread(
    host: str,
    port: int,
    output_queue: Queue,
    save_dir: Optional[str] = None,
) -> ImageReceiver:
    """Convenience function to run receiver in a thread.

    Usage:
        import threading
        queue = Queue()
        receiver = run_receiver_thread("0.0.0.0", 5001, queue)
        thread = threading.Thread(target=lambda: asyncio.run(receiver.start()), daemon=True)
        thread.start()
    """
    receiver = ImageReceiver(
        host=host,
        port=port,
        output_queue=output_queue,
        save_dir=save_dir,
    )
    return receiver
