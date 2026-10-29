"""
Live Stitcher — CLI Entry Point.

Supports three operation modes:

    1. live     — TCP receiver + real-time stitcher
    2. batch    — Stitch all images from a folder
    3. simulate — Feed images from a folder with delay (simulates live)

Usage:
    python -m live_stitcher.main --mode batch --input-dir ./dataset --output mosaic.png
    python -m live_stitcher.main --mode simulate --input-dir ./dataset --delay 0.5
    python -m live_stitcher.main --mode live --port 5001

Author: BIMA SWARM — VISION-LIVESTITCH
"""

import argparse
import asyncio
import logging
import os
import sys
import time
import threading
from pathlib import Path
from queue import Queue
from typing import List

import cv2
import numpy as np

from .config import StitchConfig
from .stitcher import LiveStitcher
from .gps_utils import extract_gps_exif
from .receiver import ImageReceiver, ReceivedFrame

logger = logging.getLogger(__name__)


def setup_logging(verbose: bool = False):
    """Configure logging with colored output."""
    level = logging.DEBUG if verbose else logging.INFO
    fmt = "%(asctime)s [%(levelname)-5s] %(name)-20s | %(message)s"
    datefmt = "%H:%M:%S"

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)
    handler.setFormatter(logging.Formatter(fmt, datefmt=datefmt))

    root = logging.getLogger()
    root.setLevel(level)
    root.handlers.clear()
    root.addHandler(handler)


def get_image_files(directory: str) -> List[Path]:
    """Get sorted list of image files from directory.

    Args:
        directory: Path to directory containing images.

    Returns:
        Sorted list of image file paths.
    """
    extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    path = Path(directory)

    if not path.exists():
        raise FileNotFoundError(f"Directory not found: {directory}")

    files = []
    for f in sorted(path.iterdir()):
        if f.suffix.lower() in extensions and f.is_file():
            files.append(f)

    logger.info(f"Found {len(files)} images in {directory}")
    return files


def run_batch(args, config: StitchConfig):
    """Run batch stitching mode — process all images from a folder.

    Args:
        args: Parsed command-line arguments.
        config: Stitching configuration.
    """
    config.output_dir = args.output_dir or "output"
    stitcher = LiveStitcher(config)

    image_files = get_image_files(args.input_dir)
    if not image_files:
        logger.error("No images found in input directory")
        return

    logger.info(f"=== BATCH MODE: {len(image_files)} images ===")

    mosaic = None
    t_total_start = time.perf_counter()

    for i, img_path in enumerate(image_files):
        t_frame_start = time.perf_counter()

        try:
            result = stitcher.process_file(str(img_path))
            t_frame = (time.perf_counter() - t_frame_start) * 1000

            if result is not None:
                mosaic = result
                logger.info(
                    f"[{i+1}/{len(image_files)}] {img_path.name} — "
                    f"stitched ({t_frame:.0f}ms)"
                )
            else:
                logger.info(
                    f"[{i+1}/{len(image_files)}] {img_path.name} — "
                    f"skipped/filtered ({t_frame:.0f}ms)"
                )

        except ValueError as e:
            logger.warning(f"[{i+1}/{len(image_files)}] {img_path.name} — error: {e}")
        except Exception as e:
            logger.error(f"[{i+1}/{len(image_files)}] {img_path.name} — error: {e}")

    t_total = time.perf_counter() - t_total_start

    # Save final mosaic
    if mosaic is not None:
        output_name = args.output or "mosaic_final.png"
        output_path = stitcher.save_mosaic(output_name)
        logger.info(f"\n{'='*60}")
        logger.info(f"BATCH COMPLETE")
        logger.info(f"  Total time: {t_total:.1f}s")
        logger.info(f"  Output: {output_path}")
        logger.info(f"  Mosaic size: {mosaic.shape[1]}x{mosaic.shape[0]}")

        stats = stitcher.get_stats()
        logger.info(f"  Accepted: {stats['total_accepted']}/{stats['total_received']}")
        logger.info(f"  Failed: {stats['total_failed']}")
        logger.info(f"  Avg time/frame: {stats['avg_time_ms']:.0f}ms")
        logger.info(f"{'='*60}")
    else:
        logger.error("No mosaic produced — check input images and GPS data")


def run_simulate(args, config: StitchConfig):
    """Run simulation mode — feed images from folder with delay.

    Simulates live stitching by processing images one at a time
    with configurable delay between frames.

    Args:
        args: Parsed command-line arguments.
        config: Stitching configuration.
    """
    config.output_dir = args.output_dir or "output"
    stitcher = LiveStitcher(config)

    image_files = get_image_files(args.input_dir)
    if not image_files:
        logger.error("No images found in input directory")
        return

    delay = args.delay
    logger.info(f"=== SIMULATE MODE: {len(image_files)} images, delay={delay}s ===")

    mosaic = None
    frame_times = []

    for i, img_path in enumerate(image_files):
        t_start = time.perf_counter()

        try:
            result = stitcher.process_file(str(img_path))
            t_frame = (time.perf_counter() - t_start) * 1000

            if result is not None:
                mosaic = result
                frame_times.append(t_frame)

                # Save intermediate mosaic
                stitcher.save_mosaic("mosaic_latest.png")

                status = (
                    f"[{i+1}/{len(image_files)}] STITCHED | "
                    f"{t_frame:.0f}ms | "
                    f"avg={sum(frame_times)/len(frame_times):.0f}ms"
                )
                logger.info(status)
            else:
                logger.info(f"[{i+1}/{len(image_files)}] FILTERED/SKIPPED")

        except Exception as e:
            logger.warning(f"[{i+1}/{len(image_files)}] ERROR: {e}")

        # Simulate live delay
        if delay > 0 and i < len(image_files) - 1:
            time.sleep(delay)

    # Final summary
    if mosaic is not None:
        output_name = args.output or "mosaic_final.png"
        output_path = stitcher.save_mosaic(output_name)

        stats = stitcher.get_stats()
        logger.info(f"\n{'='*60}")
        logger.info(f"SIMULATION COMPLETE")
        logger.info(f"  Output: {output_path}")
        logger.info(f"  Accepted: {stats['total_accepted']}/{stats['total_received']}")
        if frame_times:
            logger.info(f"  Avg stitch time: {sum(frame_times)/len(frame_times):.0f}ms")
            logger.info(f"  Min stitch time: {min(frame_times):.0f}ms")
            logger.info(f"  Max stitch time: {max(frame_times):.0f}ms")
        logger.info(f"{'='*60}")


def run_live(args, config: StitchConfig):
    """Run live mode — TCP receiver + real-time stitcher.

    Starts a TCP server and processes incoming frames in real-time.

    Args:
        args: Parsed command-line arguments.
        config: Stitching configuration.
    """
    config.output_dir = args.output_dir or "output"
    config.receiver_host = args.host
    config.receiver_port = args.port

    stitcher = LiveStitcher(config)
    frame_queue: Queue = Queue()

    # Save directory for archiving received images
    save_dir = None
    if args.save_images:
        save_dir = str(Path(config.output_dir) / "received_images")

    receiver = ImageReceiver(
        host=config.receiver_host,
        port=config.receiver_port,
        output_queue=frame_queue,
        save_dir=save_dir,
    )

    logger.info(f"=== LIVE MODE ===")
    logger.info(f"  Listening: {config.receiver_host}:{config.receiver_port}")
    logger.info(f"  GPS threshold: {config.gps_distance_threshold_m}m")
    logger.info(f"  Output: {config.output_dir}")

    # Stitcher consumer thread
    def stitch_consumer():
        """Consumer thread that processes frames from the queue."""
        frame_count = 0
        while True:
            try:
                frame: ReceivedFrame = frame_queue.get(timeout=5.0)

                if not frame.gps:
                    logger.warning(f"Frame {frame.filename}: no GPS, skipping")
                    continue

                result = stitcher.process_frame(frame.image, frame.gps)
                frame_count += 1

                if result is not None:
                    # Save mosaic periodically
                    if frame_count % 3 == 0 or frame_count <= 5:
                        stitcher.save_mosaic("mosaic_latest.png")

                    stats = stitcher.get_stats()
                    logger.info(
                        f"Live mosaic updated | "
                        f"Accepted: {stats['total_accepted']} | "
                        f"Canvas: {stats['canvas_size'][0]}x{stats['canvas_size'][1]} | "
                        f"Avg: {stats['avg_time_ms']:.0f}ms/frame"
                    )

            except Exception:
                # Queue.get timeout — just loop
                continue

    consumer = threading.Thread(target=stitch_consumer, daemon=True)
    consumer.start()

    # Run receiver (blocking)
    try:
        asyncio.run(receiver.start())
    except KeyboardInterrupt:
        logger.info("\nShutting down...")

    # Final save
    mosaic = stitcher.get_mosaic()
    if mosaic is not None and mosaic.shape[0] > 100:
        stitcher.save_mosaic("mosaic_final.png")
        logger.info("Final mosaic saved")

    stats = stitcher.get_stats()
    logger.info(f"Session stats: {stats}")


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Live Stitcher — Incremental SIFT-based Mosaic Builder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Batch stitch from folder
  python -m live_stitcher.main --mode batch --input-dir ./dataset --output mosaic.png

  # Simulate live stitching
  python -m live_stitcher.main --mode simulate --input-dir ./dataset --delay 0.3

  # Live receiver + stitcher
  python -m live_stitcher.main --mode live --port 5001
        """,
    )

    parser.add_argument(
        "--mode",
        choices=["live", "batch", "simulate"],
        default="batch",
        help="Operation mode (default: batch)",
    )

    # Input
    parser.add_argument(
        "--input-dir",
        type=str,
        help="Input directory for batch/simulate mode",
    )

    # Output
    parser.add_argument(
        "--output",
        type=str,
        help="Output filename (default: mosaic_final.png)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="output",
        help="Output directory (default: output)",
    )

    # Network (live mode)
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Receiver bind address (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5001,
        help="Receiver TCP port (default: 5001)",
    )
    parser.add_argument(
        "--save-images",
        action="store_true",
        help="Save received images to disk (live mode)",
    )

    # Simulate
    parser.add_argument(
        "--delay",
        type=float,
        default=0.3,
        help="Delay between frames in simulate mode (default: 0.3s)",
    )

    # Stitching parameters
    parser.add_argument(
        "--gps-thresh",
        type=float,
        default=3.0,
        help="GPS redundancy threshold in meters (default: 3.0)",
    )
    parser.add_argument(
        "--sift-features",
        type=int,
        default=1500,
        help="Max SIFT keypoints (default: 1500)",
    )
    parser.add_argument(
        "--lowe-ratio",
        type=float,
        default=0.6,
        help="Lowe's ratio test threshold (default: 0.6)",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=3,
        help="Downsample factor (default: 3)",
    )
    parser.add_argument(
        "--feather-power",
        type=float,
        default=2.0,
        help="Feather blend exponent (default: 2.0)",
    )
    parser.add_argument(
        "--force-rigid",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enforce rigid Euclidean transform (scale=1.0) to prevent scale drift (default: True)",
    )

    # General
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Verbose logging (DEBUG level)",
    )

    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()
    setup_logging(args.verbose)

    # Build config from CLI args
    config = StitchConfig(
        gps_distance_threshold_m=args.gps_thresh,
        sift_n_features=args.sift_features,
        lowe_ratio=args.lowe_ratio,
        downsample_factor=args.downsample,
        blend_feather_power=args.feather_power,
        force_rigid=args.force_rigid,
        output_dir=args.output_dir,
    )

    logger.info(f"Live Stitcher v1.0.0")
    logger.info(f"Mode: {args.mode}")
    logger.info(f"Config: SIFT={config.sift_n_features} ratio={config.lowe_ratio} "
                f"GPS={config.gps_distance_threshold_m}m DS={config.downsample_factor}x")

    if args.mode == "batch":
        if not args.input_dir:
            logger.error("--input-dir required for batch mode")
            sys.exit(1)
        run_batch(args, config)

    elif args.mode == "simulate":
        if not args.input_dir:
            logger.error("--input-dir required for simulate mode")
            sys.exit(1)
        run_simulate(args, config)

    elif args.mode == "live":
        run_live(args, config)


if __name__ == "__main__":
    main()
