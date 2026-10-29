#!/usr/bin/env python3
"""
=============================================================================
run_stitch_dataset.py — Offline Stitcher untuk Dataset Statis
=============================================================================
Menjalankan pipeline stitching dari PROGRAM-SENDER1 pada dataset gambar
yang sudah ada di disk (tanpa perlu receiver/sender).

Alur kerja:
  1. Load semua gambar dari folder dataset
  2. Ekstrak metadata GPS dari EXIF (atau gunakan dummy progresif)
  3. Jalankan Combiner (dengan preprocessing, feature detection, matching,
     transformation, warping, dan blending)
  4. Simpan mosaic hasil stitching

Penggunaan:
  python run_stitch_dataset.py --dataset "dataset/Dataset_Coba" --output "stitch_output"
=============================================================================
"""

import os
import sys
import time
import math
import argparse
import cv2
import numpy as np

# Setup path agar bisa import src/
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)
sys.path.insert(0, os.path.join(BASE_DIR, "src"))

from src import Combiner
from src import utilities as util

cv2.ocl.setUseOpenCL(False)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline stitcher untuk dataset statis menggunakan Combiner"
    )
    parser.add_argument(
        "--dataset", type=str,
        default=os.path.join(BASE_DIR, "dataset", "Dataset_Coba"),
        help="Path ke folder dataset gambar"
    )
    parser.add_argument(
        "--output", type=str,
        default=os.path.join(BASE_DIR, "stitch_output_dataset_coba"),
        help="Folder output hasil stitching"
    )
    parser.add_argument(
        "--max-images", type=int, default=0,
        help="Batasi jumlah gambar (0 = semua)"
    )
    parser.add_argument(
        "--downsample", type=int, default=4,
        help="Downsample factor (default 4, sebelumnya 5)"
    )
    parser.add_argument(
        "--n-features", type=int, default=2500,
        help="Jumlah SIFT keypoints (default 2500, sebelumnya 1800)"
    )
    parser.add_argument(
        "--ratio-thresh", type=float, default=0.65,
        help="Lowe's ratio test threshold (default 0.65, sebelumnya 0.55)"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_dir = args.dataset
    output_dir = args.output

    os.makedirs(output_dir, exist_ok=True)

    print("=" * 65)
    print("  OFFLINE STITCHER — Dataset_Coba")
    print("=" * 65)
    print(f"  Dataset      : {dataset_dir}")
    print(f"  Output       : {output_dir}")
    print()

    # ── 1. Load gambar dan metadata ───────────────────────────────────────
    print("[1/3] Loading gambar dan metadata GPS dari dataset ...")
    t0 = time.time()

    allImages, metadata_list = util.importData(dataset_dir, return_as_dict=True)

    if args.max_images > 0:
        allImages = allImages[:args.max_images]
        metadata_list = metadata_list[:args.max_images]

    n = len(allImages)
    print(f"      -> {n} gambar berhasil di-load ({time.time()-t0:.1f}s)")

    if n < 2:
        print("ERROR: Butuh minimal 2 gambar untuk stitching!")
        sys.exit(1)

    # ── 2. Bangun data_matrix [x, y, z, yaw, pitch, roll] ────────────────
    print("[2/3] Menyiapkan data matrix (konversi GPS → meter) ...")

    origin = metadata_list[0]
    origin_lat = origin.get("latitude", 0.0)
    origin_lon = origin.get("longitude", 0.0)

    data_matrix = np.zeros((n, 6), dtype=np.float64)
    for i, meta in enumerate(metadata_list):
        lat = meta.get("latitude", 0.0)
        lon = meta.get("longitude", 0.0)
        alt = meta.get("altitude", 0.0)
        yaw = math.radians(meta.get("yaw", 0.0))
        pitch = math.radians(meta.get("pitch", 0.0))
        roll = math.radians(meta.get("roll", 0.0))

        # Konversi GPS → meter relatif terhadap gambar pertama
        x = (lon - origin_lon) * 111320 * math.cos(math.radians(origin_lat))
        y = (lat - origin_lat) * 110540

        data_matrix[i] = [x, y, alt, yaw, pitch, roll]

    print(f"      -> Data matrix: {data_matrix.shape}")
    print(f"      -> Origin GPS : ({origin_lat:.6f}, {origin_lon:.6f})")

    # ── 3. Jalankan Combiner ──────────────────────────────────────────────
    print(f"\n[3/3] Menjalankan Combiner stitching untuk {n} gambar ...")
    print(f"      Params: downsample={args.downsample}, SIFT={args.n_features}, ratio={args.ratio_thresh}")
    print("=" * 65)

    t_stitch = time.time()
    combiner = Combiner.Combiner(
        allImages, data_matrix, output=output_dir,
        downsample=args.downsample,
        n_features=args.n_features,
        ratio_thresh=args.ratio_thresh
    )
    mosaic = combiner.create_mosaic()
    elapsed = time.time() - t_stitch

    if mosaic is not None:
        # Simpan hasil akhir
        final_path = os.path.join(output_dir, "final_mosaic.png")
        cv2.imwrite(final_path, mosaic)

        print("\n" + "=" * 65)
        print("  STITCHING SELESAI!")
        print("=" * 65)
        print(f"  Jumlah gambar  : {n}")
        print(f"  Waktu total    : {elapsed:.2f}s")
        print(f"  Ukuran mosaic  : {mosaic.shape[1]} x {mosaic.shape[0]} px")
        print(f"  File output    : {final_path}")
        print("=" * 65)
    else:
        print("\nERROR: Stitching gagal (return None)!")
        sys.exit(1)


if __name__ == "__main__":
    main()
