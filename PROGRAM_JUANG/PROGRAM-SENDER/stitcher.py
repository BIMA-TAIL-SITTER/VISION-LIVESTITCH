#!/usr/bin/env python3
"""
=============================================================================
PROGRAM-SENDER | stitcher.py
=============================================================================
Dijalankan di: Laptop / Ground Control Station (GCS)
Dijalankan BERSAMA receiver.py (proses terpisah)

Fungsi:
  1. Memantau folder sesi dari receiver
  2. Menerapkan threshold GPS: gambar baru hanya di-stitch jika cukup jauh
    dari gambar sebelumnya (mengurangi redundansi dan mempercepat stitching)
  3. Mengumpulkan gambar dalam batch sebelum menjalankan stitching
  4. Overlay bounding box object detection (dari detections.jsonl) ke mosaic
  5. Menyimpan hasil mosaic dengan anotasi lokasi

Cara Jalankan:
  python3 stitcher.py --session flight_1
  python3 stitcher.py --session flight_1 --batch 5 --gps-threshold 3.0
=============================================================================
"""

import os
import sys
import time
import json
import threading
import argparse
import logging
import math
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))  # Parent dir untuk src/

import config

# Import modul stitching dari project utama
try:
    from src import Combiner, utilities as util
    from src.blending import ROIfeatherBlender
    STITCHER_AVAILABLE = True
except ImportError:
    try:
        # Jika dijalankan dari folder PROGRAM-SENDER langsung
        parent = Path(__file__).resolve().parent.parent
        sys.path.insert(0, str(parent))
        from src import Combiner, utilities as util
        from src.blending import ROIfeatherBlender
        STITCHER_AVAILABLE = True
    except ImportError as e:
        print(f"[STITCHER] WARNING: Tidak bisa import modul stitching: {e}")
        STITCHER_AVAILABLE = False

# Disable OpenCL untuk stabilitas
cv2.ocl.setUseOpenCL(False)

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [STITCHER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("stitcher")


# =============================================================================
# GPS Utilities
# =============================================================================
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Hitung jarak dua titik GPS dalam meter (Haversine formula)."""
    R = 6371000  # Radius bumi meter
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# =============================================================================
# GPS Threshold Filter
# =============================================================================
class GPSThresholdFilter:
    """
    Filter gambar berdasarkan jarak GPS.
    Hanya menerima gambar jika jarak dari gambar terakhir ≥ threshold.
    Ini menggantikan threshold berbasis jumlah gambar dengan threshold spasial.
    """

    def __init__(self, threshold_m: float = config.GPS_DISTANCE_THRESHOLD_M):
        self.threshold_m  = threshold_m
        self._last_lat    = None
        self._last_lon    = None
        self._accepted    = 0
        self._rejected    = 0

    def should_accept(self, lat: float, lon: float) -> Tuple[bool, float]:
        """
        Return (should_accept, distance_from_last).
        Jika belum ada referensi, selalu terima.
        """
        if self._last_lat is None:
            self._last_lat = lat
            self._last_lon = lon
            self._accepted += 1
            return True, 0.0

        dist = haversine_m(self._last_lat, self._last_lon, lat, lon)
        if dist >= self.threshold_m:
            self._last_lat = lat
            self._last_lon = lon
            self._accepted += 1
            return True, dist
        else:
            self._rejected += 1
            return False, dist

    @property
    def stats(self) -> str:
        total = self._accepted + self._rejected
        return (f"Accepted: {self._accepted}/{total} gambar "
                f"(threshold={self.threshold_m}m)")


# =============================================================================
# Detection Loader
# =============================================================================
class DetectionLoader:
    """Membaca detections.jsonl yang diisi oleh receiver."""

    def __init__(self, jsonl_path: Path):
        self.path = jsonl_path
        self._last_pos = 0

    def load_all(self) -> List[dict]:
        """Muat semua deteksi dari file JSONL."""
        records = []
        if not self.path.exists():
            return records
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line:
                        try:
                            records.append(json.loads(line))
                        except json.JSONDecodeError:
                            pass
        except Exception as e:
            log.error(f"Gagal membaca detections: {e}")
        return records


# =============================================================================
# Mosaic Overlay – menggambar bounding box + label GPS ke mosaic
# =============================================================================
class MosaicOverlay:
    """
    Menggambar hasil object detection ke atas mosaic.

    Setiap deteksi punya:
    - bbox: posisi piksel di frame asli UAV
    - geo:  koordinat GPS tempat deteksi

    Karena mosaic adalah proyeksi ortho, kita overlay label teks dengan
    koordinat GPS di posisi proporsional pada mosaic.
    """

    COLORS = {
        "default": (0, 255, 0),       # Hijau
        "person":  (0, 0, 255),       # Merah
        "vehicle": (255, 128, 0),     # Oranye
        "fire":    (0, 0, 255),       # Merah
    }
    FONT       = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    THICKNESS  = 2

    def draw_detections(self,
                        mosaic: np.ndarray,
                        detections: List[dict],
                        origin_lat: float,
                        origin_lon: float,
                        mosaic_scale_mpx: float = 0.5) -> np.ndarray:
        """
        Overlay semua deteksi ke mosaic.

        Args:
            mosaic:           Gambar mosaic (BGR numpy array)
            detections:       List dict deteksi dari ODStore
            origin_lat/lon:   GPS referensi pojok kiri atas mosaic
            mosaic_scale_mpx: Meter per piksel mosaic (estimasi)

        Returns:
            Mosaic dengan overlay
        """
        result = mosaic.copy()
        h, w   = mosaic.shape[:2]

        for packet in detections:
            pkg_lat = packet.get("latitude", 0)
            pkg_lon = packet.get("longitude", 0)
            pkg_alt = packet.get("altitude", 0)

            for det in packet.get("detections", []):
                label = det.get("label", "object")
                conf  = det.get("confidence", 0.0)
                geo   = det.get("geo", {})

                det_lat = geo.get("lat", pkg_lat)
                det_lon = geo.get("lon", pkg_lon)
                det_alt = geo.get("alt", pkg_alt)

                # Konversi GPS ke piksel mosaic (estimasi linear)
                dx_m = (det_lon - origin_lon) * 111320 * math.cos(math.radians(origin_lat))
                dy_m = (origin_lat - det_lat) * 110540  # Y terbalik (lat turun = piksel naik)

                cx = int(dx_m / mosaic_scale_mpx)
                cy = int(dy_m / mosaic_scale_mpx)

                # Skip jika di luar mosaic
                if not (0 <= cx < w and 0 <= cy < h):
                    log.debug(f"Deteksi ({det_lat:.5f},{det_lon:.5f}) di luar mosaic, skip overlay")
                    continue

                color = self.COLORS.get(label.lower(), self.COLORS["default"])

                # Gambar marker dan bounding box estimasi (radius 20px)
                BOX_R = 25
                cv2.rectangle(result,
                              (cx - BOX_R, cy - BOX_R),
                              (cx + BOX_R, cy + BOX_R),
                              color, self.THICKNESS)
                cv2.circle(result, (cx, cy), 5, color, -1)

                # Label teks
                label_str  = f"{label} {conf:.0%}"
                coord_str  = f"Lat:{det_lat:.5f} Lon:{det_lon:.5f} Alt:{det_alt:.1f}m"

                # Background untuk teks
                for i, txt in enumerate([label_str, coord_str]):
                    ty = cy - BOX_R - 5 - (i * 20)
                    (tw, th), _ = cv2.getTextSize(txt, self.FONT, self.FONT_SCALE, self.THICKNESS)
                    cv2.rectangle(result,
                                  (cx - BOX_R, ty - th - 2),
                                  (cx - BOX_R + tw + 4, ty + 2),
                                  (0, 0, 0), -1)
                    cv2.putText(result, txt,
                                (cx - BOX_R + 2, ty),
                                self.FONT, self.FONT_SCALE, (255, 255, 255), 1,
                                cv2.LINE_AA)

        return result


# =============================================================================
# Stitching Engine
# =============================================================================
class StitchingEngine:
    """
    Wrapper stitching yang menggunakan Combiner dari project utama.
    Fallback ke OpenCV Stitcher jika modul tidak tersedia.
    """

    def stitch(self, images: List[np.ndarray],
               gps_list: List[dict]) -> Optional[np.ndarray]:
        """
        Stitch list gambar. Kembalikan mosaic atau None jika gagal.
        """
        if len(images) < 2:
            log.warning("Butuh minimal 2 gambar untuk stitching")
            return images[0] if images else None

        if STITCHER_AVAILABLE:
            return self._stitch_external(images, gps_list)
        else:
            return self._stitch_opencv(images)

    def _stitch_external(self, images: List[np.ndarray],
                     gps_list: List[dict]) -> Optional[np.ndarray]:
        """Gunakan Combiner dari project utama."""
        try:
            log.info(f"Menjalankan Combiner untuk {len(images)} gambar ...")

            # Buat data matrix [x, y, z, yaw, pitch, roll]
            origin_lat = gps_list[0].get("latitude", 0.0)
            origin_lon = gps_list[0].get("longitude", 0.0)

            data_matrix = np.zeros((len(gps_list), 6), dtype=np.float64)
            for i, gps in enumerate(gps_list):
                lat = gps.get("latitude", 0.0)
                lon = gps.get("longitude", 0.0)
                alt = gps.get("altitude", 0.0)
                x = (lon - origin_lon) * 111320 * math.cos(math.radians(origin_lat))
                y = (lat - origin_lat) * 110540
                data_matrix[i, 0] = x
                data_matrix[i, 1] = y
                data_matrix[i, 2] = alt

            combiner = Combiner.Combiner(images, data_matrix, output="stitch_output")
            result   = combiner.create_mosaic()
            log.info("✓ Combiner selesai")
            return result

        except Exception as e:
            log.error(f"Combiner error: {e}")
            import traceback; traceback.print_exc()
            log.info("Fallback ke OpenCV Stitcher ...")
            return self._stitch_opencv(images)

    def _stitch_opencv(self, images: List[np.ndarray]) -> Optional[np.ndarray]:
        """Fallback: OpenCV built-in stitcher."""
        log.info(f"Menggunakan OpenCV Stitcher untuk {len(images)} gambar ...")
        try:
            stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
            status, result = stitcher.stitch(images)
            if status == cv2.Stitcher_OK:
                log.info("✓ OpenCV Stitcher selesai")
                return result
            else:
                log.error(f"OpenCV Stitcher gagal: status={status}")
                return None
        except Exception as e:
            log.error(f"OpenCV Stitcher exception: {e}")
            return None


# =============================================================================
# Session Monitor
# =============================================================================
class SessionMonitor:
    """
    Memantau folder sesi, menerapkan GPS threshold, dan menjalankan stitching
    ketika batch gambar cukup.
    """

    def __init__(self, session_id: str, base_dir: str,
                 batch_size: int, gps_threshold_m: float):
        self.session_id     = session_id
        self.session_root   = Path(base_dir) / session_id
        self.img_dir        = self.session_root / "images"
        self.output_dir     = self.session_root / "output"
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.batch_size     = batch_size
        self.gps_filter     = GPSThresholdFilter(gps_threshold_m)
        self.det_loader     = DetectionLoader(self.session_root / "detections.jsonl")
        self.overlay        = MosaicOverlay()
        self.engine         = StitchingEngine()

        self._processed_imgs: set = set()
        self._accepted_imgs:  List[Path] = []  # Gambar yang lolos GPS filter
        self._accepted_gps:   List[dict] = []

        self._is_stitching  = False
        self._mosaic_count  = 0
        self._lock          = threading.Lock()

        # GPS referensi untuk overlay (set dari gambar pertama)
        self._origin_lat    = None
        self._origin_lon    = None

    def scan_new_images(self) -> List[Path]:
        """Scan folder untuk gambar baru yang belum diproses."""
        all_imgs = sorted(
            [p for ext in ["*.jpg", "*.jpeg", "*.png"]
             for p in self.img_dir.glob(ext)],
            key=lambda p: p.stat().st_mtime
        )
        return [p for p in all_imgs if p.name not in self._processed_imgs]

    def process_new_images(self, new_imgs: List[Path]):
        """
        Terapkan GPS filter ke gambar baru.
        Gambar yang lolos masuk ke batch.
        """
        for img_path in new_imgs:
            self._processed_imgs.add(img_path.name)

            # Ekstrak GPS dari EXIF atau nama file
            gps = self._extract_gps(img_path)
            lat = gps.get("latitude", 0.0)
            lon = gps.get("longitude", 0.0)

            accept, dist = self.gps_filter.should_accept(lat, lon)

            if accept:
                self._accepted_imgs.append(img_path)
                self._accepted_gps.append(gps)

                if self._origin_lat is None:
                    self._origin_lat = lat
                    self._origin_lon = lon

                log.info(
                    f"✓ Gambar diterima: {img_path.name} "
                    f"(jarak={dist:.1f}m) | Batch: {len(self._accepted_imgs)}/{self.batch_size}"
                )
            else:
                log.info(
                    f"  Gambar DILEWATI (jarak={dist:.1f}m < {self.gps_filter.threshold_m}m): "
                    f"{img_path.name}"
                )

    def should_stitch(self) -> bool:
        return (not self._is_stitching and
                len(self._accepted_imgs) >= self.batch_size)

    def run_stitch(self):
        """Jalankan stitching pada batch saat ini."""
        with self._lock:
            if self._is_stitching:
                return
            self._is_stitching = True

            # Ambil batch saat ini
            imgs_to_stitch = list(self._accepted_imgs)
            gps_to_stitch  = list(self._accepted_gps)

        t_start = time.time()
        log.info(f"\n{'='*60}")
        log.info(f"MULAI STITCHING | {len(imgs_to_stitch)} gambar | Batch #{self._mosaic_count + 1}")
        log.info(f"{'='*60}")

        try:
            # Load gambar ke memori
            images = []
            valid_gps = []
            for path, gps in zip(imgs_to_stitch, gps_to_stitch):
                img = cv2.imread(str(path))
                if img is not None:
                    images.append(img)
                    valid_gps.append(gps)
                else:
                    log.warning(f"Gagal membaca gambar: {path}")

            if len(images) < 2:
                log.error("Tidak cukup gambar valid untuk stitching")
                return

            # Jalankan stitching
            mosaic = self.engine.stitch(images, valid_gps)

            if mosaic is None:
                log.error("Stitching gagal (engine return None)")
                return

            # Overlay object detections
            detections = self.det_loader.load_all()
            if detections and self._origin_lat is not None:
                log.info(f"Overlay {sum(len(d.get('detections',[])) for d in detections)} deteksi OD ...")
                mosaic = self.overlay.draw_detections(
                    mosaic, detections,
                    self._origin_lat, self._origin_lon
                )

            # Simpan hasil
            self._mosaic_count += 1
            ts          = time.strftime("%Y%m%d_%H%M%S")
            out_name    = f"mosaic_{self._mosaic_count:03d}_{ts}.png"
            out_path    = self.output_dir / out_name
            cv2.imwrite(str(out_path), mosaic)

            # Simpan juga sebagai "latest"
            latest_path = self.output_dir / "mosaic_latest.png"
            cv2.imwrite(str(latest_path), mosaic)

            elapsed = time.time() - t_start
            log.info(f"✓ Mosaic #{self._mosaic_count} disimpan: {out_name}")
            log.info(f"  Ukuran: {mosaic.shape[1]}x{mosaic.shape[0]}px")
            log.info(f"  Waktu stitching: {elapsed:.2f}s")

        except Exception as e:
            log.error(f"Exception saat stitching: {e}")
            import traceback; traceback.print_exc()
        finally:
            with self._lock:
                self._is_stitching = False

    def run_stitch_async(self):
        """Jalankan stitching di thread terpisah agar tidak memblokir monitor."""
        t = threading.Thread(target=self.run_stitch, daemon=True)
        t.start()

    def _extract_gps(self, img_path: Path) -> dict:
        """
        Ekstrak GPS dari EXIF gambar.
        Fallback ke GPS dummy jika tidak ada EXIF.
        """
        try:
            import exifread
            with open(img_path, "rb") as f:
                tags = exifread.process_file(f, details=False)

            if "GPS GPSLatitude" in tags and "GPS GPSLongitude" in tags:
                lat_vals = tags["GPS GPSLatitude"].values
                lon_vals = tags["GPS GPSLongitude"].values
                lat_ref  = str(tags.get("GPS GPSLatitudeRef", "N"))
                lon_ref  = str(tags.get("GPS GPSLongitudeRef", "E"))
                alt_tag  = tags.get("GPS GPSAltitude")

                lat = (lat_vals[0].num / lat_vals[0].den +
                       lat_vals[1].num / (lat_vals[1].den * 60) +
                       lat_vals[2].num / (lat_vals[2].den * 3600))
                lon = (lon_vals[0].num / lon_vals[0].den +
                       lon_vals[1].num / (lon_vals[1].den * 60) +
                       lon_vals[2].num / (lon_vals[2].den * 3600))
                alt = float(alt_tag.values[0].num / alt_tag.values[0].den) if alt_tag else 0.0

                if lat_ref.strip() == "S": lat = -lat
                if lon_ref.strip() == "W": lon = -lon

                return {"latitude": lat, "longitude": lon, "altitude": alt}
        except Exception:
            pass

        # Fallback: gunakan GPS dummy progresif agar stitcher tetap berjalan
        idx = len(self._processed_imgs)
        return {
            "latitude":  -7.123456 + idx * 0.0001,
            "longitude": 112.654321 + idx * 0.0001,
            "altitude":  80.0
        }

    def print_summary(self):
        log.info("\n" + "=" * 50)
        log.info("RINGKASAN STITCHING")
        log.info(f"  {self.gps_filter.stats}")
        log.info(f"  Mosaic dihasilkan: {self._mosaic_count}")
        log.info("=" * 50)


# =============================================================================
# Main
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="Stitcher – GPS-threshold stitcher dengan OD overlay"
    )
    parser.add_argument("--session",       type=str,  default=config.DEFAULT_SESSION_ID)
    parser.add_argument("--base-dir",      type=str,  default=config.SESSION_DIR)
    parser.add_argument("--batch",         type=int,  default=config.STITCH_BATCH_SIZE,
                        help="Jumlah gambar per batch stitching")
    parser.add_argument("--gps-threshold", type=float, default=config.GPS_DISTANCE_THRESHOLD_M,
                        help="Jarak minimum antar gambar (meter)")
    parser.add_argument("--poll-interval", type=float, default=2.0,
                        help="Interval scan folder (detik)")
    return parser.parse_args()


def main():
    args = parse_args()

    log.info("=" * 60)
    log.info("PROGRAM-SENDER Stitcher v1.0")
    log.info(f"  Sesi            : {args.session}")
    log.info(f"  Batch size      : {args.batch} gambar")
    log.info(f"  GPS threshold   : {args.gps_threshold} m")
    log.info(f"  Poll interval   : {args.poll_interval} s")
    log.info(f"  Combiner stitcher : {'✓ Tersedia' if STITCHER_AVAILABLE else '✗ Fallback OpenCV'}")
    log.info("=" * 60)

    monitor = SessionMonitor(
        session_id      = args.session,
        base_dir        = args.base_dir,
        batch_size      = args.batch,
        gps_threshold_m = args.gps_threshold
    )

    log.info("Memantau folder sesi ... Tekan Ctrl+C untuk berhenti.")

    try:
        while True:
            new_imgs = monitor.scan_new_images()
            if new_imgs:
                monitor.process_new_images(new_imgs)

            if monitor.should_stitch():
                monitor.run_stitch_async()

            time.sleep(args.poll_interval)

    except KeyboardInterrupt:
        log.info("\nDihentikan oleh pengguna.")
    finally:
        monitor.print_summary()


if __name__ == "__main__":
    main()
