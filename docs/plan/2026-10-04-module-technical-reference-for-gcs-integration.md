# Technical Reference: Stitching Engine Baru — buat Integrasi ke GCS (`app/routers/stitching.py`)

> **STANDALONE DOC** — ditulis 2026-10-04 di repo VISION-LIVESTITCH, dibawa ke session terpisah di repo GCS/base-station. Repo VISION-LIVESTITCH **TIDAK bisa diakses** dari session itu, jadi source code yang relevan (`flight_metadata.py` penuh, `Combiner.py` penuh) di-INLINE LANGSUNG di §Lampiran — JANGAN coba `Read` path `src/...` dari repo VISION-LIVESTITCH, gak akan ketemu, semuanya ada di dokumen ini.
>
> **Target refactor**: `app/routers/stitching.py` di repo GCS (base station) — file REAL ini BISA diakses dari session barunya, baca langsung buat liat state TERKINI-nya (analisis di dokumen ini based on snapshot yang direview 2026-10-04, kemungkinan udah ada perubahan lain sejak itu).

---

## 1. Konteks: kenapa dokumen ini ada

GCS punya router stitching yang JALAN (`app/routers/stitching.py`, ~670 baris per snapshot 2026-10-04), tapi dia manggil **engine stitching versi LAMA** (kemungkinan di `stitching_service/src/Combiner.py` di sisi GCS) — versi SEBELUM serangkaian hardening yang udah dikerjain di proyek VISION-LIVESTITCH (chain-bridging fix, GPS sanity check, attitude drift gate, dll). Tugas integrasi = **ganti engine yang dipanggil router itu** pake versi baru (full source di §Lampiran), plus nyesuain data pipeline di sekitarnya — BUKAN nulis ulang router dari nol, BUKAN juga sekadar ganti 1 baris import.

**Poin paling krusial di seluruh dokumen ini**: engine baru cuma efektif kalo `data_matrix` yang dikasih ke `Combiner` punya **yaw/pitch/roll REAL**, bukan nol. Kalo router cuma ganti `import` tapi tetep bangun `data_matrix` dengan cara lama (yang gak pernah baca attitude), gate attitude-drift & sebagian GPS-sanity-check bakal JADI NO-OP DIAM-DIAM — gak error, cuma gak ngapa-ngapain. Detail di §5.

---

## 2. Ringkasan: apa yang beda dari engine lama

| | **Engine LAMA** (dugaan, di sisi GCS) | **Engine BARU** (full source §Lampiran) |
|---|---|---|
| Data pose yang diterima | `data_matrix` Nx6, kemungkinan **cuma kolom X/Y/altitude keisi**, Yaw/Pitch/Roll **SELALU 0** (dibangun manual dari GPS-only metadata) | `data_matrix` Nx7 (`build_data_matrix`), **Yaw/Pitch/Roll REAL** dari EXIF, + kolom ke-7 `has_telemetry` |
| Gate sebelum stitching | Kemungkinan cuma `inlier_ratio < 0.6` | Attitude drift check → feature match → inlier ratio → GPS translation sanity check (lihat §4) |
| Admission filter (sebelum file masuk proses stitching) | Kemungkinan gak ada | `GPSDistanceFilter` + `AttitudeThresholdFilter` (lihat §3) |
| Reject handling | Kemungkinan: skip doang, posisi mosaic beku kalo ada yang ketolak (gejala: patahan/staircase di mosaic kalo reject rate tinggi) | `__bridge_or_freeze` — bounded fallback pake GPS, `MAX_BRIDGE_GAP=3` (lihat §4) |

(Kolom "Engine LAMA" ditandain "kemungkinan" karena dokumen ini gak bisa baca source code engine lama itu langsung — cek kondisi REAL-nya begitu baca `app/routers/stitching.py` & file yang dia import.)

---

## 3. Fitur BARU — daftar konkret, cocokin satu-satu ke behavior GCS sekarang

1. **Ekstraksi attitude REAL dari EXIF** (`extract_flight_metadata`, §Lampiran A) — GPS (lat/lon/alt) dari GPS IFD standar, yaw dari `GPSImgDirection`, roll/pitch dari JSON compact di tag `ImageDescription`. Format JSON itu: `{"roll": <deg>, "pitch": <deg>, "yaw": <deg>}`.
2. **Dua admission filter SEBELUM gambar masuk proses stitching** (`GPSDistanceFilter`, `AttitudeThresholdFilter`, §Lampiran A) — bukan di dalam `Combiner`, tapi di LAPISAN INGESTION (titik di mana file gambar pertama kali diproses, sebelum masuk daftar "siap distitch"). Nolak gambar yang kejauhan miring ATAU kejauhan/kedeketan jaraknya SEBELUM `Combiner` pernah liat gambar itu.
3. **Attitude drift gate** (`__check_attitude_drift`, §Lampiran B) — bandingin roll/pitch frame sekarang vs frame TERAKHIR YANG DITERIMA (`self.last_acc_index`), independen dari SIFT. Dipanggil PALING AWAL di `combine()`, sebelum feature detection apapun. Nangkep drift bertahap pas drone manuver yang gak kekejar sama cek pasangan biasa (dua frame manuver yang bersebelahan bisa "konsisten" satu sama lain walau keduanya udah sama-sama drift jauh).
4. **GPS translation sanity check** (`__check_gps_translation_sanity`, §Lampiran B) — validasi independen: translasi yang DIUKUR SIFT dibandingin translasi dari data GPS asli (titik tengah gambar, bukan origin — biar gak kena efek amplifikasi rotasi). Toleransi `TRANSLATION_TOLERANCE_M=10.0` meter, dikalibrasi dari data real (gap kosong antara noise-korridor-normal vs kink-zona-manuver), bukan tebakan.
5. **Kalibrasi `meters_per_pixel` OTOMATIS saat runtime** (inline di `combine()`, §Lampiran B) — gak perlu input spek kamera/altitude fisik manual (EXIF altitude itu MSL/ketinggian laut, bukan AGL/ketinggian dari tanah — gak reliable buat itung skala). Ngambil median dari 5 sample pair yang match-nya BAGUS BANGET (`inlier_ratio>0.7`, lebih ketat dari gate accept `0.6`), terus freeze buat sisa sesi.
6. **Bounded fallback/bridging** (`__bridge_or_freeze`, `__simple_fallback_transform`, §Lampiran B) — pas ada gambar ketolak gate manapun, posisi mosaic tetep "dimajuin" (translasi dari GPS doang, gak ada rotasi) buat `MAX_BRIDGE_GAP=3` frame reject beruntun, abis itu freeze total. Mencegah 2 masalah sekaligus: patahan/staircase kalo selalu freeze, DAN ghosting kalo selalu bridging tanpa batas (dua-duanya pernah dicoba & gagal, makanya solusinya "dibatasin").
7. **Inlier ratio dipisah jadi 2 level** — `0.6` buat accept gate, `0.7` (lebih ketat) buat eligibility kalibrasi `meters_per_pixel` (poin 5). Kalibrasi itu keputusan sekali-pakai-buat-selamanya per sesi, jadi standarnya sengaja lebih tinggi dari standar accept biasa.

**Masih SAMA kayak kebanyakan engine lain** (biar gak dikira ikut berubah kalo ternyata GCS juga udah punya ini): algoritma redundant-frame/keyframe-selection — ADA di source (`redundant_filter.py`, diimport) tapi TETEP GAK DIPANGGIL di alur aktif; strategi blending — cuma 1 (`ROIfeatherBlender`, feather blend berbasis distance-transform) yang aktif dipanggil; full-mosaic re-warp tiap step (belum dioptimasi, cost makin naik seiring canvas makin gede); TIDAK ADA bundle adjustment/loop-closure global (pairwise chaining by design, trade-off real-time vs akurasi jangka panjang — drift kumulatif adalah batasan yang disadari, bukan bug).

---

## 4. Alur gate `combine(index)` — urutan lengkap

```
0. Attitude drift check (vs last_acc_index) — PALING AWAL, sebelum SIFT jalan sama sekali
   gagal → bridge_or_freeze() → return, skip sisanya
1. Deteksi fitur SIFT di image[index-1] vs image[index]
   gagal (gak ada fitur) → bridge_or_freeze() → skip
2. Feature matching (BFMatcher + ratio test 0.55)
   match < 4 → bridge_or_freeze() → skip
3. Estimasi transform (affine dulu, fallback ke homography+RANSAC)
   gagal total → bridge_or_freeze() → skip
4. Cek inlier ratio (dari RANSAC/affine inlier mask)
   < 0.6 → bridge_or_freeze() → skip
5. [kalo meters_per_pixel belum kekalibrasi]
   → coba kalibrasi (butuh inlier_ratio > 0.7 & jarak GPS > 2m, kumpulin 5 sample)
6. GPS translation sanity check (translasi SIFT vs translasi GPS, pake meters_per_pixel)
   beda > 10m → bridge_or_freeze() → skip
7. SEMUA LOLOS → last_acc_index = index, reset counter reject,
   warp + blend pixel beneran ke mosaic, simpan intermediateResult_{index}.png
```

`bridge_or_freeze()`: counter reject beruntun ditambah. Kalo masih `<= MAX_BRIDGE_GAP (3)`, majuin posisi mosaic pake fallback translasi GPS-only (TANPA nempel pixel apapun — cuma geser referensi posisi). Kalo counter udah lewat 3, diem aja (freeze), gak ngapa-ngapain sampe ada frame yang beneran lolos lagi.

State penting di `Combiner.__init__` (lihat §Lampiran B buat detail): `self._meters_per_pixel` (`None` sampe kekalibrasi — sanity check & fallback OTOMATIS SKIP selama ini `None`), `self._mpp_calibration_samples`, `self._consecutive_rejections`, `self.MAX_BRIDGE_GAP=3`, `self.last_acc_index=0`.

---

## 5. KRUSIAL: yang perlu dicek & kemungkinan WAJIB diubah di `app/routers/stitching.py`

Checklist buat dicek SATU-SATU begitu baca file real-nya di session baru:

1. **Cari di mana `data_matrix`/pose matrix dibangun sebelum dikasih ke `Combiner`.** Kalo isinya cuma dari GPS (lat/lon/altitude) tanpa baca attitude dari EXIF sama sekali → **WAJIB diganti** pake pola `extract_flight_metadata()` per file + `build_data_matrix(metadata_list)` dari §Lampiran A. Kalo enggak, attitude-drift gate & sebagian GPS-sanity-check di engine baru JADI NO-OP DIAM-DIAM (lihat §1) — mosaic ya tetep jalan, TAPI dua gate penting itu gak pernah beneran aktif karena roll/pitch yang diterima selalu 0.
2. **Cek apakah ada admission filter** (reject gambar kejauhan miring / kejauhan-kedeketan jaraknya) SEBELUM gambar masuk daftar yang di-stitch. Kalo belum ada, perlu DIPUTUSIN taro di titik mana: SEBELUM file ditulis ke disk (reject gak usah ditulis sama sekali), atau SETELAH ditulis tapi SEBELUM masuk daftar "siap distitch" (butuh struktur "accepted list" TERPISAH dari "semua file di folder gambar"). Pake `GPSDistanceFilter`/`AttitudeThresholdFilter` dari §Lampiran A — defaultnya `threshold_m=3.0` dan `threshold_deg=30.0`.
3. **Kalo GCS punya jalur ingestion dari LIVE VIDEO STREAM** (snapshot frame dari video feed, bukan upload file) — jalur ini PERLU DIPIKIRIN KHUSUS, karena frame dari situ KEMUNGKINAN gak punya EXIF attitude ter-embed (beda dari gambar yang datang dari sender UAV yang udah embed EXIF duluan). Kalo gak diisi, `extract_flight_metadata()` bakal nemu `has_telemetry=False` buat semua frame dari jalur ini → admission filter attitude otomatis gak pernah jalan buat jalur ini. Perlu diputusin: embed EXIF manual pas nulis frame dari stream, atau terima gap ini buat jalur live-capture.
4. **Cek EXIF-parsing yang dipake sekarang** — kalo GCS punya implementasi parsing EXIF sendiri (misal via `PIL` atau cara lain), JANGAN dicampur sebagian pake yang lama sebagian pake `extract_flight_metadata()` baru. Format JSON compact di `ImageDescription` (§Lampiran A) HARUS persis sama kayak yang ditulis di sisi UAV/sender — kalo beda format, roll/pitch gak akan ke-parse walau EXIF-nya ADA.
5. **Perhatiin concurrency model yang GCS udah punya** — kalo router udah pake `asyncio.to_thread` (atau pola setara: offload kerja CPU-bound ke thread pool TANPA block event loop) buat manggil `create_mosaic()`, PERTAHANIN pola itu, jangan diturunin ke pola yang lebih kasar (misal raw `threading.Thread` tanpa tracking/cancel). Ini best-practice yang independen dari integrasi engine, jangan sampe ke-regresi pas refactor.
6. **Pertahanin validasi keamanan yang mungkin udah ada** di router GCS (contoh: validasi session-ID format, validasi nama file output biar gak bisa path-traversal) — kalo ADA, JANGAN HILANG pas nyalin pola baru.

---

## 6. Alur integrasi yang disaranin (urutan kerja)

```
1. Ganti source code engine stitching di sisi GCS (lokasi file engine lama di
   GCS -- cari `Combiner.py` di repo GCS) dengan full source §Lampiran B.
   Tambahin file BARU setara §Lampiran A (flight_metadata.py) -- ini modul
   yang gak ada padanannya di versi lama, based on code review 2026-10-04.
2. Cek & ganti cara data_matrix/pose-matrix dibangun di router (§5 poin 1).
3. Putusin & implementasi titik admission filter (§5 poin 2), termasuk jalur
   live-stream-capture kalau ada (§5 poin 3).
4. Tes end-to-end: minimal 1 sequence gambar yang PUNYA EXIF attitude lengkap,
   verifikasi lewat log bahwa attitude-drift gate & GPS sanity check BENERAN
   aktif (bukan selalu lolos karena roll/pitch=0) -- cara paling gampang cek:
   print/log drift value tiap combine() dipanggil, pastiin variatif gak
   selalu 0.
5. Kalo ada perbedaan hasil mosaic dibanding sebelum migrasi, itu EXPECTED --
   engine baru lebih KETAT (lebih banyak gate), beberapa frame yang dulu
   lolos sekarang mungkin ketolak. Itu bukan regresi, itu yang diinginkan.
```

---

## 7. Konteks tambahan (ringkasan, bukan file yang bisa diakses dari sini)

Beberapa keputusan terkait yang DIAMBIL di proyek VISION-LIVESTITCH per 2026-10-04, relevan buat integrasi tapi di luar scope dokumen ini (dicatet ringkas aja, detail lengkapnya ada di repo asal yang gak bisa diakses dari session ini):

- **Auto-stitch queue/batching**: ditemukan bug "gambar terakhir yatim" (image yang masuk PAS SETELAH satu batch stitch mulai jalan bisa nyangkut gak ke-stitch sampe ada trigger lagi). Arah desain yang lagi dipertimbangin: producer/consumer dengan queue GLOBAL (lintas semua UAV/session) + worker pool DIBATASI (ganti model "spawn thread baru tanpa batas tiap trigger" yang beresiko jadi bottleneck pas banyak UAV jalan bareng). Threading vs multiprocessing buat worker pool masih didiskusiin — profiling nunjukin ~98% hot path stitching (SIFT, warp, blend) itu panggilan `cv2`/`numpy` yang ngelepas GIL Python, jadi argumen "harus multiprocessing karena CPU-bound" gak langsung berlaku bersih di codebase ini.
- **UDP vs TCP buat ingestion gambar dari UAV**: channel UDP yang LIVE dipake buat akuisisi data (fragmented UDP, reassemble di sisi GCS) belum pernah ngalamin data loss tercatat, TAPI belum dikonfirmasi apakah itu udah termasuk kondisi RF paling berat (manuver aktif). Instrumentasi loss-rate counter baru ditambahin, rencana ukur di uji terbang berikutnya sebelum mutusin pindah ke TCP atau tetep UDP.

Kalo detail dua poin di atas relevan buat kerjaan di session base station ini, user bisa kasih tau ulang detailnya — dokumen ini sengaja gak nge-link ke file yang gak reachable dari sini.

---

## Lampiran A: `flight_metadata.py` — FULL SOURCE (177 baris, snapshot 2026-10-04)

Modul baru, gak ada padanan di versi lama. Simpan sebagai file baru di sisi GCS (nama file bebas, konvensi di source aslinya `src/flight_metadata.py`), diimport dari router.

```python
# src/flight_metadata.py
"""
Correct EXIF/GPS/attitude extraction and admission filters for the
composable stitching orchestrator.

This module NEVER converts yaw/pitch/roll to radians -- the unrotation
function that consumes dataMatrix (geometry.computeUnRotMatrix in the
source engine) does that conversion itself. Converting here too would
silently double-convert the angles.
"""
import json
import math
from typing import Dict, List, Optional, Tuple

import exifread
import numpy as np

def extract_flight_metadata(image_path: str) -> Dict:
    """
    Read GPS (lat/lon/alt, GPS IFD) + yaw (GPSImgDirection) + roll/pitch
    (ImageDescription JSON) from a JPEG's EXIF, matching the convention
    the UAV-side sender's embed_flight_metadata_exif() writes. All angles
    returned in DEGREES.

    Returns has_telemetry=False + zero pose (never fabricated GPS) if no
    GPS EXIF is present.
    """

    result = {
        "latitude": None, "longitude": None, "altitude": 0.0,
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "has_telemetry": False,
    }
    try:
        with open(image_path, 'rb') as f:
            tags = exifread.process_file(f, details=False)
        if "GPS GPSLatitude" in tags and "GPS GPSLongitude" in tags:
            lat_vals = tags["GPS GPSLatitude"].values
            lon_vals = tags["GPS GPSLongitude"].values
            lat_ref = str(tags.get("GPS GPSLatitudeRef", "N"))
            lon_ref = str(tags.get("GPS GPSLongitudeRef", "E"))
            alt_tag = tags.get("GPS GPSAltitude")
            dir_tag = tags.get("GPS GPSImgDirection")

            # Convert DMS to decimal degrees
            lat = (lat_vals[0].num / lat_vals[0].den +
                   lat_vals[1].num / (lat_vals[1].den * 60) +
                   lat_vals[2].num / (lat_vals[2].den * 3600))

            lon = (lon_vals[0].num / lon_vals[0].den +
                   lon_vals[1].num / (lon_vals[1].den * 60) +
                   lon_vals[2].num / (lon_vals[2].den * 3600))

            alt = float(alt_tag.values[0].num / alt_tag.values[0].den) if alt_tag else 0.0
            yaw = float(dir_tag.values[0].num / dir_tag.values[0].den) if dir_tag else 0.0

            if lat_ref.strip() == "S":
                lat = -lat
            if lon_ref.strip() == "W":
                lon = -lon

            result.update({
                "latitude": lat, "longitude": lon, "altitude": alt,
                "yaw": yaw, "has_telemetry": True,
            })

        desc_tag = tags.get("Image ImageDescription")
        if desc_tag is not None:
            try:
                attitude = json.loads(str(desc_tag))
                result["roll"] = float(attitude.get("roll", 0.0))
                result["pitch"] = float(attitude.get("pitch", 0.0))
            except (ValueError, TypeError):
                pass

    except Exception as e:
        print(f"[flight_metadata] Failed to extract EXIF metadata from {image_path}: {type(e).__name__}: {e}")

    if result["latitude"] is None:
        result["latitude"] = 0.0
        result["longitude"] = 0.0
        result["altitude"] = 0.0
        result["roll"] = 0.0
        result["pitch"] = 0.0
        result["yaw"] = 0.0
        result["has_telemetry"] = False

    """
    output result dictionary with keys:
    - latitude: float (degrees)
    - longitude: float (degrees)
    - altitude: float (meters)
    - roll: float (degrees)
    - pitch: float (degrees)
    - yaw: float (degrees)
    - has_telemetry: bool (True if GPS data was present, False otherwise)
    """
    return result

def build_data_matrix(metadata_list: List[Dict], origin: Optional[Tuple[float, float]] = None) -> np.ndarray:
    """
    Build the Nx7 [X, Y, Z, Yaw, Pitch, Roll, has_telemetry] pose matrix the
    stitching engine (Combiner, see Lampiran B) expects.
    X/Y are local meters relative to `origin` (defaults to the first entry's lat/lon).
    Yaw/Pitch/Roll stay in DEGREES -- the unrotation step converts to
    radians itself; converting here would reintroduce the double-conversion bug.
    """
    if origin is None:
        origin_lat = metadata_list[0]["latitude"]
        origin_lon = metadata_list[0]["longitude"]
    else:
        origin_lat, origin_lon = origin

    data_matrix = np.zeros((len(metadata_list), 7), dtype=np.float64)
    for i, meta in enumerate(metadata_list):
        lat = meta["latitude"]
        lon = meta["longitude"]
        x = (lon - origin_lon) * 111320 * math.cos(math.radians(origin_lat))
        y = (lat - origin_lat) * 110540
        data_matrix[i, 0] = x
        data_matrix[i, 1] = y
        data_matrix[i, 2] = meta["altitude"]
        data_matrix[i, 3] = meta["yaw"]
        data_matrix[i, 4] = meta["pitch"]
        data_matrix[i, 5] = meta["roll"]
        data_matrix[i, 6] = 1.0  if meta["has_telemetry"] else 0.0
    return data_matrix


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters between two lat/lon points."""
    R = 6371000  # Earth radius, meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class GPSDistanceFilter:
    """
    Redundancy filter: only accepts a new position if it's at least
    `threshold_m` away from the last ACCEPTED position. One instance
    per session -- sharing an instance across sessions/UAVs would compare
    positions from unrelated flights.
    """

    def __init__(self, threshold_m: float = 3.0):
        self.threshold_m = threshold_m
        self._last_lat: Optional[float] = None
        self._last_lon: Optional[float] = None

    def should_accept(self, lat: float, lon: float) -> Tuple[bool, float]:
        if self._last_lat is None:
            self._last_lat = lat
            self._last_lon = lon
            return True, 0.0

        dist = haversine_m(self._last_lat, self._last_lon, lat, lon)
        if dist >= self.threshold_m:
            self._last_lat = lat
            self._last_lon = lon
            return True, dist
        return False, dist

class AttitudeThresholdFilter:
    """
    Quality gate: rejects a frame if roll or pitch exceeds threshold_deg.
    Checked BEFORE GPSDistanceFilter in the admission pipeline -- swapping
    the order would let a tilt-rejected frame still mutate the GPS filter's
    reference position as a side effect.
    """

    def __init__(self, threshold_deg: float = 30.0):
        self.threshold_deg = threshold_deg

    def should_accept(self, roll_deg: float, pitch_deg: float) -> Tuple[bool, float]:
        worst = max(abs(roll_deg), abs(pitch_deg))
        return worst < self.threshold_deg, worst
```

---

## Lampiran B: `Combiner.py` — FULL SOURCE (519 baris, snapshot 2026-10-04)

Engine inti. Ganti file engine LAMA di sisi GCS (`Combiner.py`-nya, lokasi tepatnya cek sendiri di repo GCS) dengan isi ini. Dependensi internal (`utilities`, `geometry`, `blending`, `src.redundant_filter`) diasumsikan SAMA kayak yang GCS udah punya (file-file itu gak berubah dari versi lama, per catatan di §3) — kalo nama modul/path import beda di GCS, sesuain baris `import` di bagian atas, LOGIKA di dalam class gak perlu diubah.

```python
# src/Combiner.py
import cv2
import numpy as np
import sys
import os
import time
import gc
sys.path.insert(0, os.path.dirname(__file__))
from src.redundant_filter import keyframe_selection_yuan2024, redundancy_removal_he2024
import utilities as util
import geometry as gm
import copy
from blending import PyramidBlender, ROIfeatherBlender

class Combiner:
    def __init__(self,imageList_,dataMatrix_, output="output"):
        '''
        :param imageList_: List of all images in dataset.
        :param dataMatrix_: Matrix with all pose data in dataset.
        :return:
        '''
        self.dataMatrix = dataMatrix_
        self.output_dir = output
        self.match_output_dir = os.path.join(self.output_dir, "matches")
        self.timing_stats = {
            'preprocessing': 0,
            'feature_detection': 0,
            'matching': 0,
            'transformation': 0,
            'warping': 0,
            'blending': 0,
            'total': 0
        }
        
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.match_output_dir, exist_ok=True)

        self.H_global_prev = np.eye(3, dtype=np.float32)
        self.H_rel_prev = None

        # GPS translation sanity check parameters
        # for MSL AGL calibration
        self._mpp_calibration_samples = []
        self._meters_per_pixel = None

        self.image_list = self.__preprocess_images(imageList_)
        self.result_image = self.image_list[0]

        # bridge fallback
        self._consecutive_rejections = 0
        self.MAX_BRIDGE_GAP = 3

        # last accepted frame index for attitude drift check
        self.last_acc_index = 0

    # ------------------------------------------------------------------ #
    #  PRIVATE HELPERS                                                     #
    # ------------------------------------------------------------------ #

    def __preprocess_images(self, raw_image):
        '''Downsample and unrotate all input images'''
        t0 = time.time()
        processed_images = []

        for i, img in enumerate(raw_image):
            downsampled = img[::5, ::5]  # Downsample by factor of 5
            M = gm.computeUnRotMatrix(self.dataMatrix[i,:])
            correctedImage = gm.warpPerspectiveWithPadding(downsampled, M)
            processed_images.append(correctedImage)

            # clear memory
            raw_image[i] = None
            if (i + 1 ) % 5 == 0:
                gc.collect()
                print(f"Processed {i + 1}/{len(raw_image)} images. Memory cleared.")

        del raw_image
        gc.collect()

        self.timing_stats['preprocessing'] = time.time() - t0
        print(f"⏱️  Preprocessing: {self.timing_stats['preprocessing']:.3f}s")
        return processed_images
    
    def __detect_features(self, image):
        ''' Detect features for a given image '''
        detector = cv2.SIFT_create(1800)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        kp, descriptors = detector.detectAndCompute(gray, mask)
        return kp, descriptors
    
    def __match_features(self, descriptors1, descriptors2):
        ''' Match features between two sets of decriptors '''
        '''use brute force matching and apply ratio test to prune bad matches'''
        matcher = cv2.BFMatcher()
        matches = matcher.knnMatch(descriptors2, descriptors1, k=2)
        good = [m for pair in matches if len(pair) == 2
                for m, n in [pair] if m.distance < 0.55 * n.distance] 
        # this block is literally the same with the for if if block
        return good


    def __estimate_transform(self, kp1, kp2, matches):
        ''' 
        try affine transform first, then homography if affine fail
        returns (A, H) A is affine transform, H is homography
        where one of them is None depending on which one is successful
        '''
        src_pts = np.float32([kp2[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp1[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # for inlier detection and checker, dont throw the inlier mask away
        # the right block used to be empty, but we need to return the inlier mask for further processing
        A, inlier_mask = cv2.estimateAffinePartial2D(src_pts, dst_pts)
        if A is not None:
            return A, None, inlier_mask, src_pts, dst_pts
        H, inlier_mask = cv2.findHomography(src_pts, dst_pts, method=cv2.RANSAC)
        return None, H, inlier_mask, src_pts, dst_pts
    
    def __compute_canvas_bounds(self, shape1, shape2, A, H):
        ''' Return (xMin, yMin, xMax, yMax) of the canvas needed to fit both images after transformation '''
        height1, width1 = shape1[:2]
        height2, width2 = shape2[:2]

        corners1 = np.float32([[0, 0], [0, height1], [width1, height1], [width1, 0]])
        corners2 = np.float32([[0, 0], [0, height2], [width2, height2], [width2, 0]])

        if A is not None:
            # affine: [x', y'] = A * [x, y, 1]^T
            ones = np.ones((4, 1))
            pts = np.hstack([corners2, ones])  # (4, 3)
            warped_corners2 = pts @ A.T  # (4, 2)
        else:
            # perspective
            warped_corners2 = cv2.perspectiveTransform(corners2.reshape(-1, 1, 2), H).reshape(-1, 2)
        
        all_corners = np.vstack([corners1, warped_corners2])
        xMin, yMin = np.int32(all_corners.min(axis=0).ravel() - 0.5)
        xMax, yMax = np.int32(all_corners.max(axis=0).ravel() + 0.5)
        return xMin, yMin, xMax, yMax
    
    def __predict_roi(self, h_rel_prev, image_shape):
        ''' 
        Predict the ROI in the new image based on the previous homography and the image shape.
        This is a simple heuristic that assumes the camera motion is mostly forward and the scene is mostly planar.
        '''
        height, width = image_shape[:2]
        corners = np.array([
            [0, 0],
            [0, height],
            [width, height],
            [width, 0]
        ], dtype=np.float32).reshape(-1, 1, 2)

        _, H_inv = cv2.invert(h_rel_prev)
        warped_corners = cv2.perspectiveTransform(corners, H_inv)
        x_min, y_min = np.int32(warped_corners.min(axis=0).ravel() - 0.5)
        x_max, y_max = np.int32(warped_corners.max(axis=0).ravel() + 0.5)
        
        # margin of tolerance
        pad_x = int(0.15 * width)
        pad_y = int(0.15* height)
        x_start = max(x_min - pad_x, 0)
        y_start = max(y_min - pad_y, 0)
        x_end = min(x_max + pad_x, width)
        y_end = min(y_max + pad_y, height)

        # FALLBACK 
        if (x_end - x_start < 100) or (y_end - y_start < 100):
            x_start, y_start, x_end, y_end = 0, 0, width, height
        return x_start, y_start, x_end, y_end
    
    def _warp_images(self, result_image, image2, A, H, xMin, yMin, xMax, yMax):
        ''' 
        warp both images onto a common canvas and return them
        '''
        canvas_size = (xMax - xMin, yMax - yMin)
        translation = np.float32([
            [1, 0, -xMin],
            [0, 1, -yMin],
            [0, 0, 1]
        ])
        warped_result = cv2.warpPerspective(result_image, translation, canvas_size)

        if A is not None:
            tmp = cv2.warpPerspective(image2, translation, canvas_size)
            warped_image2 = cv2.warpAffine(tmp, A, canvas_size)
        else:
            warped_image2 = cv2.warpPerspective(image2, translation @ H, canvas_size)
        
        return warped_result, warped_image2
    
    def _legacy_blend(self, warped_result, warped_img2):
        """
        Simple binary-mask blend:
        new image fills only pixels where the existing mosaic is black.
        Replace this method to plug in a better blending strategy.
        """
        gray = cv2.cvtColor(warped_result, cv2.COLOR_BGR2GRAY)
        _, inv_mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY_INV)
        alpha = inv_mask.astype(np.float32) / 255.0
        warped_img2_float = warped_img2.astype(np.float32)

        for c in range(3):
            warped_img2_float[:, :, c] *= alpha
        warped_img2 = warped_img2_float.astype(np.uint8)

        return warped_result + warped_img2

    def __simple_fallback_transform(self, index):
        """
        Translation-only H_rel from the GPS delta, used to bridge a SHORT run
        of consecutive rejections (see MAX_BRIDGE_GAP) so the chain doesn't
        freeze indefinitely. No rotation component -- kept deliberately simple.
        """
        # Get the previous and current GPS positions
        prev_pos = self.dataMatrix[index - 1, 0:2]
        curr_pos = self.dataMatrix[index, 0:2]
        if self.dataMatrix[index - 1, 6] == 0 or self.dataMatrix[index, 6] == 0:
            print("⚠️  Warning: Invalid GPS data. Cannot compute fallback transform.")
            return None
        
        # Convert GPS delta to pixel translation using meters per pixel
        if self._meters_per_pixel is None:
            print("⚠️  Warning: Meters per pixel not calibrated. Cannot compute fallback transform.")
            return None
        
        delta_pos_m = curr_pos - prev_pos
        translation_px = delta_pos_m / self._meters_per_pixel
        translation_px[1] = -translation_px[1]  # Invert Y-axis for image coordinates

        # Create a translation matrix
        H_fallback = np.array([
            [1, 0, translation_px[0]],
            [0, 1, translation_px[1]],
            [0, 0, 1]
        ], dtype=np.float32)

        return H_fallback

    def __bridge_or_freeze(self, index, image2_shape):
        """
        Called on every rejection. Only bridges the chain (advances
        H_global_prev via GPS translation, no paint) while the run of
        consecutive rejections is short (<= MAX_BRIDGE_GAP); beyond that it
        freezes (does nothing) rather than compounding approximations over
        an uncertain, growing gap.
        """
        self._consecutive_rejections += 1
        if self._consecutive_rejections <= self.MAX_BRIDGE_GAP:
            fallback = self.__simple_fallback_transform(index)
            if fallback is not None:
                self.__advance_global_position(fallback, image2_shape, will_paint=False)

    def __advance_global_position(self, H_rel_3x3, image2_shape, will_paint=True):
        """
        Chain H_rel_3x3 onto H_global_prev and re-base the canvas origin.
        Only called on the accepted-frame path -- rejected frames leave
        H_global_prev untouched (frozen), matching the simpler pre-fallback
        behavior. An earlier version kept the chain moving on rejected
        frames via a GPS/yaw-derived approximate transform, but that
        introduced small per-hop errors that compounded into visible
        ghosting throughout the mosaic -- worse than the occasional
        position jump this simpler freeze can cause after a long run of
        rejections.
        """
        self.H_rel_prev = H_rel_3x3
        H_global_current = np.dot(self.H_global_prev, H_rel_3x3)
        H_global_current = H_global_current / H_global_current[2, 2]

        if not will_paint:
            self.H_global_prev = H_global_current
            return H_global_current, None, None, None, None

        xMin, yMin, xMax, yMax = self.__compute_canvas_bounds(
            self.result_image.shape, image2_shape, None, H_global_current)
        translation = np.float32([
            [1, 0, -xMin],
            [0, 1, -yMin],
            [0, 0, 1]
        ])
        self.H_global_prev = np.dot(translation, H_global_current)
        return H_global_current, xMin, yMin, xMax, yMax

    def __check_gps_translation_sanity(self, index, H_rel_3x3, image_shape):
        """
        Check if the estimated translation from H_rel_3x3 is consistent with the GPS translation in dataMatrix.
        If the difference is too large, return False. Otherwise, return True.
        """
        meters_per_pixel = self._meters_per_pixel
        TRANSLATION_TOLERANCE_M = 10.0 # in meters
        tolerance_px = TRANSLATION_TOLERANCE_M / meters_per_pixel

        # compute translation from the center of the image
        h, w = image_shape[:2]
        center = np.array([w/2, h/2, 1.0])
        mapped_center = H_rel_3x3 @ center
        mapped_center = mapped_center / mapped_center[2]  # normalize

        # translation_estimated_px = H_rel_3x3[0:2, 2]
        translation_estimated_px = mapped_center[:2] - np.array([w/2, h/2])
        translation_actual_m = self.dataMatrix[index, 0:2] - self.dataMatrix[index-1, 0:2]
        translation_actual_m[1] = -translation_actual_m[1]
        translation_actual_px = translation_actual_m / meters_per_pixel

        translation_error_px = np.linalg.norm(translation_estimated_px - translation_actual_px)
        if translation_error_px > tolerance_px:
            print(f"⚠️  Warning: Estimated translation {translation_estimated_px} is too far from actual translation {translation_actual_px}.")
            return False
        return True
    
    def __check_attitude_drift(self, index):
        """
        Check if the estimated roll/pitch from the attitude data is within reasonable bounds.
        If the roll/pitch is too large, return False. Otherwise, return True.
        """
        ATTITUDE_DRIFT_TOLERANCE_DEG = 7.8 # initial: 7.5
        roll_delta = abs(self.dataMatrix[index, 5] - self.dataMatrix[self.last_acc_index, 5])
        pitch_delta = abs(self.dataMatrix[index, 4] - self.dataMatrix[self.last_acc_index, 4])
        drift = max(roll_delta, pitch_delta)

        if drift > ATTITUDE_DRIFT_TOLERANCE_DEG:
            print(f"⚠️  Warning: Attitude drift {drift:.2f}° exceeds tolerance of {ATTITUDE_DRIFT_TOLERANCE_DEG}° from last accepted frame at index {self.last_acc_index}. Skipping")
            return False
        return True

    # ------------------------------------------------------------------ #
    #  PUBLIC FUNCTIONs                                                  #
    # ------------------------------------------------------------------ #

    def combine(self, index):
        """Stitch image[index] into the running mosaic."""

        # Attempt to combine one pair of images at each step. Assume the
        # order in which the images are given is the best order.
        image1 = self.image_list[index-1].copy()
        image2 = self.image_list[index].copy()

        if not self.__check_attitude_drift(index):
            self.__bridge_or_freeze(index, image2.shape)
            return self.result_image

        # --- ADAPTIVE ROI PREDICTION (towards initial images to incoming images)--- #
        if self.H_rel_prev is not None:
            x_start, y_start, x_end, y_end = self.__predict_roi(self.H_rel_prev, image2.shape)
            image2_roi = image2[y_start:y_end, x_start:x_end]
            is_cropped = True
            print(f"Predicted ROI for image {index}: ({x_start}, {y_start}) to ({x_end}, {y_end})")
        else:
            image2_roi = image2
            is_cropped = False
            print(f"No valid homography from previous step. Using full image for image {index}.")

        # --- feature detection --- #
        t = time.time()
        kp1, descriptors1 = self.__detect_features(image1)
        kp2_roi, descriptors2 = self.__detect_features(image2_roi)

        # revert keypoint coordinates to original full image space if cropped
        kp2 = []
        if is_cropped:
            for kp in kp2_roi:
                new_kp = cv2.KeyPoint(
                    x=kp.pt[0] + x_start, 
                    y=kp.pt[1] + y_start, 
                    size=kp.size, 
                    angle=kp.angle, 
                    response=kp.response, 
                    octave=kp.octave, 
                    class_id=kp.class_id
                )
                kp2.append(new_kp)
        else:
            kp2 = kp2_roi
        elapsed_fd = time.time() - t
        self.timing_stats['feature_detection'] += elapsed_fd
        print(f"⏱️  Feature Detection: {elapsed_fd:.3f}s ({len(kp1)} + {len(kp2)} keypoints)")

        # check if descriptors were found
        if descriptors1 is None or descriptors2 is None:
            print(f"⚠️  Warning: No features detected in image pair {index-1}-{index}. Skipping.")
            self.__bridge_or_freeze(index, image2.shape)
            return self.result_image

        # --- feature matching --- #
        t = time.time()
        matches = self.__match_features(descriptors1, descriptors2)
        elapsed_match = time.time() - t
        self.timing_stats['matching'] += elapsed_match
        print(f"⏱️  Feature Matching: {elapsed_match:.3f}s ({len(matches)} good matches)")
        if len(matches) < 4:
            print(f"⚠️  Warning: Only {len(matches)} matches found for image pair {index-1}-{index}. Need at least 4. Skipping.")
            self.__bridge_or_freeze(index, image2.shape)
            return self.result_image
        
        # --- transformation estimation --- #
        t = time.time()
        A_rel, H_rel, inlier_mask, _, _ = self.__estimate_transform(kp1, kp2, matches)
        elapsed_tf = time.time() - t
        self.timing_stats['transformation'] += elapsed_tf
        print(f"⏱️  Transformation Estimation: {elapsed_tf:.3f}s")

        if A_rel is None and H_rel is None:
            print(f"⚠️  Warning: Could not compute transformation for image pair {index-1}-{index}. Skipping.")
            self.__bridge_or_freeze(index, image2.shape)
            return self.result_image

        # inlier ratio check
        inlier_ratio = inlier_mask.sum() / len(matches) if inlier_mask is not None else 0.0
        if inlier_ratio < 0.6:
            print(f"⚠️  Warning: Low inlier ratio ({inlier_ratio:.2f}) for image pair {index-1}-{index}. Skipping.")
            self.__bridge_or_freeze(index, image2.shape)
            return self.result_image

        # --- chained / accumulated homography ---
        if A_rel is not None:
            H_rel_3x3 = np.vstack([A_rel, [0, 0, 1]])
        else:
            H_rel_3x3 = H_rel

        if self._meters_per_pixel is None:
            gps_delta_m = self.dataMatrix[index, 0:2] - self.dataMatrix[index-1, 0:2]
            gps_dist_m = np.linalg.norm(gps_delta_m)
            if inlier_ratio > 0.70 and gps_dist_m > 2.0:
                # center-anchor translation, same like gps sanity check
                h, w = image2.shape[:2]
                center = np.array([w/2, h/2, 1.0])
                mapped_center = H_rel_3x3 @ center
                mapped_center = mapped_center / mapped_center[2]  # normalize
                est_px = mapped_center[:2] - np.array([w/2, h/2])
                est_dist_px = np.linalg.norm(est_px)
                sample_scale = est_dist_px / gps_dist_m
                self._mpp_calibration_samples.append(sample_scale)
                if len(self._mpp_calibration_samples) >= 5:
                    self._meters_per_pixel = 1.0 / np.median(self._mpp_calibration_samples)
                    print(f"✅  Meters per pixel calibrated: {self._meters_per_pixel:.6f} m/px")

        # sanity check: if corresponding translation of H rel 3x3 compared to
        # dataMatrix[index, 0:2] - dataMatrix[index-1, 0:2] is too large, skip this image
        if self._meters_per_pixel is not None:
            if not self.__check_gps_translation_sanity(index, H_rel_3x3, image2.shape):
                self.__bridge_or_freeze(index, image2.shape)
                return self.result_image

        # --- end of transformation estimation --- #
        self._consecutive_rejections = 0
        self.last_acc_index = index

        # --- warping --- #
        t = time.time()
        H_global_current, xMin, yMin, xMax, yMax = self.__advance_global_position(H_rel_3x3, image2.shape)
        warped_result, warped_image2 = self._warp_images(self.result_image, image2, None, H_global_current, xMin, yMin, xMax, yMax)
        elapsed_warp = time.time() - t
        self.timing_stats['warping'] += elapsed_warp
        print(f"⏱️  Warping: {elapsed_warp:.3f}s")

        # --- blending --- #
        t = time.time()
        self.result_image = ROIfeatherBlender._roi_feather_blend(warped_result, warped_image2)
        elapsed_blend = time.time() - t
        self.timing_stats['blending'] += elapsed_blend
        print(f"⏱️  Blending: {elapsed_blend:.3f}s")

        inter_out_path = os.path.join(self.output_dir, f"intermediateResult_{index}.png")
        cv2.imwrite(inter_out_path, self.result_image)
        print(f"Intermediate result saved: {inter_out_path}")

        # H_global_prev already advanced inside __advance_global_position() above

        # --- visualize matches --- #
        match_drawing = cv2.drawMatches(
            image2, kp2, image1, kp1, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        match_output_path = os.path.join(self.match_output_dir, f"matches_{index-1}_{index}.jpg")
        cv2.imwrite(match_output_path, match_drawing)
        print(f"Match visualization saved: {match_output_path}")
    
    def create_mosaic(self):
        '''
        stitch all image sequentially and return the final mosaic
        '''
        t0 = time.time()
        for i in range(1,len(self.image_list)):
            print(f"\n{'='*60}")
            print(f"stitching image {i} of {len(self.image_list)-1}")
            print(f"{'='*60}")
            self.combine(i)
        self.timing_stats['total'] = time.time() - t0
        self._print_timing_summary()
        return self.result_image
    
    def _print_timing_summary(self):
        s = self.timing_stats
        tracked_time = s['preprocessing'] + s['feature_detection'] + s['matching'] + s['transformation'] + s['warping'] + s['blending']
        untracked_time = s['total'] - tracked_time
        lines = [
            "\nTIMING SUMMARY",
            "=" * 55,
            f"Preprocessing:      {s['preprocessing']:>8.2f}s",
            f"Feature Detection:  {s['feature_detection']:>8.2f}s",
            f"Feature Matching:   {s['matching']:>8.2f}s",
            f"Transformation:     {s['transformation']:>8.2f}s",
            f"Warping:            {s['warping']:>8.2f}s",
            f"Blending:           {s['blending']:>8.2f}s",
            f"I/O & Misc (Save):  {untracked_time:>8.2f}s", 
            "-" * 55,
            f"TOTAL:              {s['total']:>8.2f}s",
        ]
        print("\n".join(lines))

        stats_path = os.path.join(self.output_dir, "timing_stats.txt")
        with open(stats_path, 'w') as f:
            f.write("\n".join(lines))
        print(f"Stats saved: {stats_path}")
```
