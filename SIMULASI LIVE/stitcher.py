# stitcher.py
# Self-contained stitching module untuk simulasi live stitching
# Mengambil logika inti dari src/Combiner.py, src/blending.py, src/geometry.py

from numpy import int16
import cv2
import numpy as np
import os
import time
import math as m


# ================================================================
#  GEOMETRY (dari src/geometry.py)
# ================================================================

def compute_unrot_matrix(pose):
    """
    Menghitung matriks rotasi inverse untuk membatalkan rotasi kamera.
    :param pose: Array 1x6 [X, Y, Z, Yaw, Pitch, Roll]
    :return: Matriks 3x3 inverse rotation
    """
    a = pose[3] * np.pi / 180  # alpha (Yaw)
    b = pose[4] * np.pi / 180  # beta  (Pitch)
    g = pose[5] * np.pi / 180  # gamma (Roll)

    Rz = np.array(([m.cos(a), -m.sin(a), 0],
                   [m.sin(a),  m.cos(a), 0],
                   [0,         0,        1]))

    Ry = np.array(([ m.cos(b), 0, m.sin(b)],
                   [ 0,        1, 0       ],
                   [-m.sin(b), 0, m.cos(b)]))

    Rx = np.array(([1, 0,         0        ],
                   [0, m.cos(g), -m.sin(g) ],
                   [0, m.sin(g),  m.cos(g) ]))

    Ryx = np.dot(Rx, Ry)
    R = np.dot(Rz, Ryx)
    R[0, 2] = 0
    R[1, 2] = 0
    R[2, 2] = 1
    Rtrans = R.transpose()
    InvR = np.linalg.inv(Rtrans)
    return InvR
    

def warp_perspective_with_padding(image, transformation):
    """
    Warp image dengan padding agar seluruh hasil transformasi terlihat.
    """
    height, width = image.shape[:2]
    corners = np.float32([[0, 0], [0, height], [width, height], [width, 0]]).reshape(-1, 1, 2)
    warped_corners = cv2.perspectiveTransform(corners, transformation)
    [x_min, y_min] = np.int32(warped_corners.min(axis=0).ravel() - 0.5)
    [x_max, y_max] = np.int32(warped_corners.max(axis=0).ravel() + 0.5)
    translation = np.array(([1, 0, -x_min], [0, 1, -y_min], [0, 0, 1]), dtype=np.float64)
    full_transformation = np.dot(translation, transformation)
    result = cv2.warpPerspective(image, full_transformation, (x_max - x_min, y_max - y_min))
    return result


# ================================================================
#  BLENDING (dari src/blending.py — ROIfeatherBlender)
# ================================================================

def roi_feather_blend(warped_result, warped_img2):
    """
    Feather blending pada Region of Interest (overlap area) saja.
    """
    gray1 = cv2.cvtColor(warped_result, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(warped_img2, cv2.COLOR_BGR2GRAY)

    mask1 = cv2.threshold(gray1, 1, 255, cv2.THRESH_BINARY)[1]
    mask2 = cv2.threshold(gray2, 1, 255, cv2.THRESH_BINARY)[1]

    overlap_mask = cv2.bitwise_and(mask1, mask2)

    # Isi piksel yang hanya ada di img2 (non-overlap)
    only_img2 = cv2.bitwise_and(mask2, cv2.bitwise_not(mask1))
    warped_result[only_img2 > 0] = warped_img2[only_img2 > 0]

    coords = cv2.findNonZero(overlap_mask)
    if coords is None:
        return warped_result  # tidak ada overlap

    x, y, w, h = cv2.boundingRect(coords)

    # Potong ROI
    roi_img1 = warped_result[y:y+h, x:x+w]
    roi_img2 = warped_img2[y:y+h, x:x+w]
    roi_mask1 = mask1[y:y+h, x:x+w]
    roi_mask2 = mask2[y:y+h, x:x+w]
    roi_overlap = overlap_mask[y:y+h, x:x+w]

    dist1 = cv2.distanceTransform(roi_mask1, cv2.DIST_L2, 3)
    dist2 = cv2.distanceTransform(roi_mask2, cv2.DIST_L2, 3)

    alpha = dist1 / (dist1 + dist2 + 1e-6)
    alpha = alpha ** 3
    alpha = cv2.merge([alpha, alpha, alpha])

    # Alpha blending
    img1_f = roi_img1.astype(np.float32)
    img2_f = roi_img2.astype(np.float32)
    blended_roi = img1_f * alpha + img2_f * (1 - alpha)
    bool_overlap = roi_overlap > 0
    roi_result = roi_img1.copy()
    roi_result[bool_overlap] = blended_roi[bool_overlap].astype(np.uint8)
    warped_result[y:y+h, x:x+w] = roi_result

    return warped_result


# ================================================================
#  LIVE STITCHER — Incremental stitching engine
# ================================================================

class LiveStitcher:
    """
    Engine untuk incremental stitching.
    
    Alur:
        1. Gambar pertama → jadi mosaic awal
        2. Gambar ke-N → stitch ke mosaic → hasilnya jadi mosaic baru
    """

    def __init__(self, output_dir="output", downsample_factor=5, stitch_start_threshold=20):
        self.output_dir = output_dir
        self.match_output_dir = os.path.join(output_dir, "matches")
        self.downsample_factor = downsample_factor
        self.stitch_start_threshold = stitch_start_threshold
        
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.match_output_dir, exist_ok=True)
        
        # State stitching
        self.mosaic = None           # Mosaic yang sedang berjalan
        self.H_global = np.eye(3, dtype=np.float32)  # Chained homography
        self.stitch_count = 0        # Jumlah stitch yang berhasil
        self.image_count = 0         # Jumlah gambar yang diterima
        
        # Timing stats
        self.timing_stats = {
            'preprocessing': 0,
            'feature_detection': 0,
            'matching': 0,
            'transformation': 0,
            'warping': 0,
            'blending': 0,
        }

    def preprocess(self, img, pose=None):
        """
        Downsample dan unrotate gambar.
        :param img: Gambar BGR (numpy array)
        :param pose: [X,Y,Z,Yaw,Pitch,Roll] — None = tanpa unrotate
        """
        t0 = time.time()
        
        # Downsample
        ds = self.downsample_factor
        downsampled = img[::ds, ::ds]
        
        # Unrotate jika ada pose data
        if pose is not None:
            M = compute_unrot_matrix(pose)
            result = warp_perspective_with_padding(downsampled, M)
        else:
            # Default pose = [0,0,0,0,0,0] → identity transform
            default_pose = np.zeros(6)
            M = compute_unrot_matrix(default_pose)
            result = warp_perspective_with_padding(downsampled, M)
        
        self.timing_stats['preprocessing'] += time.time() - t0
        return result

    def _detect_features(self, image):
        """Deteksi fitur SIFT pada gambar."""
        detector = cv2.SIFT_create(1800)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        kp, descriptors = detector.detectAndCompute(gray, mask)
        return kp, descriptors

    def _match_features(self, desc1, desc2):
        """Match fitur antara dua deskriptor menggunakan BFMatcher + ratio test."""
        matcher = cv2.BFMatcher()
        matches = matcher.knnMatch(desc2, desc1, k=2)
        good = [m for pair in matches if len(pair) == 2
                for m, n in [pair] if m.distance < 0.55 * n.distance]
        return good

    def _estimate_transform(self, kp1, kp2, matches):
        """
        Estimasi transformasi: coba affine dulu, fallback ke homography.
        Return (A, H) di mana salah satu None.
        """
        src_pts = np.float32([kp2[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp1[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        A, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
        if A is not None:
            return A, None
        H, _ = cv2.findHomography(src_pts, dst_pts, method=cv2.RANSAC)
        return None, H

    def _compute_canvas_bounds(self, shape1, shape2, H):
        """Hitung batas canvas yang diperlukan untuk menampung kedua gambar setelah transformasi."""
        h1, w1 = shape1[:2]
        h2, w2 = shape2[:2]

        corners1 = np.float32([[0, 0], [0, h1], [w1, h1], [w1, 0]])
        corners2 = np.float32([[0, 0], [0, h2], [w2, h2], [w2, 0]])

        warped_corners2 = cv2.perspectiveTransform(corners2.reshape(-1, 1, 2), H).reshape(-1, 2)

        all_corners = np.vstack([corners1, warped_corners2])
        xMin, yMin = np.int32(all_corners.min(axis=0).ravel() - 0.5)
        xMax, yMax = np.int32(all_corners.max(axis=0).ravel() + 0.5)
        return xMin, yMin, xMax, yMax

    def _warp_images(self, result_image, image2, H, xMin, yMin, xMax, yMax):
        """Warp kedua gambar ke canvas yang sama."""
        canvas_size = (xMax - xMin, yMax - yMin)
        translation = np.float32([
            [1, 0, -xMin],
            [0, 1, -yMin],
            [0, 0, 1]
        ])
        warped_result = cv2.warpPerspective(result_image, translation, canvas_size)
        warped_image2 = cv2.warpPerspective(image2, translation @ H, canvas_size)
        return warped_result, warped_image2

    def add_image(self, raw_image, pose=None):
        """
        Tambahkan satu gambar baru ke mosaic.
        
        :param raw_image: Gambar BGR (numpy array, ukuran penuh)
        :param pose: [X,Y,Z,Yaw,Pitch,Roll] — opsional
        :return: (success: bool, stitch_path: str atau None)
        """
        self.image_count += 1
        img_num = self.image_count
        
        # Preprocessing
        processed = self.preprocess(raw_image, pose)

        # Mulai stitching hanya jika jumlah gambar yang masuk sudah melewati threshold.
        # Contoh threshold=20: gambar #1..#20 hanya menunggu, gambar #21 jadi mosaic awal.
        if img_num <= self.stitch_start_threshold:
            print(
                f"  [STITCH] Gambar #{img_num}: menunggu threshold "
                f"(>{self.stitch_start_threshold})"
            )
            return True, None
        
        # Gambar pertama setelah melewati threshold → jadi mosaic awal
        if self.mosaic is None:
            self.mosaic = processed
            print(
                f"  [STITCH] Gambar #{img_num}: threshold terlewati, "
                f"dijadikan mosaic awal ({processed.shape[1]}x{processed.shape[0]})"
            )
            return True, None

        print(f"\n{'='*60}")
        print(f"  [STITCH] Menggabungkan gambar #{img_num} ke mosaic...")
        print(f"{'='*60}")

        # --- Feature detection ---
        t = time.time()
        kp1, desc1 = self._detect_features(self.mosaic)
        kp2, desc2 = self._detect_features(processed)
        dt = time.time() - t
        self.timing_stats['feature_detection'] += dt
        print(f"  ⏱️  Feature Detection: {dt:.3f}s ({len(kp1)} + {len(kp2)} keypoints)")

        if desc1 is None or desc2 is None:
            print(f"  ⚠️  Tidak ada fitur terdeteksi. Skip.")
            return False, None

        # --- Feature matching ---
        t = time.time()
        matches = self._match_features(desc1, desc2)
        dt = time.time() - t
        self.timing_stats['matching'] += dt
        print(f"  ⏱️  Feature Matching: {dt:.3f}s ({len(matches)} good matches)")

        if len(matches) < 4:
            print(f"  ⚠️  Hanya {len(matches)} match ditemukan (min 4). Skip.")
            return False, None

        # --- Transform estimation ---
        t = time.time()
        A_rel, H_rel = self._estimate_transform(kp1, kp2, matches)
        dt = time.time() - t
        self.timing_stats['transformation'] += dt
        print(f"  ⏱️  Transform Estimation: {dt:.3f}s")

        if A_rel is None and H_rel is None:
            print(f"  ⚠️  Tidak bisa menghitung transformasi. Skip.")
            return False, None

        # Chained homography
        if A_rel is not None:
            H_rel_3x3 = np.vstack([A_rel, [0, 0, 1]])
        else:
            H_rel_3x3 = H_rel

        H_global_current = np.dot(self.H_global, H_rel_3x3)
        H_global_current = H_global_current / H_global_current[2, 2]

        # --- Warping ---
        t = time.time()
        xMin, yMin, xMax, yMax = self._compute_canvas_bounds(
            self.mosaic.shape, processed.shape, H_global_current
        )
        warped_mosaic, warped_new = self._warp_images(
            self.mosaic, processed, H_global_current, xMin, yMin, xMax, yMax
        )
        dt = time.time() - t
        self.timing_stats['warping'] += dt
        print(f"  ⏱️  Warping: {dt:.3f}s")

        # --- Blending ---
        t = time.time()
        self.mosaic = roi_feather_blend(warped_mosaic, warped_new)
        dt = time.time() - t
        self.timing_stats['blending'] += dt
        print(f"  ⏱️  Blending: {dt:.3f}s")

        # Update global homography
        translation = np.float32([
            [1, 0, -xMin],
            [0, 1, -yMin],
            [0, 0, 1]
        ])
        self.H_global = np.dot(translation, H_global_current)

        # Simpan hasil intermediate
        self.stitch_count += 1
        stitch_path = os.path.join(self.output_dir, f"stitch_{self.stitch_count:04d}.png")
        cv2.imwrite(stitch_path, self.mosaic)
        print(f"  ✓ Hasil stitch disimpan: {stitch_path}")
        print(f"    Ukuran mosaic: {self.mosaic.shape[1]}x{self.mosaic.shape[0]}")

        # Simpan visualisasi matches
        try:
            match_drawing = cv2.drawMatches(
                processed, kp2, self.mosaic, kp1, matches[:50], None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            match_path = os.path.join(self.match_output_dir, f"matches_{self.stitch_count:04d}.jpg")
            cv2.imwrite(match_path, match_drawing)
        except Exception:
            pass  # Visualisasi match opsional

        return True, stitch_path

    def get_mosaic(self):
        """Ambil mosaic terkini."""
        return self.mosaic

    def print_summary(self):
        """Print ringkasan timing."""
        s = self.timing_stats
        total = sum(s.values())
        lines = [
            "",
            "=" * 55,
            "  TIMING SUMMARY",
            "=" * 55,
            f"  Preprocessing:      {s['preprocessing']:>8.2f}s",
            f"  Feature Detection:  {s['feature_detection']:>8.2f}s",
            f"  Feature Matching:   {s['matching']:>8.2f}s",
            f"  Transformation:     {s['transformation']:>8.2f}s",
            f"  Warping:            {s['warping']:>8.2f}s",
            f"  Blending:           {s['blending']:>8.2f}s",
            "-" * 55,
            f"  TOTAL:              {total:>8.2f}s",
            f"  Images received:    {self.image_count}",
            f"  Stitches completed: {self.stitch_count}",
            "=" * 55,
        ]
        print("\n".join(lines))
