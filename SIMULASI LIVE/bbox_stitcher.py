# bbox_stitcher.py
# Engine incremental stitching dengan modul proyeksi Homography Bounding Box

import cv2
import numpy as np

class BBoxStitcher:
    """
    Stitching engine yang menggabungkan gambar secara berurutan (incremental)
    dan memproyeksikan koordinat Bounding Box dari frame lokal ke canvas stitched global.
    """
    def __init__(self, downsample=2.0):
        self.downsample = downsample
        self.panorama = None
        self.accumulated_H = np.eye(3, dtype=np.float64)
        self.all_bboxes = []  # Menyimpan seluruh bbox yang sudah diproyeksikan ke canvas
        
        # Inisialisasi SIFT detector
        self.sift = cv2.SIFT_create()
        
        # FLANN Matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def _resize(self, img):
        if self.downsample > 1.0:
            h, w = img.shape[:2]
            new_w = int(w / self.downsample)
            new_h = int(h / self.downsample)
            return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return img.copy()

    def transform_bbox_corners(self, bbox, H, img_scale_factor=1.0):
        """
        Mentransformasikan 4 titik sudut Bounding Box [x1, y1, x2, y2]
        ke koordinat canvas menggunakan matriks Homography H.
        """
        x1, y1, x2, y2 = bbox
        
        # Skalakan bbox jika gambar disesuaikan ukurannya (downsample)
        x1 *= img_scale_factor
        y1 *= img_scale_factor
        x2 *= img_scale_factor
        y2 *= img_scale_factor

        # 4 titik sudut persegi (top-left, top-right, bottom-right, bottom-left)
        corners = np.array([
            [[x1, y1]],
            [[x2, y1]],
            [[x2, y2]],
            [[x1, y2]]
        ], dtype=np.float32)

        # Proyeksi perspektif menggunakan H: P_canvas = H * P_local
        transformed_corners = cv2.perspectiveTransform(corners, H)
        return transformed_corners.reshape(-1, 2)

    def add_frame(self, img, bboxes=None, gps_data=None):
        """
        Menambahkan frame baru, menghitung Homography, me-warp gambar ke canvas,
        dan memproyeksikan Bounding Box ke canvas panorama.
        """
        if bboxes is None:
            bboxes = []
        if gps_data is None:
            gps_data = {}

        img_resized = self._resize(img)
        scale_factor = img_resized.shape[1] / img.shape[1]

        # 1. Jika ini frame pertama
        if self.panorama is None:
            self.panorama = img_resized.copy()
            self.last_img = img_resized.copy()
            
            # Ekstrak SIFT pada frame 1
            self.last_kp, self.last_des = self.sift.detectAndCompute(self.last_img, None)
            
            # BBox pada frame 1 berada pada koordinat canvas langsung (identitas H)
            for b in bboxes:
                pts = self.transform_bbox_corners(b['bbox'], np.eye(3), scale_factor)
                self.all_bboxes.append({
                    'corners': pts,
                    'label': b.get('label', 'target'),
                    'confidence': b.get('confidence', 1.0),
                    'lat': b.get('lat', gps_data.get('latitude', 0.0)),
                    'lon': b.get('lon', gps_data.get('longitude', 0.0))
                })
            
            return self.draw_overlays(self.panorama)

        # 2. Frame berikutnya: Ekstrak & Match SIFT features
        kp_curr, des_curr = self.sift.detectAndCompute(img_resized, None)
        
        if des_curr is None or self.last_des is None or len(des_curr) < 4 or len(self.last_des) < 4:
            print("[STITCHER] Warning: Fitur tidak cukup untuk matching.")
            return self.draw_overlays(self.panorama)

        matches = self.flann.knnMatch(self.last_des, des_curr, k=2)
        
        # Lowe's ratio test
        good_matches = []
        for m_tuple in matches:
            if len(m_tuple) == 2:
                m, n = m_tuple
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        if len(good_matches) < 8:
            print(f"[STITCHER] Warning: Match terlalu sedikit ({len(good_matches)}). Frame dilewati.")
            return self.draw_overlays(self.panorama)

        pts_prev = np.float32([self.last_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        pts_curr = np.float32([kp_curr[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Hitung Homography H antara frame baru -> frame sebelumnya
        H_curr_to_prev, mask = cv2.findHomography(pts_curr, pts_prev, cv2.RANSAC, 5.0)

        if H_curr_to_prev is None:
            print("[STITCHER] Warning: Gagal menghitung Homography.")
            return self.draw_overlays(self.panorama)

        # Akumulasikan matriks Homography relatif terhadap canvas utama
        self.accumulated_H = self.accumulated_H @ H_curr_to_prev

        # 3. Hitung canvas ekspansi & warp gambar
        h_curr, w_curr = img_resized.shape[:2]
        corners_curr = np.array([
            [[0, 0]], [[w_curr, 0]], [[w_curr, h_curr]], [[0, h_curr]]
        ], dtype=np.float32)
        
        warped_corners = cv2.perspectiveTransform(corners_curr, self.accumulated_H).reshape(-1, 2)

        # Hitung batas canvas baru
        h_pan, w_pan = self.panorama.shape[:2]
        all_corners = np.vstack(([0, 0], [w_pan, 0], [w_pan, h_pan], [0, h_pan], warped_corners))

        x_min, y_min = np.int32(all_corners.min(axis=0) - 0.5)
        x_max, y_max = np.int32(all_corners.max(axis=0) + 0.5)

        # Matriks translasi jika offset minus
        translation = np.array([
            [1, 0, -x_min],
            [0, 1, -y_min],
            [0, 0, 1]
        ], dtype=np.float64)

        # Canvas panorama baru
        new_w = x_max - x_min
        new_h = y_max - y_min
        panorama_large = np.zeros((new_h, new_w, 3), dtype=np.uint8)

        # Tempelkan panorama lama ke lokasi offset
        panorama_large[-y_min:-y_min+h_pan, -x_min:-x_min+w_pan] = self.panorama

        # Warp gambar baru ke canvas panorama
        H_final = translation @ self.accumulated_H
        warped_img = cv2.warpPerspective(img_resized, H_final, (new_w, new_h))

        # Blending sederhana (overlay di atas area non-hitam)
        mask_warped = (warped_img > 0)
        panorama_large[mask_warped] = warped_img[mask_warped]

        # Update state
        self.panorama = panorama_large
        self.accumulated_H = H_final
        self.last_img = img_resized
        self.last_kp = kp_curr
        self.last_des = des_curr

        # Adjust posisi titik-titik bbox lama akibat pergeseran canvas (translation)
        for b in self.all_bboxes:
            b['corners'] += np.array([-x_min, -y_min])

        # Proyeksi Bounding Box frame baru ke canvas akhir
        for b in bboxes:
            pts = self.transform_bbox_corners(b['bbox'], H_final, scale_factor)
            self.all_bboxes.append({
                'corners': pts,
                'label': b.get('label', 'target'),
                'confidence': b.get('confidence', 1.0),
                'lat': b.get('lat', gps_data.get('latitude', 0.0)),
                'lon': b.get('lon', gps_data.get('longitude', 0.0))
            })

        return self.draw_overlays(self.panorama)

    def draw_overlays(self, img_canvas):
        """
        Menggambar polygon/bounding box dan metadata GPS pada canvas hasil stitching.
        """
        result = img_canvas.copy()
        
        for b in self.all_bboxes:
            pts = np.int32(b['corners'])
            
            # Gambar kontur bounding box (polygon)
            cv2.polylines(result, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Tentukan posisi teks di sudut kiri atas bounding box
            top_left = pts[0]
            label_text = f"{b['label']} ({b['confidence']:.2f})"
            gps_text = f"Lat:{b['lat']:.5f}, Lon:{b['lon']:.5f}"

            # Frame latar belakang teks (biar terbaca jelas)
            cv2.putText(result, label_text, (top_left[0], max(15, top_left[1] - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(result, gps_text, (top_left[0], max(35, top_left[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        return result
