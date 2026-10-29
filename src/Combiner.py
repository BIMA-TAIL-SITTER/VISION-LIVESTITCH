# src/Combiner.py
# ═══════════════════════════════════════════════════════════════════════════
#  REVISED Combiner — Anti-drift + Exposure Compensation + Auto-crop
# ═══════════════════════════════════════════════════════════════════════════
#  Perubahan utama dari versi sebelumnya:
#    1. Match terhadap MOSAIC (akumulasi), bukan gambar sebelumnya saja
#       → menghilangkan chaining homography → eliminasi drift
#    2. Exposure compensation sebelum blending
#    3. Auto-crop black borders pada mosaic akhir
#    4. Parameter tuning: downsample 5→4, SIFT 1800→2500, ratio 0.55→0.65
#    5. Overflow fix di blending (di blending.py)
#  Original backup: src/backup/Combiner_pre_revision.py
# ═══════════════════════════════════════════════════════════════════════════

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
from blending import PyramidBlender, ROIfeatherBlender, compensate_exposure

class Combiner:
    def __init__(self,imageList_,dataMatrix_, output="output",
                 downsample=4, n_features=2500, ratio_thresh=0.65):
        '''
        :param imageList_: List of all images in dataset.
        :param dataMatrix_: Matrix with all pose data in dataset.
        :param output: Output directory for results.
        :param downsample: Downsample factor for preprocessing (default 4, was 5).
        :param n_features: Number of SIFT keypoints (default 2500, was 1800).
        :param ratio_thresh: Lowe's ratio test threshold (default 0.65, was 0.55).
        :return:
        '''
        self.dataMatrix = dataMatrix_
        self.output_dir = output
        self.match_output_dir = os.path.join(self.output_dir, "matches")
        self.downsample = downsample
        self.n_features = n_features
        self.ratio_thresh = ratio_thresh
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

        # Exposure compensation: reference mean brightness dari gambar pertama
        self._ref_mean = None

        self.image_list = self.__preprocess_images(imageList_)
        self.result_image = self.image_list[0]
        self._ref_mean = float(self.result_image.mean())

    # ------------------------------------------------------------------ #
    #  PRIVATE HELPERS                                                     #
    # ------------------------------------------------------------------ #

    def __preprocess_images(self, raw_image):
        '''Downsample and unrotate all input images'''
        t0 = time.time()
        processed_images = []
        ds = self.downsample

        for i, img in enumerate(raw_image):
            downsampled = img[::ds, ::ds]  # Downsample by configurable factor
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
        print(f"Preprocessing: {self.timing_stats['preprocessing']:.3f}s "
              f"(downsample={ds}x, output ~{processed_images[0].shape[1]}x{processed_images[0].shape[0]}px)")
        return processed_images
    
    def __detect_features(self, image):
        ''' Detect features for a given image '''
        detector = cv2.SIFT_create(self.n_features)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        kp, descriptors = detector.detectAndCompute(gray, mask)
        return kp, descriptors
    
    def __match_features(self, descriptors1, descriptors2):
        ''' Match features between two sets of descriptors '''
        '''use brute force matching and apply ratio test to prune bad matches'''
        matcher = cv2.BFMatcher()
        matches = matcher.knnMatch(descriptors2, descriptors1, k=2)
        good = [m for pair in matches if len(pair) == 2
                for m, n in [pair] if m.distance < self.ratio_thresh * n.distance] 
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

        A, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
        if A is not None:
            return A, None, src_pts, dst_pts
        H, _ = cv2.findHomography(src_pts, dst_pts, method=cv2.RANSAC)
        return None, H, src_pts, dst_pts
    
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

    def _auto_crop(self, mosaic):
        """
        Remove black borders from the final mosaic.
        Detects the largest non-black contour and crops to its bounding rect.
        """
        gray = cv2.cvtColor(mosaic, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

        # Morphological close to fill small internal gaps
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            print(f"Auto-crop: origin=({x},{y}), size={w}x{h} "
                  f"(was {mosaic.shape[1]}x{mosaic.shape[0]})")
            return mosaic[y:y+h, x:x+w]
        return mosaic
    
    # ------------------------------------------------------------------ #
    #  PUBLIC FUNCTIONs                                                  #
    # ------------------------------------------------------------------ #

    def combine(self, index):
        """
        Stitch image[index] into the running mosaic.

        REVISED: Matches the new image directly against the accumulated
        mosaic (self.result_image) instead of just the previous frame.
        This eliminates homography chaining (H_global = H_prev * H_rel)
        and therefore prevents drift accumulation.

        Also applies exposure compensation before blending.
        """

        image2 = self.image_list[index].copy()

        # --- Exposure Compensation ---
        # Normalize brightness to match the first frame (mosaic reference)
        if self._ref_mean is not None:
            image2 = compensate_exposure(image2, self._ref_mean)

        # --- Feature detection on MOSAIC and NEW IMAGE ---
        t = time.time()
        kp1, descriptors1 = self.__detect_features(self.result_image)  # mosaic
        kp2, descriptors2 = self.__detect_features(image2)             # new image
        elapsed_fd = time.time() - t
        self.timing_stats['feature_detection'] += elapsed_fd
        print(f"Feature Detection: {elapsed_fd:.3f}s "
              f"({len(kp1)} mosaic + {len(kp2)} new keypoints)")

        # check if descriptors were found
        if descriptors1 is None or descriptors2 is None:
            print(f"Warning: No features detected for image {index}. Skipping.")
            return self.result_image

        # --- Feature matching ---
        t = time.time()
        matches = self.__match_features(descriptors1, descriptors2)
        elapsed_match = time.time() - t
        self.timing_stats['matching'] += elapsed_match
        print(f"Feature Matching: {elapsed_match:.3f}s ({len(matches)} good matches)")
        if len(matches) < 4:
            print(f"Warning: Only {len(matches)} matches found for image {index}. "
                  f"Need at least 4. Skipping.")
            return self.result_image
        
        # --- Transformation estimation ---
        # Transform maps image2 coords -> mosaic coords DIRECTLY.
        # No chaining needed — this is the key anti-drift change.
        t = time.time()
        A_direct, H_direct, _, _ = self.__estimate_transform(kp1, kp2, matches)
        elapsed_tf = time.time() - t
        self.timing_stats['transformation'] += elapsed_tf
        print(f"Transformation Estimation: {elapsed_tf:.3f}s")

        if A_direct is None and H_direct is None:
            print(f"Warning: Could not compute transformation for image {index}. Skipping.")
            return

        # Convert affine to 3x3 if needed
        if A_direct is not None:
            H_3x3 = np.vstack([A_direct, [0, 0, 1]]).astype(np.float32)
        else:
            H_3x3 = H_direct.astype(np.float32)

        # normalized scale
        H_3x3 = H_3x3 / H_3x3[2, 2]

        # --- Match visualization (before warp, while kp coords still valid) ---
        match_subset = matches[:40]  # cap for readability
        match_drawing = cv2.drawMatches(
            image2, kp2, self.result_image, kp1, match_subset, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        match_output_path = os.path.join(self.match_output_dir, f"matches_{index}.jpg")
        cv2.imwrite(match_output_path, match_drawing)

        # --- Warping ---
        t = time.time()
        xMin, yMin, xMax, yMax = self.__compute_canvas_bounds(
            self.result_image.shape, image2.shape, None, H_3x3
        )
        warped_result, warped_image2 = self._warp_images(
            self.result_image, image2, None, H_3x3, xMin, yMin, xMax, yMax
        )
        elapsed_warp = time.time() - t
        self.timing_stats['warping'] += elapsed_warp
        print(f"Warping: {elapsed_warp:.3f}s")

        # --- Blending ---
        t = time.time()
        self.result_image = ROIfeatherBlender._roi_feather_blend(warped_result, warped_image2)
        elapsed_blend = time.time() - t
        self.timing_stats['blending'] += elapsed_blend
        print(f"Blending: {elapsed_blend:.3f}s")

        inter_out_path = os.path.join(self.output_dir, f"intermediateResult_{index}.png")
        cv2.imwrite(inter_out_path, self.result_image)
        print(f"Intermediate result saved: {inter_out_path}")

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

        # Auto-crop black borders
        print(f"\nApplying auto-crop to remove black borders...")
        self.result_image = self._auto_crop(self.result_image)

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