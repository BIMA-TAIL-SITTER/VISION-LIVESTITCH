import cv2
import numpy as np
import sys
import os

import utilities as util
import geometry as gm
import copy
from blending import PyramidBlender

class Combiner:
    def __init__(self, imageList_, dataMatrix_, output="output", use_blending=True):
        self.imageList = []
        self.dataMatrix = dataMatrix_
        self.output_dir = output
        self.use_blending = use_blending
        
        os.makedirs(self.output_dir, exist_ok=True)

        print("[COMBINER] Preprocessing images...")
        for i in range(0, len(imageList_)):
            image = imageList_[i][::6, ::6, :]  # Downsample
            M = gm.computeUnRotMatrix(self.dataMatrix[i, :])
            correctedImage = gm.warpPerspectiveWithPadding(image, M)
            self.imageList.append(correctedImage)

            imageList_[i] = None
            if (i + 1) % 5 == 0:
                import gc
                gc.collect()
                print(f"[COMBINER] Processed {i + 1}/{len(imageList_)} images")
        
        self.resultImage = None  # Will be initialized in createMosaic methods
        
        del imageList_
        import gc
        gc.collect()
    
    def createMosaicGPS(self, gsd=0.1):
        """
        Create mosaic using GPS coordinates for global positioning with feathering
        :param gsd: Ground Sample Distance in meters/pixel (adjust based on flight altitude)
        """
        print("[GPS STITCHING] Starting GPS-based global alignment...")
        
        # Extract X, Y positions from dataMatrix (first 2 columns)
        positions = self.dataMatrix[:, :2]  # X (meters), Y (meters)
        
        # Find bounds
        min_x, min_y = positions.min(axis=0)
        max_x, max_y = positions.max(axis=0)
        
        print(f"[GPS] Coverage area: {max_x - min_x:.1f}m x {max_y - min_y:.1f}m")
        
        # Convert meters to pixels (adjusted for downsampling)
        # Original GSD, then divide by 6 for downsampling
        scale = 1.0 / (gsd * 6)  # pixels per meter at downsampled resolution
        
        pixel_positions = (positions - [min_x, min_y]) * scale
        
        # Determine canvas size
        height, width = self.imageList[0].shape[:2]
        canvas_width = int(pixel_positions[:, 0].max() + width + 200)
        canvas_height = int(pixel_positions[:, 1].max() + height + 200)
        
        print(f"[GPS] Creating canvas: {canvas_width}x{canvas_height} pixels")
        
        # Create accumulation buffers
        canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.float32)
        weight_map = np.zeros((canvas_height, canvas_width), dtype=np.float32)
        
        # Place each image with distance-based feathering
        for i, (image, pos) in enumerate(zip(self.imageList, pixel_positions)):
            print(f"[GPS] Placing image {i+1}/{len(self.imageList)}", end="\r")
            
            x, y = int(pos[0]), int(pos[1])
            h, w = image.shape[:2]
            
            # Check bounds
            if x < 0 or y < 0 or x + w > canvas_width or y + h > canvas_height:
                print(f"\n⚠️  Image {i} out of bounds ({x},{y}), adjusting...")
                x = max(0, min(x, canvas_width - w))
                y = max(0, min(y, canvas_height - h))
                if x + w > canvas_width or y + h > canvas_height:
                    continue
            
            # Create feathering weight mask
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
            
            # Distance transform for smooth feathering
            dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
            
            # Normalize and smooth
            if dist_transform.max() > 0:
                weight = dist_transform / dist_transform.max()
            else:
                weight = np.ones_like(dist_transform, dtype=np.float32)
            
            # Apply Gaussian smoothing for seamless blending
            weight = cv2.GaussianBlur(weight, (51, 51), 0)
            
            # Get canvas regions
            canvas_region = canvas[y:y+h, x:x+w]
            weight_region = weight_map[y:y+h, x:x+w]
            
            # Weighted blending
            weight_3ch = np.stack([weight, weight, weight], axis=2)
            
            canvas[y:y+h, x:x+w] = (canvas_region * weight_region[:, :, np.newaxis] + 
                                    image.astype(np.float32) * weight_3ch) / (weight_region[:, :, np.newaxis] + weight_3ch + 1e-6)
            
            weight_map[y:y+h, x:x+w] = np.maximum(weight_region, weight)
            
            # Save intermediate results periodically
            if (i + 1) % 10 == 0 or i == len(self.imageList) - 1:
                intermediate_path = os.path.join(self.output_dir, f"gps_intermediate_{i+1}.png")
                cv2.imwrite(intermediate_path, canvas.astype(np.uint8))
                print(f"\n[GPS] Intermediate saved: {intermediate_path}")
        
        print(f"\n[GPS] Mosaic complete!")
        
        # Crop to non-zero regions
        gray_canvas = cv2.cvtColor(canvas.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        coords = cv2.findNonZero(gray_canvas)
        if coords is not None:
            x, y, w, h = cv2.boundingRect(coords)
            canvas = canvas[y:y+h, x:x+w]
            print(f"[GPS] Cropped to: {w}x{h}")
        
        self.resultImage = canvas.astype(np.uint8)
        return self.resultImage
    
    def createMosaic(self):
        """
        Sequential feature-based stitching (original method)
        """
        print("[SEQUENTIAL STITCHING] Starting feature-based alignment...")
        self.resultImage = self.imageList[0]
        
        for i in range(1, len(self.imageList)):
            print(f"stitching image {i} of {len(self.imageList)-1}")
            self.combine(i)
        
        return self.resultImage

    def combine(self, index2):
        '''
        Sequential stitching: combine current result with next image
        '''
        image1 = copy.copy(self.resultImage)
        image2 = copy.copy(self.imageList[index2])

        # Increase features for better matching
        detector = cv2.SIFT_create(1000)
        
        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        ret1, mask1 = cv2.threshold(gray1, 1, 255, cv2.THRESH_BINARY)
        kp1, descriptors1 = detector.detectAndCompute(gray1, mask1)

        gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        ret2, mask2 = cv2.threshold(gray2, 1, 255, cv2.THRESH_BINARY)
        kp2, descriptors2 = detector.detectAndCompute(gray2, mask2)
        
        if descriptors1 is None or descriptors2 is None:
            print(f"⚠️  No features for image {index2}. Skipping.")
            return self.resultImage

        # Use FLANN for faster matching
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params, search_params)
        
        matches = matcher.knnMatch(descriptors2, descriptors1, k=2)
        
        # Lowe's ratio test (relaxed)
        good = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good.append(m)
        matches = good
        
        if len(matches) < 4:
            print(f"⚠️  Only {len(matches)} matches. Skipping image {index2}.")
            return self.resultImage

        src_pts = np.float32([kp2[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp1[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Compute transformation
        A, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts, method=cv2.RANSAC)
        if A is None:
            HomogResult = cv2.findHomography(src_pts, dst_pts, method=cv2.RANSAC)
            if HomogResult[0] is None:
                print(f"⚠️  No transformation for image {index2}. Skipping.")
                return self.resultImage
            H = HomogResult[0]
        else:
            H = None

        # Compute canvas size
        height1, width1 = image1.shape[:2]
        height2, width2 = image2.shape[:2]
        corners1 = np.float32([[0, 0], [0, height1], [width1, height1], [width1, 0]])
        corners2 = np.float32([[0, 0], [0, height2], [width2, height2], [width2, 0]])
        
        warpedCorners2 = np.zeros((4, 2))
        for i in range(4):
            cornerX, cornerY = corners2[i]
            if H is None:
                warpedCorners2[i] = [A[0, 0] * cornerX + A[0, 1] * cornerY + A[0, 2],
                                     A[1, 0] * cornerX + A[1, 1] * cornerY + A[1, 2]]
            else:
                denom = H[2, 0] * cornerX + H[2, 1] * cornerY + H[2, 2]
                warpedCorners2[i] = [(H[0, 0] * cornerX + H[0, 1] * cornerY + H[0, 2]) / denom,
                                     (H[1, 0] * cornerX + H[1, 1] * cornerY + H[1, 2]) / denom]
        
        allCorners = np.concatenate((corners1, warpedCorners2), axis=0)
        xMin, yMin = np.int32(allCorners.min(axis=0) - 0.5)
        xMax, yMax = np.int32(allCorners.max(axis=0) + 0.5)

        # Warp images
        translation = np.float32([[1, 0, -xMin], [0, 1, -yMin], [0, 0, 1]])
        warpedResImg = cv2.warpPerspective(self.resultImage, translation, (xMax - xMin, yMax - yMin))
        
        if H is None:
            A_homog = np.vstack([A, [0, 0, 1]])
            fullTransformation = np.dot(translation, A_homog)
        else:
            fullTransformation = np.dot(translation, H)
        
        warpedImage2 = cv2.warpPerspective(image2, fullTransformation, (xMax - xMin, yMax - yMin))

        # Create masks
        resGray = cv2.cvtColor(warpedResImg, cv2.COLOR_BGR2GRAY)
        _, mask_result = cv2.threshold(resGray, 1, 255, cv2.THRESH_BINARY)
        
        img2Gray = cv2.cvtColor(warpedImage2, cv2.COLOR_BGR2GRAY)
        _, mask_new = cv2.threshold(img2Gray, 1, 255, cv2.THRESH_BINARY)
        
        overlap_mask = cv2.bitwise_and(mask_result, mask_new)
        new_only_mask = cv2.bitwise_and(mask_new, cv2.bitwise_not(mask_result))

        # Blend
        if self.use_blending and np.count_nonzero(overlap_mask) > 100:
            try:
                dist_result = cv2.distanceTransform(mask_result, cv2.DIST_L2, 5)
                dist_new = cv2.distanceTransform(mask_new, cv2.DIST_L2, 5)
                
                overlap_float = overlap_mask.astype(np.float32) / 255.0
                total_dist = dist_result + dist_new + 1e-5
                weight_new = (dist_new / total_dist) * overlap_float
                weight_new = cv2.GaussianBlur(weight_new, (51, 51), 0)
                
                weight_new_3ch = cv2.merge([weight_new, weight_new, weight_new])
                
                result = warpedResImg.astype(np.float32) * (1 - weight_new_3ch) + warpedImage2.astype(np.float32) * weight_new_3ch
                
                new_only_3ch = cv2.merge([new_only_mask, new_only_mask, new_only_mask]) / 255.0
                result = result * (1 - new_only_3ch) + warpedImage2.astype(np.float32) * new_only_3ch
                
                result = np.clip(result, 0, 255).astype(np.uint8)
                
            except Exception as e:
                print(f"⚠️  Blending failed: {e}")
                mask_combined = cv2.bitwise_or(new_only_mask, overlap_mask)
                mask_float = mask_combined.astype(np.float32) / 255.0
                mask_3ch = cv2.merge([mask_float, mask_float, mask_float])
                result = warpedResImg + (warpedImage2 * mask_3ch).astype(np.uint8)
        else:
            mask_float = new_only_mask.astype(np.float32) / 255.0
            mask_3ch = cv2.merge([mask_float, mask_float, mask_float])
            result = warpedResImg + (warpedImage2 * mask_3ch).astype(np.uint8)
        
        self.resultImage = result
        
        intermediate_result_path = os.path.join(self.output_dir, f"intermediateResult_{index2}.png")
        cv2.imwrite(intermediate_result_path, result)
        print(f"Intermediate saved: {intermediate_result_path}")

        return result