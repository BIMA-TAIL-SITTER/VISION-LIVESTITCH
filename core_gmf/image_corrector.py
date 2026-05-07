# homography computation

import cv2
import numpy as np
from typing import List, Tuple, Dict, Union, Optional
from .stitch_utils import StitchUtils

class ImageCorrector:
    error_x = []
    error_y = []
    error_h = []
    reproj_count = []
    matches_total = []
    
    def __init__(self, detector_type: str = 'sift'):
        '''initialize homography calculator
        Args:
            detector_type: Feature detector type ('sift' or 'orb')
        '''
        self.detector_type = detector_type.lower()

        if self.detector_type == 'sift':
            # FLANN parameters for SIFT
            self.algorithm = 1 # KDTree
            self.tree = 5
            self.checks = 50
            flann_params = dict(algorithm=self.algorithm, trees=self.tree)
            self.matcher = cv2.FlannBasedMatcher(flann_params, {})

        elif self.detector_type == 'orb':
            # Brute force matching with Hamming distance for ORB
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        else:
            raise ValueError("Unsupported detector type. Use 'sift' or 'orb'.")
        
    def get_homography(self, ref_kp_des: Tuple[np.ndarray, np.ndarray],
                       flo_kp_des: Tuple[np.ndarray, np.ndarray],
                       ratio: float = 0.8,
                       ref_H: np.ndarray = np.eye(3),
                       flo_H: np.ndarray = np.eye(3),
                       return_match: bool = False) -> Union[
                           Tuple[np.ndarray, np.ndarray, np.ndarray],
                           Tuple[np.ndarray, np.ndarray]
                       ]:
        """
        Compute homography between two sets of features.

        Args:
            ref_kp_des: (keypoints, descriptors) of reference image
            flo_kp_des: (keypoints, descriptors) of floating image
            ratio: Ratio test threshold (0.0 to 1.0)
            ref_H: Existing homography for reference image
            flo_H: Existing homography for floating image
            return_match: Whether to return matched points instead of homography

        Returns:
            If return_match is False:
                tuple: (homography, reference_points, floating_points)
            If return_match is True:
                tuple: (reference_points, floating_points)
        """
        
        # Extract keypoints and descriptors
        ref_kp, ref_des = ref_kp_des
        flo_kp, flo_des = flo_kp_des
        
        matches = []
        
        if self.detector_type == 'sift':
            # knn matching with ratio test for SIFT
            raw_matches = self.matcher.knnMatch(ref_des, flo_des, k=2)
            for m, n in raw_matches:
                if m.distance < ratio * n.distance:
                    matches.append(m)
        else:
            # knn maatching with ratios test for orb
            raw_matches = self.matcher.match(ref_des, flo_des, k=2)
            for pair in raw_matches:
                if len(pair) == 2:
                    m, n = pair
                    if m.distance < ratio * n.distance:
                        matches.append(m)
                        
        # store stats
        ImageCorrector.matches_total.append(len(matches))
        
        if len(matches) < 4:
            if return_match:
                return np.array([]), np.array([])
            return None, np.array([]), np.array([])

        ref_pts = np.float32([ref_kp[m.queryIdx] for m in matches])
        flo_pts = np.float32([flo_kp[m.trainIdx] for m in matches])

        if return_match:
            return ref_pts, flo_pts
        
        # apply existing transformations
        if not np.array_equal(ref_H, np.eye(3)) or not np.array_equal(flo_H, np.eye(3)):
            ref_pts = StitchUtils.warp_point(ref_pts, ref_H, False)
            flo_pts = StitchUtils.warp_point(flo_pts, flo_H, False)
            
        # find homography using RANSAC
        H, _ = cv2.findHomography(flo_pts, ref_pts, cv2.RANSAC, 5.0)
        
        # Refine Using inliers only
        ImageCorrector.reproj_count.append(1)
        cor_ref, cor_flo, H_refined = StitchUtils.average_error_measure(flo_pts, ref_pts, correction=True)
        
        # calculate error metrics
        total_error, error_x, error_y = StitchUtils.average_error_measure(cor_flo, cor_ref, source_transform=H_refined)

        # Store error metrics
        ImageCorrector.error_h.append(total_error)
        ImageCorrector.error_x.append(error_x)
        ImageCorrector.error_y.append(error_y)
        
        return H_refined, cor_ref, cor_flo
    
    @staticmethod
    def get_non_neg(shape_flo: Tuple[int, int],
                    raw_h: np.ndarray,
                    return_stable: bool = False) -> np.ndarray:
        """
        Ensure non-negative coordinates after transformation.

        Args:
            shape_flo: (height, width) of the image
            raw_h: Raw homography matrix
            return_stable: Whether to return stabilized transformation

        Returns:
            Adjusted homography ensuring non-negative coordinates
        """
        
        row, col = shape_flo
        
        # define corners of the image
        corner_point = np.float32([[0, 0], [0, row], [col, row], [col, 0]]).reshape(-1, 1, 2)
        
        # transform corners using homography
        pose_point = StitchUtils.warp_point(corner_point, raw_h)
        
        # find minimum coordinates
        x_min, y_min = pose_point.min(axis=0).ravel()
        
        # ensure non-negative coordinates
        x_cor = min(x_min, 0)
        y_cor = min(y_min, 0)
        
        # create translation matrix
        translation = np.eye(3)
        translation[0, 2] = -x_cor
        translation[1, 2] = -y_cor
        
        # combine transformations
        non_neg_h = translation @ raw_h
        
        # normalize homography
        result = non_neg_h / non_neg_h[2, 2]
        
        if not return_stable:
            return result
        
        corner_point = np.float32([[0, 0], [0, row], [col, row], [col, 0]]).reshape(-1, 1, 2)
        pose_point = StitchUtils.warp_point(corner_point, non_neg_h)
        
        x_max, y_max = pose_point.max(axis=0).ravel()
        x_min, y_min = pose_point.min(axis=0).ravel()
        
        width_new, height_new = int(x_max - x_min), int(y_max - y_min)
        scale_w, scale_h = col / width_new, row / height_new
        
        scale_factor = min(1.0, scale_w, scale_h)
        
        # create scaling matrix
        scale_matrix = np.eye(3)
        scale_matrix[0, 0] = scale_factor
        scale_matrix[1, 1] = scale_factor
        
        stable_h = scale_matrix @ non_neg_h
        
        return stable_h
    
    @staticmethod
    def get_position(shape_flo: Tuple[int, int], homography: np.ndarray,
                    return_shape: bool = False,
                    return_corner: bool = False) -> Union[Dict[str, float],
                                                         Tuple[Dict[str, float], Tuple[int, int]],
                                                         np.ndarray]:
        """
        Calculate position and size after transformation.

        Args:
            shape_flo: (height, width) of the image
            homography: Homography matrix
            return_shape: Whether to return shape information
            return_corner: Whether to return transformed corner coordinates

        Returns:
            If return_shape is True:
                tuple: (position_dict, shape_tuple)
            If return_corner is True:
                ndarray: transformed corner coordinates
            Otherwise:
                dict: position information {'x_min', 'x_max', 'y_min', 'y_max'}
        """
        
        row, col = shape_flo
        
        corner_point = np.float32([[0, 0], [0, row], [col, row], [col, 0]]).reshape(-1, 1, 2)
        
        pose_point = StitchUtils.warp_point(corner_point, homography)
        
        if return_corner:
            return pose_point
        
        # find bounds of transformed image
        x_min, y_min = pose_point.min(axis=0).ravel()
        x_max, y_max = pose_point.max(axis=0).ravel()
        
        # create position dictionary
        position = {
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max
        }
        
        if return_shape:
            width, height = int(x_max - x_min) + 1, int(y_max - y_min) + 1
            return position, (height, width)
        
        return position
    
    @staticmethod
    def get_translation(shape_ref: Tuple[int, int], shape_flo: Tuple[int, int],
                       H_flo: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        Calculate translation for proper alignment
        
        Args:
            shape_ref: (height, width) of reference image
            shape_flo: (height, width) of floating image
            H_flo: Homography for floating image

        Returns:
            tuple: (adjusted_homography, new_shape)
        """
        
        position_flo, shape_flo_trans = ImageCorrector.get_position(shape_flo, H_flo, return_shape=True)

        # calculate combined canvas size
        height_new = max(shape_ref[0], int (position_flo['y_max']))
        width_new = max(shape_ref[1], int (position_flo['x_max']))
        
        H_ref = np.eye(3)
        
        return H_ref, (height_new, width_new)