# feature extraction and detection engine

import cv2 as cv
from matplotlib import image
import numpy as np
import time
from multiprocessing import Pool
import os
from typing import List, Tuple, Dict, Union, Optional
from .stitch_utils import StitchUtils


class Image:
    keypoint_total = []
    l_cdf = []

    def __init__(self, image: np.ndarray, kp_des: np.ndarray, idx: int):
        self.image = image
        self.kp_des = kp_des
        self.shapes = image.shape[:2]  # (height, width)
        self.raw_h = np.eye(3)  # Initial identity homography
        self.nn_h = np.eye(3)   # Non-negative homography
        self.homography = np.eye(3)
        self.shape_sem = self.shapes  # Semantic shape (may change after warping)
        
        # self.color = cv.split(image)  # [B, G, R] channels
        self.color = list(cv.split(image))  # [B, G, R] channels
        self.image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV_FULL)

        # Calculate PDF/CDF for color correction
        self.find_pdf()
    
    def find_pdf(self) -> None:
        Image.keypoint_total.append(len(self.kp_des[0]))
        
        # calculate gray PDF and CDF
        gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        pdf = StitchUtils.calc_pdf(gray)
        cdf = StitchUtils.calc_cdf(pdf)
        Image.l_cdf.append(cdf)
    
    def assign(self, homography: np.ndarray, shape_ref: Tuple[int, int]) -> None:
        """
        Assign a new homography and reference shape.

        Args:
            homography: 3x3 homography matrix
            shape_ref: (height, width) of reference shape
        """
        self.raw_h = homography
        self.nn_h = homography.copy()
        self.homography = homography.copy()
        self.shape_sem = shape_ref
        
    def update(self, translation: np.ndarray, shape_ref: Tuple[int, int]) -> None:
        """
        Update homography with a translation matrix.
        
        Args:
            translation: 3x3 translation matrix
            shape_ref: (height, width) of reference shape
        """
        self.raw_h = translation @ self.raw_h
        self.nn_h = translation @ self.nn_h
        self.homography = translation @ self.homography
        self.shape_sem = shape_ref
    
    def update_channel(self, channel: np.ndarray, index: int) -> None:
        """
        Update a specific color channel.

        Args:
            channel: New channel data
            index: Channel index (0 for B, 1 for G, 2 for R)
        """
        self.color[index] = channel
        self.image = cv.merge(self.color)


class ImageRegistration:
    """
    Extract features from multiple images in parallel.
    """
    load_time = 0
    reg_time = 0
    
    def __init__(self, detector_type: str = 'sift',
                 compression_factor: Optional[float] = None,
                 nfeature: int = 0):
        """
        Initialize registration with specified detector type.

        Args:
            detector_type: Type of feature detector ('sift' or 'orb')
            compression_factor: Image scaling factor for processing (0.1-1.0)
            nfeature: Maximum number of features to extract per image
        """
        self.detector_type = detector_type.lower()
        self.compression_factor = compression_factor
        self.nfeature = nfeature
    
    def load_img(self, folder: List[str]) -> List[List]:
        """
        Load images from a list of paths.

        Args:
            folder: List of image paths

        Returns:
            List of [path, image] pairs
        """
        start_time = time.time()

        list_image = []
        for image_path in folder:
            image = cv.imread(str(image_path))
            if image is None:
                print(f"Failed to load image: {image_path}")
                continue

            # Apply compression if specified
            if self.compression_factor is not None and 0.1 <= self.compression_factor <= 1.0:
                height, width = image.shape[:2]
                new_height = int(height * self.compression_factor)
                new_width = int(width * self.compression_factor)
                image = cv.resize(image, (new_width, new_height))

            list_image.append([image_path, image])

        ImageRegistration.load_time = time.time() - start_time
        return list_image
    
    def run_map(self, items: List[List], processes: Optional[int] = None,
                chunksize: Optional[int] = None) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """
        Extract features in parallel from multiple images.

        Args:
            items: List of [path, image] pairs
            processes: Number of parallel processes (None uses CPU count)
            chunksize: Size of work chunks for parallel processing

        Returns:
            Dictionary mapping paths to (keypoints, descriptors) pairs
        """
        start_time = time.time()
        result_dict = {}

        # for small batches, avoid multiprocessing overhead
        if len(items) < 4:
            print("Using single process for small batch")
            for path, img in items:
                try:
                    result = self.worker(path, img)
                    result_dict[result[0]] = result[1]
                except Exception as e:
                    print(f"Error processing {path}: {e}")
        else:
            # use multiprocessing for larger batches
            if processes is None:
                processes = min(os.cpu_count() or 1, len(items))
                
            try:
                with Pool(processes=processes) as pool:
                    results = []
                    for path, img in items:
                        results.append(pool.apply_async(self.worker, (path, img)))
                    
                    for res in results:
                        try:
                            path, kp_des = res.get(timeout=30)  # timeout to avoid hanging
                            result_dict[path] = kp_des
                        except Exception as e:
                            print(f"Error in worker process: {e}")
            except Exception as e:
                print(f"Multiprocessing error: {e}")

        ImageRegistration.reg_time = time.time() - start_time
        return result_dict
    
    def worker(self, img_name: str, image: np.ndarray) -> Tuple[str, Tuple[np.ndarray, np.ndarray]]:
        """
        Worker process for feature extraction.

        Args:
            img_name: Path to the image
            image: Image data

        Returns:
            Tuple of (img_name, (keypoints, descriptors))
        """
        try:
            if image is None or image.size == 0:
               print(f"Invalid image data for {img_name}")
               return img_name, (np.array([]), np.array([]))
            image_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            kp_des = self.get_kp(image_gray)
            return img_name, kp_des
        except Exception as e:
            print(f"Worker error processing {img_name}: {e}")
            return img_name, (np.array([]), np.array([]))
    
    def get_kp(self, image_gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract keypoints and descriptors from grayscale image.

        Args:
            image_gray: Grayscale image

        Returns:
            Tuple of (keypoints, descriptors)
        """
        try:
            if image_gray is None or image_gray.size == 0:
                print("Invalid grayscale image")
                return np.array([]), np.array([])
            
            if self.detector_type == 'sift':
                nfeatures = min(2000, self.nfeature if self.nfeature > 0 else 1000)
                detector = cv.SIFT_create(nfeatures=nfeatures)
            elif self.detector_type == 'orb':
                nfeatures = min(1500, self.nfeature if self.nfeature > 0 else 500)
                detector = cv.ORB_create(nfeatures=nfeatures)
            else:
                raise ValueError(f"Unsupported detector type: {self.detector_type}")
            
            # extract features with timeout handling
            start_time = time.time()
            keypoints, descriptors = detector.detectAndCompute(image_gray, None)
            elapsed_time = time.time() - start_time
            
            if elapsed_time > 5:
                print(f"Warning: Feature extraction took too long ({elapsed_time:.2f}s)")
                
            if keypoints is None or len(keypoints) == 0:
                print("No keypoints detected")
                return np.array([]), np.array([])
            
            keypoints = np.float32([kp.pt for kp in keypoints])
            return keypoints, descriptors
        except Exception as e:
            print(f"Error in feature detection: {e}")
            return np.array([]), np.array([])