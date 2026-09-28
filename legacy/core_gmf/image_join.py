# image registration and joining engine

import numpy as np
from typing import List, Tuple, Dict, Union, Optional
import time
import cv2 

from .image_corrector import ImageCorrector
from .stitch_utils import StitchUtils

class ImageJoiner:
    homography_time = []
    
    def __init__(self, dict_image: Dict, detector_type: str = 'sift'):
        self.dict_image = dict_image
        self.homography = ImageCorrector(detector_type)
        
    @staticmethod
    def convert_to_lists(structure):
        if isinstance(structure, tuple):
            return [ImageJoiner.convert_to_lists(item) for item in structure]
        elif isinstance(structure, list):
            if all(isinstance(item, list) for item in structure):
                return [subitem for item in structure for subitem in ImageJoiner.convert_to_lists(item)]
            return [ImageJoiner.convert_to_lists(item) for item in structure]
        return structure
    
    def join_image(self, ops_sequence: List) -> None:
        """
        Join images based on the provided operation sequence.
        """
        
        start_time = time.time()
        
        # ops_sequence = ImageJoiner.convert_to_lists(ops_sequence)
        
        for layer in ops_sequence:
            for group in layer:
                if isinstance(group[0], str) and len(group) > 2:
                    self.ref_center_image(group)
                elif isinstance(group[0], str) and len(group) == 2:
                    self.seq_image(group)
                elif len(group) == 3:
                    self.ref_center_group(group)
                elif len(group) == 2:
                    self.seq_group(group)
        ImageJoiner.homography_time = time.time() - start_time

    def ref_center_image(self,group: List[str]) -> None:
        """
        Register images in a group to the center image.

        Args:
            group: List of image keys in the group
        """
        center_idx = len(group) // 2
        
        # get image object
        images = [self.dict_image[img_path] for img_path in group]
        ref_image = images[center_idx]
        
        homographies = []
        for i, image in enumerate(images):
            if i == center_idx:
                homographies.append(np.eye(3))
                continue
            
            # compute homography between this image and the reference image
            H, _, _ = self.homography.get_homography(ref_image.kp_des, image.kp_des)
            
            if H is None:
                print(f"Warning: Homography computation failed {group[center_idx]} and {group[i]}")
                H = np.eye(3)
            homographies.append(H)

        # calculate canvas size to accomodate all images
        shape_ref = ref_image.shapes
        for i, image in enumerate(images):
            if i == center_idx:
                continue
            
            # Calculate new canvas size
            H1, shape_ref = ImageCorrector.get_translation(shape_ref, image.shapes, homographies[i])
            
            for j in range(len(homographies)):
                homographies[j] = H1 @ homographies[j]
        
        # assign homographies to all images
        for i, image in enumerate(images):
            image.assign(homographies[i], shape_ref)
            
    def ref_center_group(self, group: List[List[str]]) -> None:
        """
        Register groups of images to the center group.

        Args:
            group: List of image groups
        """
        center_idx = len(group) // 2    
        
        # get images for each group (flattened)
        group_images = []
        for subgroup in group:
            images = [self.dict_image[img_path] for img_path in subgroup]
            group_images.append(images)
            
        # get reference and floating groups
        ref_group = group_images[center_idx]
        other_groups = group_images[:center_idx] + group_images[center_idx+1:]
        
        ref_img = ref_group[len(ref_group)//2]
        flo_imgs = [flo_group[len(flo_group)//2] for flo_group in other_groups]
        
        shape_ref = ref_img.shapes
        H_trans = np.eye(3)
        
        # calculate homographies
        homographies = []
        for i, fl0_img in enumerate(flo_imgs):
            H, _, _ = self.homography.get_homography(ref_img.kp_des, fl0_img.kp_des)
            
            if H is None:
                print(f"Warning: Homography computation failed {group[center_idx]} and {group[i]}")
                H = np.eye(3)
            homographies.append(H)
            
        # calculate canvas size to accomodate all images
        shape_ref = ref_img.shapes
        
        # process each groups
        for i, flo_img in enumerate(flo_imgs):
            H_trans, shape_ref = ImageCorrector.get_translation(shape_ref, flo_img.shapes, homographies[i])

            # update homographies with translation
            homographies[i] = np.matmul(H_trans, homographies[i])
            
            # update all images in the floating group
            for img in other_groups[i]:
                H_final = homographies[i] @ img.homography
                img.assign(H_final, shape_ref)
                
        # update all images in the reference group
        for img in ref_group:
            img.update(H_trans, shape_ref)
            
    def seq_group(self, group: List[List[str]]) -> None:
        """
        Sequentially register two groups of images.

        Args:
            group: List of two image groups
        """
        group1_images = [self.dict_image[img_path] for img_path in group[0]]
        group2_images = [self.dict_image[img_path] for img_path in group[1]]
        
        ref_img = group1_images[len(group1_images)//2]
        flo_img = group2_images[len(group2_images)//2]
        
        # calculate homography between groups
        H, _, _ = self.homography.get_homography(ref_img.kp_des, flo_img.kp_des)

        if H is None:
            print(f"Warning: Homography computation failed {group[0]} and {group[1]}")
            H = np.eye(3)

        # calculate canvas size to accomodate both groups
        H_trans, shape_ref = ImageCorrector.get_translation(ref_img.shapes, flo_img.shapes, H)

        # apply transformation to second group
        H_final = H_trans @ H
        for img in group2_images:
            H_combined = H_final @ img.homography
            img.assign(H_combined, shape_ref)
        
        for img in group1_images:
            img.update(H_trans, shape_ref)

    def seq_image(self, group: List[str]) -> None:
        """
        Sequentially register two images.

        Args:
            group: List of two image keys
        """
        img1 = self.dict_image[group[0]]
        img2 = self.dict_image[group[1]]
        
        # calculate homography between images
        H, _, _ = self.homography.get_homography(img1.kp_des, img2.kp_des)

        if H is None:
            print(f"Warning: Homography computation failed {group[0]} and {group[1]}")
            H = np.eye(3)

        # calculate canvas size to accomodate both images
        H_trans, shape_ref = ImageCorrector.get_translation(img1.shapes, img2.shapes, H)

        # apply transformation to second image
        H_final = H_trans @ H
        img2.assign(H_final, shape_ref)

        img1.update(H_trans, shape_ref)

    def update_shape(self) -> Tuple[int, int]:
        """
        Calculate final canvas size and update transformations

        Returns:
            Tuple[int, int]: The updated canvas shape (height, width)
        """
        # find global bounding box
        
        xmin_g = float('inf')
        ymin_g = float('inf')
        xmax_g = float('-inf')
        ymax_g = float('-inf')
        
        for img_name, img in self.dict_image.items():
            corners = np.float32([[0, 0], [0, img.shapes[0]], [img.shapes[1], img.shapes[0]], [img.shapes[1], 0]]).reshape(-1, 1, 2)
            transformed_corners = cv2.perspectiveTransform(corners, img.homography)
            
            x_min, y_min = transformed_corners.min(axis=0).ravel()
            x_max, y_max = transformed_corners.max(axis=0).ravel()
            
            xmin_g = min(xmin_g, x_min)
            ymin_g = min(ymin_g, y_min)
            xmax_g = max(xmax_g, x_max)
            ymax_g = max(ymax_g, y_max)
            
        # calculate final dimensions
        final_shape = (int(ymax_g - ymin_g) + 2, int(xmax_g - xmin_g) + 2)  # (height, width)
        
        # create final translation for positive coordinates
        translation = np.eye(3)
        translation[0, 2] = -xmin_g
        translation[1, 2] = -ymin_g 
        
        # update all images with final translation
        for img in self.dict_image.values():
            img.update(translation, final_shape)
        
        return final_shape