import cv2 
import numpy as np

class PyramidBlender:

    @staticmethod
    def gaussian_pyramid(img, num_levels):
        lower = img.copy()
        gaussian_pyr = [lower]
        for i in range(num_levels):
            lower = cv2.pyrDown(lower)
            gaussian_pyr.append(lower)
        return gaussian_pyr
    
    @staticmethod
    def laplacian_pyramid(gaussian_pyr):
        laplacian_top = gaussian_pyr[-1]
        num_levels = len(gaussian_pyr) - 1
        
        laplacian_pyr = [laplacian_top]
        for i in range(num_levels, 0, -1):
            size = (gaussian_pyr[i - 1].shape[1], gaussian_pyr[i - 1].shape[0])
            gaussian_expanded = cv2.pyrUp(gaussian_pyr[i], dstsize=size)
            laplacian = cv2.subtract(gaussian_pyr[i - 1], gaussian_expanded)
            laplacian_pyr.append(laplacian)
        return laplacian_pyr
    
    @staticmethod
    def blend_pyramids(laplacian_A, laplacian_B, mask_pyr):
        """Blend two Laplacian pyramids using mask pyramid"""
        blended_pyr = []
        for la, lb, mask in zip(laplacian_A, laplacian_B, mask_pyr):
            # Ensure mask has correct dimensions for broadcasting
            if len(la.shape) == 2 and len(mask.shape) == 2:
                # Both are 2D, check dimensions match
                if la.shape != mask.shape:
                    mask = cv2.resize(mask, (la.shape[1], la.shape[0]))
            
            # Expand mask dimensions if needed for multi-channel images
            if len(la.shape) == 3 and len(mask.shape) == 2:
                mask = mask[:, :, np.newaxis]
            
            ls = la * mask + lb * (1.0 - mask)
            ls = np.clip(ls, 0, 255)
            blended_pyr.append(ls)
        return blended_pyr
    
    @staticmethod
    def reconstruct(laplacian_pyr):
        '''
        Reconstruct image from its Laplacian pyramid.
        
        :param laplacian_pyr: Laplacian pyramid (list of images)
        '''
        laplacian_top = laplacian_pyr[0]
        num_levels = len(laplacian_pyr) - 1

        for i in range(num_levels):
            size = (laplacian_pyr[i + 1].shape[1], laplacian_pyr[i + 1].shape[0])
            laplacian_expanded = cv2.pyrUp(laplacian_top, dstsize=size)
            laplacian_top = cv2.add(laplacian_pyr[i+1].astype('float32'), laplacian_expanded.astype('float32'))
        return np.clip(laplacian_top, 0, 255).astype('uint8')
    
    @staticmethod
    def create_blend_mask(shape, mask_binary):
        """Create smooth blending mask with proper dimensions"""
        mask_float = mask_binary.astype('float32') / 255.0
        # Apply gaussian blur to the mask to create a smooth transition
        mask_smooth = cv2.GaussianBlur(mask_float, (51, 51), 0)
        return mask_smooth
    
    @staticmethod
    def blend_images(img1, img2, mask_binary, num_levels=4):
        '''
        Blend two images using pyramid blending
        
        :param img1: Background image (already placed)
        :param img2: New image to blend in
        :param mask_binary: Binary mask (255 where img2 should be placed, 0 elsewhere)
        :param num_levels: Number of pyramid levels

        :return: Blended image
        '''
        if img1.shape != img2.shape:
            raise ValueError(f"Images must have the same dimensions. Got {img1.shape} and {img2.shape}")
        
        # Ensure mask has same spatial dimensions as images
        if mask_binary.shape[:2] != img1.shape[:2]:
            mask_binary = cv2.resize(mask_binary, (img1.shape[1], img1.shape[0]))
        
        # Create smooth blending mask
        mask_smooth = PyramidBlender.create_blend_mask(img1.shape[:2], mask_binary)
        
        # Handle multi-channel images
        if len(img1.shape) == 3:
            channels = img1.shape[2]
            blended = np.zeros_like(img1, dtype=np.float32)
            
            for c in range(channels):
                # Generate pyramids for each channel
                gauss_A = PyramidBlender.gaussian_pyramid(img1[:,:,c].astype(np.float32), num_levels)
                gauss_B = PyramidBlender.gaussian_pyramid(img2[:,:,c].astype(np.float32), num_levels)
                
                # Generate mask pyramid
                mask_pyr = PyramidBlender.gaussian_pyramid(mask_smooth.astype(np.float32), num_levels)
                
                # Generate Laplacian pyramids
                lap_A = PyramidBlender.laplacian_pyramid(gauss_A)
                lap_B = PyramidBlender.laplacian_pyramid(gauss_B)
                
                # Blend pyramids
                blended_pyr = PyramidBlender.blend_pyramids(lap_A, lap_B, mask_pyr)
                
                # Reconstruct
                blended[:,:,c] = PyramidBlender.reconstruct(blended_pyr)
            
            return blended.astype(np.uint8)
        else:
            # Single channel
            gauss_A = PyramidBlender.gaussian_pyramid(img1.astype(np.float32), num_levels)
            gauss_B = PyramidBlender.gaussian_pyramid(img2.astype(np.float32), num_levels)
            mask_pyr = PyramidBlender.gaussian_pyramid(mask_smooth.astype(np.float32), num_levels)
            
            lap_A = PyramidBlender.laplacian_pyramid(gauss_A)
            lap_B = PyramidBlender.laplacian_pyramid(gauss_B)
            
            blended_pyr = PyramidBlender.blend_pyramids(lap_A, lap_B, mask_pyr)
            return PyramidBlender.reconstruct(blended_pyr).astype(np.uint8)