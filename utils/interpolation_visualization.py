"""
created on 6th April 2026
authored by Abyan

tools intended for visualizing the interpolation process in graphic form, such as blending masks and intermediate blended images

(well i can call this my playground for testing out different blending techniques and visualizing the results)
"""

import cv2
import numpy as np

import sys
import os

src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(src_dir)

from src.blending import PyramidBlender
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import scipy.interpolate as itp

class justInterpolation:
    # just a class for testing out different interpolation techniques, not necessarily for image processing
    @staticmethod
    def test_interpolation():
        x = np.array([0, 1, 2, 3, 4])
        y = np.array([0, 1, 0, 1, 0])

        linear_interp = itp.interp1d(x, y, kind='linear')
        cubic_interp = itp.interp1d(x, y, kind='cubic')

        x_new = np.linspace(0, 4, 100)
        y_linear = linear_interp(x_new)
        y_cubic = cubic_interp(x_new)

        plt.figure(figsize=(10, 6))
        plt.plot(x, y, 'ro', label='Data Points')
        plt.plot(x_new, y_linear, 'b-', label='Linear Interpolation')
        plt.plot(x_new, y_cubic, 'g-', label='Cubic Interpolation')
        plt.legend()
        plt.title("Interpolation Techniques")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.grid()
        plt.savefig("interpolation_comparison.png")
        plt.show()

class InterpolationVisualizer:
    # just a class for visualizing the interpolation process, not necessarily for image processing
    # curious to see how the blending mask evolves as we change the parameters, and how the blended image changes as well
    @staticmethod
    def interpolation():
        # create a simple blending mask
        width, height = 400, 400
        mask_binary = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask_binary, (width//2, height//2), 100, 255, -1)  # white circle in the center

        # visualize the blending mask
        plt.figure(figsize=(6, 6))
        plt.title("Blending Mask")
        plt.imshow(mask_binary, cmap='gray')
        plt.axis('off')
        plt.show()
        # plt.savefig("blending_mask.png")


class interpolationCompute:
    # class for computing interpolation between two images using different techniques, and visualizing the results
    @staticmethod
    def blend_and_visualize(img1, img2, mask_binary, num_levels=2):
        blended = PyramidBlender.blend_images_lowres(img1, img2, mask_binary, num_levels)
        
        # visualize the original images, mask, and blended result
        plt.figure(figsize=(12, 8))
        plt.subplot(2, 2, 1)
        plt.title("Image 1")
        plt.imshow(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        
        plt.subplot(2, 2, 2)
        plt.title("Image 2")
        plt.imshow(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        
        plt.subplot(2, 2, 3)
        plt.title("Blending Mask")
        plt.imshow(mask_binary, cmap='gray')
        plt.axis('off')
        
        plt.subplot(2, 2, 4)
        plt.title("Blended Result")
        plt.imshow(cv2.cvtColor(blended.astype('uint8'), cv2.COLOR_BGR2RGB))
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    justInterpolation.test_interpolation()

