# image registration and joining engine
# from legacy

from .image_corrector import ImageCorrector
from .stitch_utils import StitchUtils
import numpy as np
import time


class ImageJoiner():
    homo_time = 0

    def __init__(self, dict_image:dict, detector_type:str = 'sift'):
        self.dict_image = dict_image
        self.homography = ImageCorrector(detector_type = detector_type)
        print('\nstart registering homography')

    def join_image(self,ops_sequence):
        start = time.perf_counter()
        total_operations = sum(len(layer) for layer in ops_sequence)
        completed_operations = 0
        
        for layer in ops_sequence:
            for group in layer:
                if type(group[0]) == str and len(group) > 2:
                    self.ref_center_image(group)
        
                elif type(group[0]) == str and len(group) == 2:
                    self.seq_image(group)

                elif len(group) == 3:
                    self.ref_center_group(group)
  
                elif len(group) == 2:
                    self.seq_group(group)

                elif len(group) == 1:
                    continue
                
                completed_operations += 1

                progress_percentage = int((completed_operations / total_operations) * 33 + 33)
                      
        end = time.perf_counter()
        total = end - start
        ImageJoiner.homo_time += total

        print('\ndone registering homography')

    def ref_center_image(self,group:list):

        instance0 = self.dict_image[group[0]]
        instance1 = self.dict_image[group[1]]
        instance2 = self.dict_image[group[2]]

        H0,_,_ = self.homography.get_homography(instance1.kp_des,
                                           instance0.kp_des)
        H2,_,_ = self.homography.get_homography(instance1.kp_des,
                                           instance2.kp_des)

        H1_1,shape_ref = ImageCorrector.get_translation(instance1.shapes, instance0.shapes,H0)
        H2 = np.matmul(H1_1,H2)
        H1_2,shape_ref = ImageCorrector.get_translation(shape_ref,instance2.shapes,H2)

        H1 = np.matmul(H1_2,H1_1)
        H0 = np.matmul(H1,H0)

        instance0.assign(H0,shape_ref)
        instance1.assign(H1,shape_ref)
        instance2.assign(H2,shape_ref)

        if len(group) == 4:
            instance3 = self.dict_image[group[3]]
            H3,_,_ = self.homography.get_homography(instance2.kp_des,
                                                    instance3.kp_des,
                                                    ref_H = instance2.nn_h)
            H3_1,shape_ref = ImageCorrector.get_translation(shape_ref,instance3.shapes,H3)

            instance0.update(H3_1,shape_ref)
            instance1.update(H3_1,shape_ref)
            instance2.update(H3_1,shape_ref)

            instance3.assign(H3,shape_ref)

    def ref_center_group(self,group):
        
        inst_img1 = self.dict_image[group[0][-1]]
        inst_img2_1 = self.dict_image[group[1][0]]
        inst_img2_3 = self.dict_image[group[1][-1]]
        inst_img3 = self.dict_image[group[2][0]]

        H0_layer,_,_ = self.homography.get_homography(inst_img2_1.kp_des,
                                                 inst_img1.kp_des,
                                                 ref_H = inst_img2_1.nn_h,
                                                 flo_H = inst_img1.nn_h)

        H2_layer,_,_ = self.homography.get_homography(inst_img2_3.kp_des,
                                                 inst_img3.kp_des,
                                                 ref_H = inst_img2_3.nn_h,
                                                 flo_H = inst_img3.nn_h)

        H1_layer1, shape_ref_L =  ImageCorrector.get_translation(inst_img2_1.shape_sem,
                                                                   inst_img1.shape_sem,
                                                                   H0_layer)
        H2_layer = np.matmul(H1_layer1,H2_layer)

        H1_layer2, shape_ref_L =  ImageCorrector.get_translation(shape_ref_L,
                                                                   inst_img3.shape_sem ,
                                                                   H2_layer)

        H1_layer = np.matmul(H1_layer1,H1_layer2)
        H0_layer = np.matmul(H1_layer,H0_layer)
        H2_layer = np.matmul(H1_layer2,H2_layer)

        list_H = [H0_layer,H1_layer,H2_layer]

        for inv, H in zip(group,list_H):
            for img in inv:
                self.dict_image[img].update(H,shape_ref_L)

    def seq_group(self, group):

        instance0 = self.dict_image[group[0][-1]]
        instance1 = self.dict_image[group[1][0]]

        H0_layer,_,_ = self.homography.get_homography(instance1.kp_des,
                                                  instance0.kp_des,
                                                  ref_H = instance1.nn_h,
                                                  flo_H = instance0.nn_h)

        H1_layer, shape_ref_L =  ImageCorrector.get_translation(instance1.shape_sem,
                                                                  instance0.shape_sem,
                                                                  H0_layer)
        H0_layer = np.matmul(H1_layer,H0_layer)

        list_H = [H0_layer,H1_layer]

        for inv, H in zip(group,list_H):
            for img in inv:
                self.dict_image[img].update(H,shape_ref_L)


    def seq_image(self,group):

        instance0 = self.dict_image[group[0]]
        instance1 = self.dict_image[group[1]]

        H1,_,_ = self.homography.get_homography(instance0.kp_des,
                                                instance1.kp_des)

        H0,shape_ref = ImageCorrector.get_translation(instance0.shapes,
                                                        instance1.shapes,H1)


        instance0.assign(H0,shape_ref)
        instance1.assign(H1,shape_ref)
    
    def update_shape(self):

        # xmin_g = 1000000
        # ymin_g = 1000000
        # xmax_g = -100000
        # ymax_g = -100000

      
        # for img_name in self.dict_image:
        #     instance = self.dict_image[img_name]
        #     ukuran = instance.shapes
        #     non_negh = instance.nn_h
        #     row, col = ukuran

        #     corner_point = [[0,0], [0, row], [col, row], [col, 0]]
        #     corner_point = np.float32(corner_point).reshape(-1, 1, 2)
        #     pose_point = StitchUtils.warp_point(corner_point, non_negh)

        #     x_min, y_min = pose_point.min(axis=0).ravel()
        #     x_max, y_max = pose_point.max(axis=0).ravel()

        #     if x_min < xmin_g:
        #        xmin_g = x_min

        #     if y_min < ymin_g:
        #        ymin_g = y_min

        #     if x_max > xmax_g:
        #        xmax_g = x_max

        #     if y_max > ymax_g:
        #        ymax_g = y_max
            
        # height = ymax_g - ymin_g
        # lenght = xmax_g - xmin_g

        # shapes = np.int32(height) + 2, np.int32(lenght) + 2

        # translasi = np.eye(3)
        # translasi[0,2] = -xmin_g
        # translasi[1,2] = -ymin_g

        # for img_name in self.dict_image:
        #    instance = self.dict_image[img_name]
        #    instance.update(translasi,shapes)
        
        # return shapes
         
        xmin_g = float('inf')
        ymin_g = float('inf')
        xmax_g = float('-inf')
        ymax_g = float('-inf')
        
        for img_name, img in self.dict_image.items():
            corners = np.float32([[0, 0], [0, img.shapes[0]], [img.shapes[1], img.shapes[0]], [img.shapes[1], 0]]).reshape(-1, 1, 2)
            transformed_corners = StitchUtils.warp_point(corners, img.homography)
            
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