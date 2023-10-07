#------------------------------------------------------------------------------
# file: annotator.py
# description: Annotator class for ZED camera
# author: Balachandra Bhat <bnbhat311@gmail.com>
# created: 2023-09-23
# modified: 2023-09-23
#------------------------------------------------------------------------------

import cv2
import numpy as np
import pyzed.sl as sl
from zed_driver.utils import *


class Annotator(object):
    def __init__(self) -> None:
        pass

    #Function that scales point coordinates
    def scale_keypoint(self, pt, scale) -> list:
        out = [pt[0]*scale[0], pt[1]*scale[1]]
        return out

    def render_skeleton(self, left_display, img_scale, obj, color, BODY_BONES) -> None:
        # Draw skeleton bones
        for part in BODY_BONES:
            kp_a = self.scale_keypoint(obj.keypoint_2d[part[0].value], img_scale)
            kp_b = self.scale_keypoint(obj.keypoint_2d[part[1].value], img_scale)
            # Check that the keypoints are inside the image
            if(kp_a[0] < left_display.shape[1] and kp_a[1] < left_display.shape[0] 
            and kp_b[0] < left_display.shape[1] and kp_b[1] < left_display.shape[0]
            and kp_a[0] > 0 and kp_a[1] > 0 and kp_b[0] > 0 and kp_b[1] > 0 ):
                cv2.line(left_display, (int(kp_a[0]), int(kp_a[1])), (int(kp_b[0]), int(kp_b[1])), color, 2, cv2.LINE_AA)

        # Skeleton joints
        for kp in obj.keypoint_2d:
            cv_kp = self.scale_keypoint(kp, img_scale)
            if(cv_kp[0] < left_display.shape[1] and cv_kp[1] < left_display.shape[0]):
                cv2.circle(left_display, (int(cv_kp[0]), int(cv_kp[1])), 4, color, -1)

    def mark_body(self, left_display, img_scale, body, kp) -> None:
        color = [0, 255, 0]
        #draw a dimond arround kp
        cv_kp = self.scale_keypoint(kp, img_scale)
        if(cv_kp[0] < left_display.shape[1] and cv_kp[1] < left_display.shape[0]):
            cv2.circle(left_display, (int(cv_kp[0]), int(cv_kp[1]) + 50), 25, color, -1)

    def render_image(self, left_display, img_scale, objects, is_tracking_on, body_format) -> None:
        '''
        Parameters
            left_display (np.array): numpy array containing image data
            img_scale (list[float])
            objects (list[sl.ObjectData]) 
        '''
        overlay = left_display.copy()

        # Render skeleton joints and bones
        for obj in objects:
            if render_object(obj, is_tracking_on):
                if len(obj.keypoint_2d) > 0:
                    color = generate_color_id_u(obj.id)
                    if body_format == sl.BODY_FORMAT.BODY_18:
                        self.render_skeleton(left_display, img_scale, obj, color, sl.BODY_18_BONES)
                    elif body_format == sl.BODY_FORMAT.BODY_34:
                        self.render_skeleton(left_display, img_scale, obj, color, sl.BODY_34_BONES)
                    elif body_format == sl.BODY_FORMAT.BODY_38:
                        self.render_skeleton(left_display, img_scale, obj, color, sl.BODY_38_BONES) 
        cv2.addWeighted(left_display, 0.9, overlay, 0.1, 0.0, left_display)

    def render_prime_body(self, left_display, img_scale, body, is_tracking_on, body_format) -> None:
        if body is None:
            return
        overlay = left_display.copy()
        if render_object(body, is_tracking_on):
            if len(body.keypoint_2d) > 0:
                if body_format == sl.BODY_FORMAT.BODY_18:
                    self.mark_body(left_display, img_scale, body, body.keypoint_2d[1])
                elif body_format == sl.BODY_FORMAT.BODY_34:
                    self.mark_body(left_display, img_scale, body, body.keypoint_2d[2])
                elif body_format == sl.BODY_FORMAT.BODY_38:
                    self.mark_body(left_display, img_scale, body, body.keypoint_2d[3]) 
        cv2.addWeighted(left_display, 0.9, overlay, 0.1, 0.0, left_display)

    def render_fps(self, img,  fps) -> None:
        cv2.putText(img, f'FPS: {fps:.0f}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

