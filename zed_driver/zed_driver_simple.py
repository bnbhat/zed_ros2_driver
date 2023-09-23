#-----------------------------------------------------------------------------------
# filename: zed_driver_simple.py
# description: simple ROS2 Driver for ZED Camera
# author: Balachandra Bhat <bnbhat311@gmail.com>
# created: 2023-09-01
# last modified: 2023-09-23
#-----------------------------------------------------------------------------------

import cv2
import sys
import pyzed.sl as sl
import numpy as np
import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from arc_interfaces.msg import ArcHumanPose

cam2World = np.array([[0.368484,  0.532548,  -0.76354,       0.7],
                    [0.92846,  -0.14931,   0.34227,     0.303],
                    [0.067548, -0.834014,  -0.55005,     1.059],
                    [0,         0,         0,         1]])

def parse_args(init):
    if len(opt.input_svo_file)>0 and opt.input_svo_file.endswith(".svo"):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address)>0 :
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
            init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
            print("[Sample] Using Stream input, IP : ",ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
            init.set_from_stream(ip_str)
            print("[Sample] Using Stream input, IP : ",ip_str)
        else :
            print("Unvalid IP format. Using live stream")
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
        print("[Sample] Using Camera in resolution HD2K")
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
        print("[Sample] Using Camera in resolution HD1200")
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
        print("[Sample] Using Camera in resolution HD1080")
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
        print("[Sample] Using Camera in resolution HD720")
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
        print("[Sample] Using Camera in resolution SVGA")
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA
        print("[Sample] Using Camera in resolution VGA")
    elif len(opt.resolution)>0: 
        print("[Sample] No valid resolution entered. Using default")
    else : 
        print("[Sample] Using default resolution")


def publish_image(publisher, bridge, image):
    try:
        #print(type(image))
        image_msg = bridge.cv2_to_imgmsg(image)
        publisher.publish(image_msg)
    except Exception as e:
        print('Error publishing image:', e)

def publish_keypoints(publisher, node, keypoints):

    print('Publishing keypointss')

    try:
        points_list = []  
        for kp in keypoints:
            if np.isnan(kp).any():
                point = Point()
                point.x = 0.0
                point.y = 0.0
                point.z = 0.0
            else:
                # Convert the keypoint to homogeneous coordinates
                kp_homogeneous = np.array([kp[0], kp[1], kp[2], 1.0])

                # Transform the point using the camera-to-world transformation matrix
                transformed_kp = cam2World @ kp_homogeneous

                # Convert back to 3D from homogeneous coordinates
                point = Point()
                point.x = float(transformed_kp[0])
                point.y = float(transformed_kp[1])
                point.z = float(transformed_kp[2])

            points_list.append(point)

        msg = ArcHumanPose()
        msg.type = 18
        msg.time_stamp = node.get_clock().now().to_msg() 
        msg.key_points = points_list
        publisher.publish(msg)
    except Exception as e:
        print('Error publishing image:', e)     


def main():
    rclpy.init()
    node = Node('zed_ros_driver')
    image_publisher = node.create_publisher(Image, '/zed/image', 10)
    body_publisher = node.create_publisher(ArcHumanPose, "/arc_rt_human_pose", 1) 
    bridge = CvBridge()

    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD1080 video mode
    init_params.coordinate_units = sl.UNIT.METER          # Set coordinate units
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    
    parse_args(init_params)

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Enable Positional tracking (mandatory for object detection)
    positional_tracking_parameters = sl.PositionalTrackingParameters()
    # If the camera is static, uncomment the following line to have better performances
    # positional_tracking_parameters.set_as_static = True
    zed.enable_positional_tracking(positional_tracking_parameters)
    
    body_param = sl.BodyTrackingParameters()
    body_param.enable_tracking = True                # Track people across images flow
    body_param.enable_body_fitting = False            # Smooth skeleton move
    body_param.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST 
    body_param.body_format = sl.BODY_FORMAT.BODY_18  # Choose the BODY_FORMAT you wish to use

    # Enable Object Detection module
    zed.enable_body_tracking(body_param)

    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    body_runtime_param.detection_confidence_threshold = 40

    # Get ZED camera information
    camera_info = zed.get_camera_information()
    # 2D viewer utilities
    display_resolution = sl.Resolution(min(camera_info.camera_configuration.resolution.width, 1280), min(camera_info.camera_configuration.resolution.height, 720))
    image_scale = [display_resolution.width / camera_info.camera_configuration.resolution.width
                 , display_resolution.height / camera_info.camera_configuration.resolution.height]


    # Create ZED objects filled in the main loop
    bodies = sl.Bodies()
    image = sl.Mat()
    key_wait = 10 
    while rclpy.ok():
        # Grab an image
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
            zed.retrieve_bodies(bodies, body_runtime_param)
            image_left_ocv = image.get_data()
            publish_image(image_publisher, bridge, image_left_ocv)

            prime_body = None
            if len(bodies.body_list) > 0:
                for body in bodies.body_list:
                    if body.keypoint is None:
                        continue

                    if prime_body is None:
                        prime_body = body
                    elif body.position[2] < prime_body.position[2]:
                        prime_body = body
                publish_keypoints(body_publisher, node, prime_body.keypoint)

    image.free(sl.MEM.CPU)
    zed.disable_body_tracking()
    zed.disable_positional_tracking()
    zed.close()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
    parser.add_argument('--ip_address', type=str, help='IP Adress, in format a.b.c.d:port or a.b.c.d, if you have a streaming setup', default = '')
    parser.add_argument('--resolution', type=str, help='Resolution, can be either HD2K, HD1200, HD1080, HD720, SVGA or VGA', default = '')
    opt = parser.parse_args()
    if len(opt.input_svo_file)>0 and len(opt.ip_address)>0:
        print("Specify only input_svo_file or ip_address, or none to use wired camera, not both. Exit program")
        exit()
    main() 