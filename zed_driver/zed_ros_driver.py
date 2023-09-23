#-----------------------------------------------------------------------------------
# file: zed_ros_driver.py
# description: ROS2 Driver for ZED Camera
# author: Balachandra Bhat <bnbhat311@gmail.com>
# created: 2023-09-23
# modified: 2023-09-23
#-----------------------------------------------------------------------------------

import argparse
import numpy as np
import pyzed.sl as sl
import cv2
import rclpy
from rclpy.node import Node
from annotator import Annotator
from utils import calculate_fps
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from arc_interfaces.msg import ArcHumanPose

cam2base_link = np.array([[0.368484,  0.532548,  -0.76354,       0.7],
                    [0.92846,  -0.14931,   0.34227,     0.303],
                    [0.067548, -0.834014,  -0.55005,     1.059],
                    [0,         0,         0,         1]])


class ZedDriver(Node):
    def __init__(self, options):
        super().__init__('zed_ros_driver')
        self.init_camera(options)
        self.init_ros()
        self.annotator = Annotator()
        self.fps = 0.0
    
    def init_ros(self):
        self.image_publisher = self.create_publisher(Image, '/zed/image', 10)
        self.body_publisher = self.create_publisher(ArcHumanPose, "/arc_rt_human_pose", 1)
        self.bridge = CvBridge()

    def init_camera(self, options):
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.coordinate_units = sl.UNIT.METER    
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

        self.set_options(options)

        # Open the camera
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print('[ZED Driver] failed to open camera',repr(err))
            exit(1)
    
        self.positional_tracking_parameters= sl.PositionalTrackingParameters()
        #self.positional_tracking_parameters.set_as_static = True
        self.zed.enable_positional_tracking(self.positional_tracking_parameters)
        
        self.body_param = sl.BodyTrackingParameters()
        self.body_param.enable_tracking = True                # Track people across images flow
        self.body_param.enable_body_fitting = False            # Smooth skeleton move
        self.body_param.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST 
        self.body_param.body_format = sl.BODY_FORMAT.BODY_18  # Choose the BODY_FORMAT you wish to use

        # Enable Object Detection module
        self.zed.enable_body_tracking(self.body_param)        

        self.body_runtime_param = sl.BodyTrackingRuntimeParameters()
        self.body_runtime_param.detection_confidence_threshold = 40

        # Get ZED camera information
        self.camera_info = self.zed.get_camera_information()
        
        # 2D viewer utilities
        self.display_resolution = sl.Resolution(min(self.camera_info.camera_configuration.resolution.width, 1280), min(self.camera_info.camera_configuration.resolution.height, 720))
        self.image_scale = [self.display_resolution.width / self.camera_info.camera_configuration.resolution.width
                    , self.display_resolution.height / self.camera_info.camera_configuration.resolution.height]
        
        self.bodies = sl.Bodies()
        self.image = sl.Mat()

    def set_options(self, options):
        if options:
            if len(options.input_svo_file)>0 and options.input_svo_file.endswith(".svo"):
                self.init_params.set_from_svo_file(options.input_svo_file)
                print("[Zed Driver] Using SVO File input: {0}".format(options.input_svo_file))
            elif len(options.ip_address)>0 :
                ip_str = options.ip_address
                if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
                    self.init_params.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
                    print("[Zed Driver] Using Stream input, IP : ",ip_str)
                elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
                    self.init_params.set_from_stream(ip_str)
                    print("[Zed Driver] Using Stream input, IP : ",ip_str)
                else :
                    print("Unvalid IP format. Using live stream")
            if ("HD2K" in options.resolution):
                self.init_params.camera_resolution = sl.RESOLUTION.HD2K
                print("[Zed Driver] Using Camera in resolution HD2K")
            elif ("HD1200" in options.resolution):
                self.init_params.camera_resolution = sl.RESOLUTION.HD1200
                print("[Zed Driver] Using Camera in resolution HD1200")
            elif ("HD1080" in options.resolution):
                self.init_params.camera_resolution = sl.RESOLUTION.HD1080
                print("[Zed Driver] Using Camera in resolution HD1080")
            elif ("HD720" in options.resolution):
                self.init_params.camera_resolution = sl.RESOLUTION.HD720
                print("[Zed Driver] Using Camera in resolution HD720")
            elif ("SVGA" in options.resolution):
                self.init_params.camera_resolution = sl.RESOLUTION.SVGA
                print("[Zed Driver] Using Camera in resolution SVGA")
            elif ("VGA" in options.resolution):
                self.init_params.camera_resolution = sl.RESOLUTION.VGA
                print("[Zed Driver] Using Camera in resolution VGA")
            elif len(options.resolution)>0: 
                print("[Zed Driver] No valid resolution entered. Using default")
            else : 
                print("[Zed Driver] Using default resolution")


    def publish_image(self, image, prime_body):
        try:
            self.annotator.render_image(image, self.image_scale, self.bodies.body_list, True, sl.BODY_FORMAT.BODY_18)
            self.annotator.render_prime_body(image, self.image_scale, prime_body, True, sl.BODY_FORMAT.BODY_18)
            self.annotator.render_fps(image, self.fps)
            image_msg = self.bridge.cv2_to_imgmsg(image)
            self.image_publisher.publish(image_msg)
        except Exception as e:
            print('Error publishing image:', e)

    def publish_keypoints(self, keypoints):
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
                    transformed_kp = cam2base_link @ kp_homogeneous
                    # Convert back to 3D from homogeneous coordinates
                    point = Point()
                    point.x = float(transformed_kp[0])
                    point.y = float(transformed_kp[1])
                    point.z = float(transformed_kp[2])
                points_list.append(point)

            msg = ArcHumanPose()
            msg.type = 18
            msg.time_stamp = self.get_clock().now().to_msg() 
            msg.key_points = points_list
            self.body_publisher.publish(msg)
        except Exception as e:
            print('Error publishing image:', e)  

    def run(self):
        while rclpy.ok():
            start = self.get_clock().now()
            if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU, self.display_resolution)
                self.zed.retrieve_bodies(self.bodies, self.body_runtime_param)
                image_left_ocv = self.image.get_data()

                prime_body = None
                if len(self.bodies.body_list) > 0:
                    for body in self.bodies.body_list:
                        if body.keypoint is None:
                            continue

                        if prime_body is None:
                            prime_body = body
                        elif body.position[2] < prime_body.position[2]:
                            prime_body = body
                    self.publish_keypoints(prime_body.keypoint)
                
                self.publish_image(image_left_ocv, prime_body)
                self.fps = 1.0 / (self.get_clock().now() - start).nanoseconds * 1e9

    def __del__(self):
        if hasattr(self, 'zed'):
            self.image.free(sl.MEM.CPU)
            self.zed.disable_object_detection()
            self.zed.disable_positional_tracking()
            self.zed.close()
            cv2.destroyAllWindows()

def arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
    parser.add_argument('--ip_address', type=str, help='IP Adress, in format a.b.c.d:port or a.b.c.d, if you have a streaming setup', default = '')
    parser.add_argument('--resolution', type=str, help='Resolution, can be either HD2K, HD1200, HD1080, HD720, SVGA or VGA', default = '')
    opt = parser.parse_args()
    if len(opt.input_svo_file)>0 and len(opt.ip_address)>0:
        print("Specify only input_svo_file or ip_address, or none to use wired camera, not both. Exit program")
        exit()
    return opt

def main(options=None):
    rclpy.init()
    zed_driver_node = ZedDriver(options)
    zed_driver_node.run()


if __name__ == "__main__":
    options = arg_parser()
    main(options)