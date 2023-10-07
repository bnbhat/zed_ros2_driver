#-----------------------------------------------------------------------------------
# file: zed_ros_driver.py
# description: ROS2 Driver for ZED Camera
# author: Balachandra Bhat <bnbhat311@gmail.com>
# created: 2023-09-23
# modified: 2023-10-07
#-----------------------------------------------------------------------------------

import numpy as np
import pyzed.sl as sl

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from zed_driver.annotator import Annotator
from zed_driver.utils import calculate_fps
from zed_driver.config_builder import Config, ConfigBuilder
from zed_driver.arg_parser import arg_parser
from arc_interfaces.msg import ArcHumanPose

class ZedDriver(Node):
    def __init__(self, config: Config) -> None:
        super().__init__('zed_ros_driver')
        self.config = config
        self.init_camera()
        self.init_ros()
        self.annotator = Annotator()
        self.fps = 0.0

    def init_ros(self):
        self.image_publisher = self.create_publisher(Image, '/zed/image', 10)
        self.body_publisher = self.create_publisher(ArcHumanPose, "/arc_rt_human_pose", 1)
        self.bridge = CvBridge()
        print('[ZED Driver] ROS Initialized')

    def init_camera(self):
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = self.config.resolution
        self.init_params.coordinate_units = sl.UNIT.METER    
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        
        if self.config.input_svo_file is not None:
            self.init_params.set_from_svo_file(self.config.input_svo_file)
        
        if self.config.ip_address is not None:
            self.init_params.set_from_stream(self.config.ip_address)
        

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
        self.body_param.detection_model = self.config.detection_model
        self.body_param.body_format = self.config.body_format

        # Enable Object Detection module
        self.zed.enable_body_tracking(self.body_param)        

        self.body_runtime_param = sl.BodyTrackingRuntimeParameters()
        self.body_runtime_param.detection_confidence_threshold = self.config.detection_confidence

        # Get ZED camera information
        self.camera_info = self.zed.get_camera_information()
        
        # 2D viewer utilities
        self.display_resolution = sl.Resolution(min(self.camera_info.camera_configuration.resolution.width, 1280), min(self.camera_info.camera_configuration.resolution.height, 720))
        self.image_scale = [self.display_resolution.width / self.camera_info.camera_configuration.resolution.width
                    , self.display_resolution.height / self.camera_info.camera_configuration.resolution.height]
        
        self.bodies = sl.Bodies()
        self.image = sl.Mat()
        print('[ZED Driver] Camera Initialized')

    def publish_image(self, image, prime_body):
        try:
            self.annotator.render_image(image, self.image_scale, self.bodies.body_list, True, self.config.body_format)
            self.annotator.render_prime_body(image, self.image_scale, prime_body, True, self.config.body_format)
            self.annotator.render_fps(image, self.fps)
            image_msg = self.bridge.cv2_to_imgmsg(image)
            self.image_publisher.publish(image_msg)
        except Exception as e:
            print('[ZED Driver] Error publishing image:', e)

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
                    transformed_kp = self.config.Tcw @ kp_homogeneous
                    # Convert back to 3D from homogeneous coordinates
                    point = Point()
                    point.x = float(transformed_kp[0])
                    point.y = float(transformed_kp[1])
                    point.z = float(transformed_kp[2])
                points_list.append(point)

            msg = ArcHumanPose()
            msg.type = self.config.body_format_int_value
            msg.time_stamp = self.get_clock().now().to_msg() 
            msg.key_points = points_list
            self.body_publisher.publish(msg)
        except Exception as e:
            print('[ZED Driver] Error publishing image:', e)  

    def run(self):
        print('[ZED Driver] Running...')
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
            if self.zed.is_opened():
                self.image.free(sl.MEM.CPU)
                self.zed.disable_object_detection()
                self.zed.disable_positional_tracking()
                self.zed.close()

def main(args=None):
    rclpy.init()
    config = ConfigBuilder.get_config(filename="config.yaml", args=args)
    zed_driver_node = ZedDriver(config)
    zed_driver_node.run()


if __name__ == "__main__":
    args = arg_parser()
    main(args)