#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import sys
import numpy as np
import yaml

class ImagePublisher(Node):
    def __init__(self, file_path, calibration_file=None):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(file_path)
        self.pub = self.create_publisher(Image, "/video_frames", 10)
        self.pub_undistorted = self.create_publisher(Image, "/fine_frame", 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, "/camera_info", 10)
        # Create the set_camera_info service
        self.set_camera_info_service = self.create_service(
            SetCameraInfo,
            '/camera/set_camera_info',
            self.set_camera_info_callback
        )

        if calibration_file:
            self.load_camera_info(calibration_file)
        

    def set_camera_info_callback(self, request, response):
        # Load the camera calibration parameters from the .yaml file
        try:
            with open(request.camera_info.name, 'r') as stream:
                calibration_data = yaml.safe_load(stream)
                self.camera_info = CameraInfo()
                self.camera_info.header.frame_id = "camera"
                self.camera_info.width = calibration_data['image_width']
                self.camera_info.height = calibration_data['image_height']
                self.camera_info.k = calibration_data['camera_matrix']['data']
                self.camera_info.d = calibration_data['distortion_coefficients']['data']
                self.camera_info.r = calibration_data['rectification_matrix']['data']
                self.camera_info.p = calibration_data['projection_matrix']['data']
                self.camera_info.distortion_model = calibration_data['distortion_model']
                #self.pub_camera_info = self.create_publisher(CameraInfo, "/camera_info", 10)
                
                response.success = True
                response.status_message = "Camera calibration parameters set"
        except yaml.YAMLError as exc:
            response.success = False
            response.status_message = str(exc)
        return response
        
    def load_camera_info(self, calibration_file):
        with open(calibration_file, 'r') as stream:
            try:
                calibration_data = yaml.safe_load(stream)
                self.camera_info = CameraInfo()
                self.camera_info.header.frame_id = "camera"
                self.camera_info.width = calibration_data['image_width']
                self.camera_info.height = calibration_data['image_height']
                self.camera_info.k = calibration_data['camera_matrix']['data']
                self.camera_info.d = calibration_data['distortion_coefficients']['data']
                self.camera_info.r = calibration_data['rectification_matrix']['data']
                self.camera_info.p = calibration_data['projection_matrix']['data']
                self.camera_info.distortion_model = calibration_data['distortion_model']
                self.pub_camera_info = self.create_publisher(CameraInfo, "/camera_info", 10)
                print("Publishing camera info...") # Debug print statement
                self.pub_camera_info.publish(self.camera_info)
            except yaml.YAMLError as exc:
                calibration_data = yaml.safe_load(stream)
                self.camera_info = CameraInfo()
                self.camera_info.header.frame_id = "camera"
                self.camera_info.width = 1280
                self.camera_info.height = 720
                self.camera_info.k = [1015.92294,    0.     ,  628.08821, 
                			0.     , 1016.99929 , 323.29014,
                			0. ,    0. , 1.  ]
                self.camera_info.d = [0.060649, -0.199074, -0.010716, -0.005530, 0.000000]
                self.camera_info.r = [1., 0., 0.,
                			0., 1., 0.,
			                0., 0., 1.]
                self.camera_info.p = [1007.22241,    0.     ,  618.95743,    0.     ,
                			 0.     , 1021.54712,  317.6725 ,    0.     ,
                			 0.     ,    0.     ,    1.     ,    0.     ]
                self.camera_info.distortion_model = plumb_bob
                self.pub_camera_info = self.create_publisher(CameraInfo, "/camera_info", 10)
                print("Publishing camera info...") # Debug print statement
                self.pub_camera_info.publish(self.camera_info)
                #print(exc)

    def run(self):
        while(self.cap.isOpened()):
            ret, frame = self.cap.read()
            if ret:
                if hasattr(self, 'camera_info'):
                    self.pub_camera_info.publish(self.camera_info)
                    camera_matrix = np.array(self.camera_info.k, dtype=np.float32).reshape(3, 3)
                    dist_coeffs = np.array(self.camera_info.d, dtype=np.float32).reshape(1, 5)
                    
                    # Now, use camera_matrix and dist_coeffs in the cv2.undistort function
                    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
                    self.pub_camera_info.publish(self.camera_info)
                    
                    self.pub_undistorted.publish(self.bridge.cv2_to_imgmsg(undistorted_frame, "bgr8"))
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    if(len(sys.argv) < 2 or len(sys.argv) > 3):
        print("Incorrect number of arguments\nUsage:\n\tpython3 <path_to_video_file> [<path_to_calibration_file>]")
        exit()

    if not os.path.isfile(sys.argv[1]):
        print("Invalid video file path")
        exit()

    calibration_file = sys.argv[2] if len(sys.argv) == 3 else None

    ip = ImagePublisher(sys.argv[1], calibration_file)
    print("Publishing...")
    ip.run()

    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()