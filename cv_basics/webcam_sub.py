import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import random as rng
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'video_frames', 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning
        self.br = CvBridge()
        self.pub = self.create_publisher(Image, "/video_frames_imp_proc", 100)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        src = np.copy(current_frame)
        ########################################## the opencv function to be run on the frame
        
        ## Cany Edge detector 
        
        max_lowThreshold = 100
        window_name = 'Edge Map'
        title_trackbar = 'Min Threshold:'
        ratio = 3
        kernel_size = 3
       
        src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        low_threshold = 0
        img_blur = cv.blur(src_gray, (3,3))
        detected_edges = cv.Canny(img_blur, low_threshold, low_threshold*ratio, kernel_size)
        mask = detected_edges != 0
        frame = src * (mask[:,:,None].astype(src.dtype))

        #src[np.all(src == 255, axis=2)] = 0
        #kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]], dtype=np.float32)
        #imgLaplacian = cv.filter2D(src, cv.CV_32F, kernel)
        #sharp = np.float32(src)
        #imgResult = sharp - imgLaplacian
        
        #imgResult = np.clip(imgResult, 0, 255)
        #imgResult = imgResult.astype('uint8')
             
        #frame = cv.GaussianBlur(imgResult, (5, 5), 0)
        #
        
        ################################### ORB detector
        # Initiate ORB detector
        #orb = cv.ORB_create()
        # find the keypoints with ORB
        #kp = orb.detect(src,None)
        # compute the descriptors with ORB
        #kp, des = orb.compute(src, kp)
        # draw only keypoints location,not size and orientation
        #frame = cv.drawKeypoints(src, kp, None, color=(0,255,0), flags=0)
        
        ################################# background substraction
        #backSub = cv.createBackgroundSubtractorMOG2()
        #frame = backSub.apply(frame)
        
        self.pub.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()