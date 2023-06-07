#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2
import pytesseract
from proyecto_interfaces.srv import StartPerceptionTest
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # Image is the message type

bridge = CvBridge()


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
      
#        self.start_perception_test = self.create_service(StartPerceptionTest, '/group_5/start_perception_test_srv', self.start_perception_test_callback)
        self.image_topic = self.create_subscription(
            Image,
            'video_frames',
            self.image_topic_callback,
            100)
        self.image_topic  # prevent unused variable warning

    def image_topic_callback(self, msg):
        print("Received an image!")
        cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
        cv2.imwrite('camera_image.jpeg', cv2_img)

#    def start_perception_test_callback(self, request, response):
#        # Receive the request data and sum it
#        response.string = "Debo identificar el banner a que se encuentra en las coordenadas x_1, y_1 y el banner b que se encuentra en las coordenadas x_2, y_2"
#        # Return the sum as the reply
 #       self.get_logger().info('Incoming request\na: %d b: %d' % (request.banner_a, request.banner_b))
 #       return response

def main(args=None):
 
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()