#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge 
import cv2 

class CameraAIProcessor(Node):
    def __init__(self):
        super().__init__('camera_ai_processor')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
            
        self.detection_pub = self.create_publisher(PoseStamped, '/object_detection/pose', 10)
        
    def image_callback(self, msg):
        #30fois par seconde
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_image = cv2.resize(cv_image, (640, 640))
            

            cv2.imshow("Vue du Robot", cv_image)
            cv2.waitKey(1) 

        except Exception as e:
            self.get_logger().error(f'Erreur conversion image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraAIProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()