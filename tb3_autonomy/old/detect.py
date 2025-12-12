# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# # Note: Tu devras installer ultralytics: pip install ultralytics
# from ultralytics import YOLO
#
# class SimuAI(Node):
#     def __init__(self):
#         super().__init__('simu_ai_node')
#         # On �coute l'image couleur de la simulation
#         self.subscription = self.create_subscription(
#             Image,
#             '/oakd/rgb/image_raw',
#             self.listener_callback,
#             10)
#         self.bridge = CvBridge()
#         self.model = YOLO("yolov8n.pt") # Charge un petit mod�le YOLO
#
#     def listener_callback(self, msg):
#         # 1. Convertir ROS Image -> OpenCV Image
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#
#         # 2. Faire la d�tection (Simule le VPU de la OAK-D)
#         results = self.model(cv_image)
#
#         # 3. Afficher le r�sultat dans une fen�tre
#         annotated_frame = results[0].plot()
#         cv2.imshow("Vision IA Simulation", annotated_frame)
#         cv2.waitKey(1)
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = SimuAI()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()