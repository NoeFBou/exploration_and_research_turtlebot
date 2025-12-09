# # import rclpy
# from rclpy.node import Node
# from stereo_msgs.msg import DisparityImage
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
#
# class DepthView(Node):
#     def __init__(self):
#         super().__init__('depth_visualizer_node')
#
#         self.subscription = self.create_subscription(
#             DisparityImage,
#             '/oakd/disparity',
#             self.listener_callback,
#             10)
#
#         self.bridge = CvBridge()
#         print("Visualiseur de Profondeur demarre ! En attente de donnees...")
#
#     def listener_callback(self, msg):
#         try:
#             ros_image = msg.image
#             cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
#
#             norm_image = cv2.normalize(cv_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
#
#
#             color_depth = cv2.applyColorMap(norm_image, cv2.COLORMAP_JET)
#
#             cv2.imshow("Vision Profondeur Robot (Disparity)", color_depth)
#             cv2.waitKey(1)
#
#         except Exception as e:
#             self.get_logger().error(f'Erreur conversion: {e}')
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = DepthView()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()
#
# if __name__ == '__main__':
#     main()