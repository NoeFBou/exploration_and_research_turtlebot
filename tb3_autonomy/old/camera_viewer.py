#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()

        # Abonnement RGB
        self.sub_rgb = self.create_subscription(
            Image, '/rgb_camera/image_raw', self.rgb_callback, 10)
            
        # Abonnement Depth
        self.sub_depth = self.create_subscription(
            Image, '/depth_camera/image_raw', self.depth_callback, 10)

    def rgb_callback(self, msg):
        try:
            # Conversion ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Vue RGB (OAK-D)", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Erreur RGB: {e}')

    def depth_callback(self, msg):
        try:
            # Conversion ROS -> OpenCV (32FC1 = Float 32 bits, distance en mètres)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # --- Normalisation pour l'affichage ---
            # On remplace les 'inf' (infini) par une distance max (ex: 5m)
            depth_image = np.nan_to_num(depth_image, posinf=5.0, neginf=0.0)

            # Normalisation 0-255 pour affichage (0 = proche/noir, 255 = loin/blanc)
            # On divise par la distance max arbitraire (5m) pour avoir une échelle
            norm_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            norm_image = np.uint8(norm_image)

            # Application d'une colormap pour faire "joli" et lisible (JET = Bleu->Rouge)
            colored_depth = cv2.applyColorMap(norm_image, cv2.COLORMAP_JET)

            cv2.imshow("Vue Profondeur (OAK-D)", colored_depth)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Erreur Depth: {e}')

    # def depth_callback(self, msg):
    #     try:
    #         # Conversion ROS -> OpenCV
    #         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #
    #         # --- ECHELLE FIXE (Plus stable) ---
    #         # On définit une distance max (ex: 3.5 mètres comme le LIDAR)
    #         max_dist = 3.5
    #
    #         # On coupe tout ce qui est plus loin que max_dist
    #         depth_image = np.clip(depth_image, 0.0, max_dist)
    #
    #         # Normalisation : 0m -> 0 (Noir), 3.5m -> 255 (Blanc)
    #         # On multiplie pour mettre à l'échelle 0-255
    #         norm_image = (depth_image / max_dist) * 255.0
    #         norm_image = np.uint8(norm_image)
    #
    #         # Option 1 : Noir et Blanc (Plus réaliste pour un robot)
    #         cv2.imshow("Vue Profondeur (OAK-D)", norm_image)
    #
    #         # Option 2 : Couleur (Si tu préfères, décommente la ligne suivante)
    #         # colored_depth = cv2.applyColorMap(norm_image, cv2.COLORMAP_TURBO) # TURBO est mieux que JET
    #         # cv2.imshow("Vue Profondeur (OAK-D)", colored_depth)
    #
    #         cv2.waitKey(1)
    #     except Exception as e:
    #         self.get_logger().error(f'Erreur Depth: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
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
