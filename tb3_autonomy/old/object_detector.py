# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PoseStamped
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
#
#
# class ObjectDetector(Node):
#     def __init__(self):
#         super().__init__('object_detector')
#
#         self.bridge = CvBridge()
#
#         # --- Paramètres de détection (Rouge par défaut) ---
#         # Plage HSV pour le rouge (le rouge est au début et à la fin du spectre HSV)
#         self.lower_red1 = np.array([0, 100, 100])
#         self.upper_red1 = np.array([10, 255, 255])
#         self.lower_red2 = np.array([160, 100, 100])
#         self.upper_red2 = np.array([180, 255, 255])
#
#         # --- Variables de stockage ---
#         self.last_depth_image = None
#         self.camera_info = None
#
#         # --- Abonnements ---
#         # On s'abonne à la caméra Info pour avoir les maths de la caméra (focale, centre)
#         self.create_subscription(CameraInfo, '/rgb_camera/camera_info', self.info_callback, 10)
#
#         # On s'abonne à la profondeur
#         #/depth_camera/depth/image_raw
#         self.create_subscription(Image, '/depth_camera/depth/image_raw', self.depth_callback, 10)
#
#         # On s'abonne à l'image couleur (C'est elle qui déclenche le calcul)
#         self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, 10)
#
#         # --- Publication ---
#         # On publie la position de l'objet détecté
#         self.pub_target = self.create_publisher(PoseStamped, '/target_object_pose', 10)
#
#         self.get_logger().info("Detecteur d'objets (SIMULATION) démarré !")
#
#     def info_callback(self, msg):
#         # On récupère les infos optiques une seule fois
#         if self.camera_info is None:
#             self.camera_info = msg
#
#     def depth_callback(self, msg):
#         try:
#             # On stocke l'image de profondeur pour l'utiliser quand on reçoit une image couleur
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#             self.last_depth_image = np.nan_to_num(cv_image, posinf=0.0, neginf=0.0)
#         except Exception as e:
#             self.get_logger().error(f"Erreur depth: {e}")
#
#     def image_callback(self, msg):
#         if self.last_depth_image is None or self.camera_info is None:
#             return
#
#         try:
#             # 1. Conversion ROS -> OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#
#             # 2. Conversion en HSV
#             hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#
#             # 3. Masque Rouge
#             mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
#             mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
#             mask = mask1 + mask2
#
#             # 4. Contours
#             contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#
#             if contours:
#                 # On prend le plus gros contour
#                 largest_contour = max(contours, key=cv2.contourArea)
#
#                 if cv2.contourArea(largest_contour) > 500:
#                     M = cv2.moments(largest_contour)
#                     if M["m00"] != 0:
#                         cx = int(M["m10"] / M["m00"])
#                         cy = int(M["m01"] / M["m00"])
#
#                         # 5. Récupérer la distance Z (Correction "Bulletproof")
#                         raw_depth = self.last_depth_image[cy, cx]
#                         depth_value = float(np.array(raw_depth).flatten()[0])
#
#                         # --- FILTRE DE SÉCURITÉ ---
#                         # Si la valeur est Infini (inf) ou Pas un Nombre (nan) -> On arrête
#                         if np.isinf(depth_value) or np.isnan(depth_value):
#                             return  # <--- REMPLACÉ 'continue' par 'return'
#
#                         # Si la distance est absurde (> 5m ou < 0.2m) -> On arrête
#                         if depth_value > 3.0 or depth_value < 0.2:
#                             return  # <--- REMPLACÉ 'continue' par 'return'
#
#                         # Si on arrive ici, c'est bon !
#                         self.publish_detection(cx, cy, depth_value, msg.header)
#
#                         # Debug visuel
#                         cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
#                         cv2.putText(cv_image, f"Z: {depth_value:.2f}m", (cx + 10, cy),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#
#             # Affichage
#             cv2.imshow("IA Vision (Simulation)", cv_image)
#             cv2.waitKey(1)
#
#         except Exception as e:
#             self.get_logger().error(f"Erreur processing image: {e}")
#     def publish_detection(self, u, v, depth, header):
#         # 6. Mathématiques : Transformer Pixel (u,v) + Distance (Z) -> Espace 3D (X,Y,Z)
#         # Modèle sténopé (Pinhole Camera Model)
#         fx = self.camera_info.k[0]  # Focale X
#         fy = self.camera_info.k[4]  # Focale Y
#         cx = self.camera_info.k[2]  # Centre optique X
#         cy = self.camera_info.k[5]  # Centre optique Y
#
#         # Formule : X = (u - cx) * Z / fx
#         X = (u - cx) * depth / fx
#         Y = (v - cy) * depth / fy
#         Z = depth
#
#         # Création du message
#         pose_msg = PoseStamped()
#         pose_msg.header = header  # Important: garde le frame_id de la caméra
#         pose_msg.pose.position.x = X
#         pose_msg.pose.position.y = Y
#         pose_msg.pose.position.z = Z
#         # Orientation neutre pour l'instant
#         pose_msg.pose.orientation.w = 1.0
#
#         self.pub_target.publish(pose_msg)
#         self.get_logger().info(f"Objet détecté à X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetector()
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