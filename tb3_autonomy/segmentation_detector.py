#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import depthai as dai
import blobconverter


class SegmentationDetector(Node):
    def __init__(self):
        super().__init__('segmentation_detector')

        # --- Paramètres du modèle ---
        self.declare_parameter('nn_size', 256)
        self.declare_parameter('num_classes', 2)  # Personne/fond par défaut
        self.declare_parameter('target_class', 1)  # Classe à détecter (1 = personne)
        self.declare_parameter('min_area', 500)    # Surface minimale en pixels
        
        self.nn_size = self.get_parameter('nn_size').value
        self.num_classes = self.get_parameter('num_classes').value
        self.target_class = self.get_parameter('target_class').value
        self.min_area = self.get_parameter('min_area').value

        # --- Variables pour caméra info (pour calcul 3D) ---
        self.camera_intrinsics = None
        self.bridge = CvBridge()

        # --- Publication ROS2 (même interface que object_detector.py) ---
        self.pub_target = self.create_publisher(PoseStamped, '/target_object_pose', 10)
        self.pub_mask = self.create_publisher(Image, '/segmentation/mask', 10)
        self.pub_overlay = self.create_publisher(Image, '/segmentation/overlay', 10)

        # --- Pipeline DepthAI ---
        self.device = None
        self.pipeline = self._create_pipeline()
        self._start_device()

        # Timer pour traiter les frames
        self.create_timer(0.033, self.process_frame)  # ~30 FPS

        self.get_logger().info("🚀 Détecteur par segmentation DeepLabV3+ démarré !")
        self.get_logger().info(f"   - Modèle: DeepLabV3+MobileNetV2 ({self.nn_size}x{self.nn_size})")
        self.get_logger().info(f"   - Classe cible: {self.target_class}")

    def _create_pipeline(self):
        """Crée le pipeline DepthAI avec RGB + Depth + Neural Network"""
        pipeline = dai.Pipeline()

        # === Caméra RGB ===
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self.nn_size, self.nn_size)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # === Caméras Stéréo pour Profondeur ===
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # === StereoDepth avec Alignement RGB ===
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Aligner sur RGB
        stereo.setOutputSize(self.nn_size, self.nn_size)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # === Réseau de Neurones ===
        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(blobconverter.from_zoo(
            name="deeplab_v3_mnv2_256x256",
            zoo_type="depthai",
            shaves=6
        ))
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        cam_rgb.preview.link(nn.input)

        # === Sorties XLink ===
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        nn.out.link(xout_nn.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline

    def _start_device(self):
        """Démarre le device DepthAI et récupère les queues"""
        try:
            self.device = dai.Device(self.pipeline)
            self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self.q_nn = self.device.getOutputQueue("nn", maxSize=4, blocking=False)
            self.q_depth = self.device.getOutputQueue("depth", maxSize=4, blocking=False)

            # Récupérer les intrinsèques caméra
            calib = self.device.readCalibration()
            intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.nn_size, self.nn_size)
            self.camera_intrinsics = {
                'fx': intrinsics[0][0],
                'fy': intrinsics[1][1],
                'cx': intrinsics[0][2],
                'cy': intrinsics[1][2]
            }
            self.get_logger().info(f"📷 Intrinsèques caméra: fx={self.camera_intrinsics['fx']:.1f}, fy={self.camera_intrinsics['fy']:.1f}")

        except Exception as e:
            self.get_logger().error(f"❌ Erreur initialisation OAK-D: {e}")
            raise

    def process_frame(self):
        """Traite une frame : segmentation + profondeur + publication pose 3D"""
        if self.device is None or self.camera_intrinsics is None:
            return

        # Récupérer les données
        in_rgb = self.q_rgb.tryGet()
        in_nn = self.q_nn.tryGet()
        in_depth = self.q_depth.tryGet()

        if in_nn is None or in_depth is None:
            return

        # --- 1. Décoder le masque de segmentation ---
        output = np.array(in_nn.getFirstLayerFp16())
        output = output.reshape(self.num_classes, self.nn_size, self.nn_size)
        mask = np.argmax(output, axis=0).astype(np.uint8)

        # --- 2. Extraire les objets de la classe cible ---
        binary_mask = (mask == self.target_class).astype(np.uint8) * 255
        
        # Publier le masque brut
        mask_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        mask_msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        self.pub_mask.publish(mask_msg)

        # --- 3. Trouver le plus grand contour (objet principal) ---
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < self.min_area:
            return  # Objet trop petit

        # --- 4. Calculer le centroïde ---
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # --- 5. Récupérer la profondeur au centroïde ---
        depth_frame = in_depth.getFrame()
        depth_value = float(depth_frame[cy, cx]) / 1000.0  # mm -> m

        if depth_value < 0.1 or depth_value > 10.0:
            return  # Profondeur invalide

        # --- 6. Calcul de la pose 3D (comme object_detector.py) ---
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx_cam = self.camera_intrinsics['cx']
        cy_cam = self.camera_intrinsics['cy']

        X = (cx - cx_cam) * depth_value / fx
        Y = (cy - cy_cam) * depth_value / fy
        Z = depth_value

        # --- 7. Publication PoseStamped (interface compatible supervisor_node.py) ---
        pose_msg = PoseStamped()
        pose_msg.header = mask_msg.header
        pose_msg.pose.position.x = X
        pose_msg.pose.position.y = Y
        pose_msg.pose.position.z = Z
        pose_msg.pose.orientation.w = 1.0

        self.pub_target.publish(pose_msg)
        self.get_logger().info(f"🎯 Objet détecté à X={X:.2f}, Y={Y:.2f}, Z={Z:.2f} (aire={area:.0f}px)")

        # --- 8. Visualisation (overlay) ---
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            
            # Dessiner le contour
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Z: {depth_value:.2f}m", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            overlay_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            overlay_msg.header = mask_msg.header
            self.pub_overlay.publish(overlay_msg)

    def destroy_node(self):
        """Nettoyage propre"""
        if self.device is not None:
            self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()