#!/usr/bin/env python3
"""
Node de detection YOLO + Depth pour simulation Gazebo.

Ce node :
- Subscribe a /oakd/rgb/image_raw, /oakd/depth/image_raw, /oakd/rgb/camera_info
- Synchronise RGB+depth (approx) avec message_filters
- Lance YOLO sur GPU
- Prend la mediane de profondeur dans une petite ROI au centre de la bbox
- Publie /target_object_pose

Test:
    ros2 run tb3_autonomy sim_yolo_depth --ros-args \
        -p weights:=/chemin/vers/best.pt \
        -p device:=0 \
        -p conf:=0.5 \
        -p debug_view:=true

Dependances:
    pip install ultralytics
    # Assure-toi d'avoir torch CUDA (sinon ca tourne en CPU)
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from visualization_msgs.msg import Marker

class SimYoloDepthNode(Node):
    def __init__(self):
        super().__init__("sim_yolo_depth_node")
        self.bridge = CvBridge()

        # Params
        self.declare_parameter("weights", "best.pt")  # ton yolo11n finetune
        # self.declare_parameter("device", 0)  # 0 = GPU, "cpu" sinon
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf", 0.5)
        self.declare_parameter(
            "max_rate_hz", 15.0
        )  # throttle (15 Hz suffit au supervisor)
        self.declare_parameter("debug_view", True)

        self.weights = self.get_parameter("weights").value
        # self.device = self.get_parameter("device").value
        self.device = str(self.get_parameter("device").value)
        self.conf = float(self.get_parameter("conf").value)
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self.debug_view = bool(self.get_parameter("debug_view").value)

        # YOLO
        self.model = YOLO(self.weights)

        # Camera intrinsics
        self.camera_info = None

        # Publisher (contrat du supervisor)
        self.pub = self.create_publisher(PoseStamped, "/target_object_pose", 10)

        # Subscriptions
        # self.create_subscription(CameraInfo, "/oakd/rgb/camera_info", self.info_cb, 10)
        self.create_subscription(CameraInfo, "/rgb_camera/camera_info", self.info_cb, 10)

        # rgb_sub = Subscriber(self, Image, "/oakd/rgb/image_raw")
        rgb_sub = Subscriber(self, Image, "/rgb_camera/image")
        # depth_sub = Subscriber(self, Image, "/oakd/depth/image_raw")
        depth_sub = Subscriber(self, Image, "/depth_camera/depth/image")

        # Sync approx RGB + depth
        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.08
        )
        self.sync.registerCallback(self.synced_cb)

        self._last = 0.0
        self.get_logger().info(
            "SimYoloDepthNode ready (Gazebo RGB+Depth -> YOLO -> /target_object_pose)."
        )
        self.pub = self.create_publisher(PoseStamped, "/target_object_pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/detected_object_marker", 10)

    def info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg

    @staticmethod
    def median_depth_m(depth_m, u, v, roi=10):
        """Calcule la mediane de profondeur dans une ROI autour de (u, v)."""
        h, w = depth_m.shape
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))
        x1, x2 = max(0, u - roi), min(w, u + roi)
        y1, y2 = max(0, v - roi), min(h, v + roi)
        patch = depth_m[y1:y2, x1:x2]
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0.0]
        if patch.size == 0:
            return 0.0
        return float(np.median(patch))

    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        if self.camera_info is None:
            return

        # Throttle
        now = time.time()
        if now - self._last < (1.0 / max(self.max_rate_hz, 1e-3)):
            return
        self._last = now

        # Convert RGB
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        # Depth in Gazebo: souvent 32FC1 = metres
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        # Inference YOLO (Ultralytics)
        results = self.model.predict(
            source=rgb, device=self.device, conf=self.conf, verbose=False
        )
        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            if self.debug_view:
                cv2.imshow("SIM YOLO + Depth", rgb)
                cv2.waitKey(1)
            return

        # Meilleure box (score max)
        confs = r.boxes.conf.cpu().numpy()
        best_i = int(np.argmax(confs))
        box = (
            r.boxes.xyxy[best_i].cpu().numpy()
        )  # [x1,y1,x2,y2] en coords image originale
        score = float(confs[best_i])

        x1, y1, x2, y2 = box
        u = int(0.5 * (x1 + x2))
        v = int(0.5 * (y1 + y2))

        # Si depth et rgb n'ont pas la meme taille, on scale (robuste)
        dh, dw = depth.shape[:2]
        rh, rw = rgb.shape[:2]
        u_d = int(u * (dw / rw))
        v_d = int(v * (dh / rh))

        z = self.median_depth_m(depth, u_d, v_d, roi=10)
        if not (0.10 < z < 3.0):
            return

        # Pinhole avec CameraInfo
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Convention optique: +x droite, +y bas, +z avant
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z

        out = PoseStamped()
        out.header.stamp = rgb_msg.header.stamp
        out.header.frame_id = "oak_d_pro_depth_optical_frame"

        out.pose.position.x = float(X)
        out.pose.position.y = float(Y)
        out.pose.position.z = float(Z)
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

        marker = Marker()
        marker.header = out.header
        marker.ns = "test"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker.pose = out.pose

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000

        self.marker_pub.publish(marker)

        if self.debug_view:
            cv2.rectangle(rgb, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(rgb, (u, v), 4, (0, 255, 0), -1)
            cv2.putText(
                rgb,
                f"z={Z:.2f}m conf={score:.2f}",
                (int(x1), max(0, int(y1) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.imshow("SIM YOLO + Depth", rgb)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = SimYoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
