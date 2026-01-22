#!/usr/bin/env python3
"""
ROS2 node: OAK-D Pro (DepthAI) + YOLO blob + depth -> publish /target_object_pose (PoseStamped)

Ce node est destine au ROBOT REEL avec camera OAK-D Pro physique.
Pour la simulation Gazebo, utiliser sim_yolo_depth_node.py.

Publie:
- pose.position.x : X (m) lateral (droite = +)
- pose.position.y : Y (m) vertical (bas = + si repere optique), non utilise par le supervisor
- pose.position.z : Z (m) profondeur / distance (avant = +)

Build et installation:
    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash

Lancement (sur robot reel):
    ros2 run tb3_autonomy oak_yolo_depth --ros-args \
        -p blob_path:=/chemin/vers/best_openvino_2022.1_6shave.blob \
        -p camera_frame:=oak_d_pro_color_optical_frame \
        -p conf_thres:=0.5 \
        -p debug_view:=true

Test rapide:
    # Verifier que le node publie
    ros2 topic echo /target_object_pose

Dependances:
    pip install depthai opencv-python numpy
"""

import threading
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class OakYoloDepthNode(Node):
    def __init__(self):
        super().__init__("oak_yolo_depth_node")

        # ---------- Params ----------
        self.declare_parameter("blob_path", "")
        self.declare_parameter("camera_frame", "oak_d_pro_color_optical_frame")
        self.declare_parameter("class_name", "red_cube")
        self.declare_parameter("conf_thres", 0.5)
        self.declare_parameter("iou_thres", 0.4)
        self.declare_parameter("input_size", 640)  # YOLO blob input = 640x640
        self.declare_parameter("depth_out_w", 640)
        self.declare_parameter("depth_out_h", 360)
        self.declare_parameter("min_z", 0.10)  # m
        self.declare_parameter("max_z", 3.00)  # m
        self.declare_parameter("debug_view", True)

        blob_path_param = self.get_parameter("blob_path").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.class_name = self.get_parameter("class_name").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.iou_thres = float(self.get_parameter("iou_thres").value)
        self.input_size = int(self.get_parameter("input_size").value)
        self.depth_out_w = int(self.get_parameter("depth_out_w").value)
        self.depth_out_h = int(self.get_parameter("depth_out_h").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.debug_view = bool(self.get_parameter("debug_view").value)

        # Chemin blob (fallback : layout par defaut)
        if blob_path_param:
            self.blob_path = Path(blob_path_param)
        else:
            self.blob_path = (
                Path(__file__).parent.parent
                / "models/red_cube_01/best_openvino_2022.1_6shave.blob"
            )

        if not self.blob_path.exists():
            raise FileNotFoundError(f"Blob introuvable: {self.blob_path}")

        # Publisher attendu par supervisor_node.py
        self.pub_target = self.create_publisher(PoseStamped, "/target_object_pose", 10)

        # Thread de capture (evite de bloquer rclpy.spin)
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

        self.get_logger().info(f"Starting DepthAI pipeline with blob: {self.blob_path}")
        self._thread.start()

    # ------------------- YOLO decode -------------------
    def decode_yolo_v11(self, output_layer):
        """
        Decode YOLO v11 output layer.
        output_layer is a flat fp16 array.
        Expected shape: (4 + num_classes, N) -> transpose -> (N, 4+num_classes)
        Ici on fait 1 classe (red_cube) mais tu peux adapter.
        """
        NUM_CLASSES = 1

        data = (
            np.array(output_layer, dtype=np.float32)
            .reshape(NUM_CLASSES + 4, -1)
            .transpose()
        )
        scores = np.max(data[:, 4:], axis=1)
        mask = scores > self.conf_thres
        data_f = data[mask]
        scores_f = scores[mask]

        if scores_f.size == 0:
            return [], [], []

        class_ids = np.argmax(data_f[:, 4:], axis=1)
        boxes = data_f[:, 0:4].copy()  # [cx, cy, w, h]
        # Convert to top-left [x, y, w, h]
        boxes[:, 0] -= 0.5 * boxes[:, 2]
        boxes[:, 1] -= 0.5 * boxes[:, 3]

        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(), scores_f.tolist(), self.conf_thres, self.iou_thres
        )

        final_boxes, final_scores, final_ids = [], [], []
        if len(indices) > 0:
            for i in indices.flatten():
                final_boxes.append(boxes[i])  # [x, y, w, h] in INPUT space
                final_scores.append(float(scores_f[i]))
                final_ids.append(int(class_ids[i]))

        return final_boxes, final_scores, final_ids

    # ------------------- Depth util -------------------
    @staticmethod
    def median_depth_mm(depth_frame_u16, u, v, roi=10):
        """Calcule la mediane de profondeur (mm) dans une ROI autour de (u, v)."""
        h, w = depth_frame_u16.shape
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))

        x1 = max(0, u - roi)
        y1 = max(0, v - roi)
        x2 = min(w, u + roi)
        y2 = min(h, v + roi)

        roi_arr = depth_frame_u16[y1:y2, x1:x2]
        valid = roi_arr[roi_arr > 0]  # 0 = invalide
        if valid.size == 0:
            return 0
        return int(np.median(valid))

    # ------------------- Main loop -------------------
    def _run(self):
        pipeline = dai.Pipeline()

        # --- RGB ---
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self.input_size, self.input_size)
        cam_rgb.setInterleaved(False)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(30)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # --- Mono + StereoDepth ---
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # align depth to RGB
        stereo.setOutputSize(self.depth_out_w, self.depth_out_h)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # --- NN ---
        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(str(self.blob_path))
        cam_rgb.preview.link(nn.input)

        # --- Outputs ---
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        nn.out.link(xout_nn.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Device
        with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
            # Intrinsics (fx,fy,cx,cy) depuis la calibration device
            # IMPORTANT: On utilise la resolution depth (pas input_size) car:
            # - Le preview 640x640 est CROPPE par DepthAI pour respecter l'aspect ratio carre
            # - La depth 640x360 a une geometrie differente
            # - On calcule X/Y avec les coordonnees depth (du,dv), donc K doit correspondre
            calib = device.readCalibration()
            K = np.array(
                calib.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_A, self.depth_out_w, self.depth_out_h
                ),
                dtype=np.float32,
            )
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            frame = None
            depth_frame = None

            last_pub = 0.0

            while not self._stop_evt.is_set():
                in_rgb = q_rgb.tryGet()
                in_nn = q_nn.tryGet()
                in_depth = q_depth.tryGet()

                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()

                if in_depth is not None:
                    depth_frame = in_depth.getFrame()  # U16 en mm

                if in_nn is not None and frame is not None:
                    layer_names = in_nn.getAllLayerNames()
                    raw = in_nn.getLayerFp16(layer_names[0])

                    boxes, scores, _ = self.decode_yolo_v11(raw)

                    # On ne publie que la meilleure box (plus stable pour le supervisor)
                    best = None
                    if boxes:
                        best_i = int(np.argmax(scores))
                        best = (boxes[best_i], scores[best_i])

                    if best is not None and depth_frame is not None:
                        box, score = best
                        x, y, w, h = box  # INPUT space (640x640)
                        u = x + 0.5 * w
                        v = y + 0.5 * h

                        # Map INPUT (square) -> depth (depth_out_w x depth_out_h)
                        du = int(u * (depth_frame.shape[1] / self.input_size))
                        dv = int(v * (depth_frame.shape[0] / self.input_size))

                        d_mm = self.median_depth_mm(depth_frame, du, dv, roi=10)
                        z = d_mm / 1000.0

                        # Filtre distance
                        if z < self.min_z or z > self.max_z:
                            z = 0.0

                        if z > 0.0:
                            # Calcul 3D avec coordonnees depth (du,dv) et intrinsics depth
                            # Cela corrige le biais du au crop du preview carre vs depth 16:9
                            X = (du - cx) * z / fx
                            Y = (dv - cy) * z / fy
                            Z = z

                            msg = PoseStamped()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = self.camera_frame
                            msg.pose.position.x = float(X)
                            msg.pose.position.y = float(Y)
                            msg.pose.position.z = float(Z)
                            msg.pose.orientation.w = 1.0

                            # Publish (evite de spammer a 200Hz si boucles rapides)
                            now = time.time()
                            if now - last_pub > 0.03:  # ~30 Hz max
                                self.pub_target.publish(msg)
                                last_pub = now

                            if self.debug_view:
                                # Affichage debug
                                H, W = frame.shape[:2]
                                sx, sy = W / self.input_size, H / self.input_size
                                x1 = int(x * sx)
                                y1 = int(y * sy)
                                x2 = int((x + w) * sx)
                                y2 = int((y + h) * sy)
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.putText(
                                    frame,
                                    f"{self.class_name} {score:.2f} [{Z:.2f}m]",
                                    (x1, max(0, y1 - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (0, 255, 0),
                                    2,
                                )

                    if self.debug_view:
                        cv2.imshow("OAK-D YOLO + Depth (ROS2)", frame)
                        if cv2.waitKey(1) == ord("q"):
                            self._stop_evt.set()

                time.sleep(0.001)

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def destroy_node(self):
        self._stop_evt.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakYoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
