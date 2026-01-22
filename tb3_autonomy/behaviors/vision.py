#!/usr/bin/env python3
import math
import traceback
from typing import Optional, List, Dict, Any

import rclpy
import py_trees
import tf2_ros
# Note: tf2_geometry_msgs is required to register the transform for PoseStamped
import tf2_geometry_msgs

from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class ObjectRecorder(py_trees.behaviour.Behaviour):
    """
    Subscribes to object detections, transforms them into the 'map' frame,
    and performs simple clustering (data association) to maintain a list of unique objects.

    It also publishes visualization markers for RViz.
    """

    def __init__(self, name: str = "Recorder", topic_name: str = "/target_object_pose"):
        """
        Args:
            name (str): The name of the behavior.
            topic_name (str): The ROS topic to subscribe to for object poses.
        """
        super(ObjectRecorder, self).__init__(name)
        self.topic_name = topic_name
        self.min_distance = 0.7  # Distance threshold (meters) for merging objects

        # Blackboard setup
        self.blackboard = py_trees.blackboard.Client(name="Vision")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.WRITE)
        self.blackboard.known_objects = []

        # Runtime variables
        self.node: Optional[Node] = None
        self.tf_buffer = None
        self.tf_listener = None
        self.sub = None
        self.marker_pub = None
        self.latest_msg: Optional[PoseStamped] = None

    def setup(self, **kwargs):
        """
        Sets up the ROS node interfaces (Subscribers, Publishers, TF Buffer).
        """
        self.node = kwargs.get('node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.sub = self.node.create_subscription(
            PoseStamped,
            self.topic_name,
            self._cb,
            10
        )
        self.marker_pub = self.node.create_publisher(
            MarkerArray,
            '/supervisor/known_objects',
            10
        )

    def _cb(self, msg: PoseStamped):
        """
        Callback for new object detections.
        """
        self.latest_msg = msg

    def update(self):
        """
        Main execution loop:
        1. Checks for new data.
        2. Transforms pose to map frame.
        3. associates data with existing objects or creates new ones.
        4. Updates blackboard and RViz markers.
        """
        if self.latest_msg is None:
            return py_trees.common.Status.RUNNING

        try:
            # 1. Transform Verification
            # Use a small timeout to allow TF buffer to catch up
            timeout = Duration(seconds=0.1)

            if not self.tf_buffer.can_transform(
                    'map',
                    self.latest_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout
            ):
                return py_trees.common.Status.RUNNING

            # 2. Perform Transformation
            # Force time to 0 to get the latest available transform
            self.latest_msg.header.stamp = rclpy.time.Time(seconds=0).to_msg()

            pose_map = self.tf_buffer.transform(
                self.latest_msg,
                'map',
                timeout=timeout
            )

            x_new = pose_map.pose.position.x
            y_new = pose_map.pose.position.y

            # 3. Clustering / Data Association
            current_list = self.blackboard.known_objects
            match_index = -1

            for i, obj in enumerate(current_list):
                dx = obj['x'] - x_new
                dy = obj['y'] - y_new
                # hypot is cleaner and faster than sqrt(dx*dx + dy*dy)
                dist = math.hypot(dx, dy)

                if dist < self.min_distance:
                    match_index = i
                    break

            if match_index != -1:
                # Update existing object (Running Average)
                old_obj = current_list[match_index]
                n = old_obj['count']

                current_list[match_index]['x'] = (old_obj['x'] * n + x_new) / (n + 1)
                current_list[match_index]['y'] = (old_obj['y'] * n + y_new) / (n + 1)
                current_list[match_index]['count'] += 1
                current_list[match_index]['pose'] = pose_map
            else:
                # Create new object
                obj_id = len(current_list) + 1
                new_entry = {
                    'id': obj_id,
                    'pose': pose_map,
                    'x': x_new,
                    'y': y_new,
                    'count': 1
                }
                current_list.append(new_entry)
                self.node.get_logger().info(f"[Vision] New Object #{obj_id} detected at (X={x_new:.2f}, Y={y_new:.2f})")
                self.publish_markers()

            # Update Blackboard
            self.blackboard.known_objects = current_list
            self.latest_msg = None

        except Exception as e:
            self.node.get_logger().error(f"[Vision] Transformation/Logic Error: {e}")
            # Optional: Print traceback for deep debugging if needed
            # traceback.print_exc()
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def publish_markers(self):
        """
        Publishes MarkerArray to RViz for visualization (Spheres + IDs).
        """
        if not self.marker_pub:
            return

        marker_array = MarkerArray()
        objects = self.blackboard.known_objects

        for i, obj in enumerate(objects):
            # 1. Sphere Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "known_objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = obj['pose'].pose

            # Ensure strictly float types for serialization
            marker.pose.position.x = float(obj['x'])
            marker.pose.position.y = float(obj['y'])

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

            # 2. Text ID Marker
            text = Marker()
            text.header = marker.header
            text.ns = "ids"
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = marker.pose

            # Offset text slightly above the sphere
            text.pose.position.z += 0.4
            text.scale.z = 0.2

            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0

            text.text = f"ID {obj['id']}"
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)