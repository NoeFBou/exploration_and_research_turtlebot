import rclpy
import py_trees
import math
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
import traceback

class ObjectRecorder(py_trees.behaviour.Behaviour):
    def __init__(self, name="Recorder", topic_name="/target_object_pose"):
        super(ObjectRecorder, self).__init__(name)
        self.topic_name = topic_name
        self.min_distance = 0.7#70cm

        self.blackboard = py_trees.blackboard.Client(name="Vision")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.WRITE)
        self.blackboard.known_objects = []

        self.node = None
        self.tf_buffer = None
        self.sub = None
        self.marker_pub = None
        self.latest_msg = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.sub = self.node.create_subscription(PoseStamped, self.topic_name, self._cb, 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, '/supervisor/known_objects', 10)

    def _cb(self, msg):
        self.latest_msg = msg

    def update(self):
        # Sécurité : Si pas de message, on continue
        if self.latest_msg is None:
            return py_trees.common.Status.RUNNING

        try:
            # 1. Protection TF (Timeout généreux)
            timeout = Duration(seconds=0.1)

            # On vérifie d'abord si la TF est dispo
            if not self.tf_buffer.can_transform('map', self.latest_msg.header.frame_id, rclpy.time.Time(), timeout):
                # Si TF indisponible, on ignore cette frame (c'est mieux que de crasher)
                return py_trees.common.Status.RUNNING

            # 2. Transformation
            self.latest_msg.header.stamp = rclpy.time.Time(seconds=0).to_msg()

            pose_map = self.tf_buffer.transform(self.latest_msg, 'map', timeout=timeout)
            x_new = pose_map.pose.position.x
            y_new = pose_map.pose.position.y

            # 3. Clustering (Logique existante)
            current_list = self.blackboard.known_objects
            match_index = -1

            for i, obj in enumerate(current_list):
                dx = obj['x'] - x_new
                dy = obj['y'] - y_new
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < self.min_distance:
                    match_index = i
                    break

            if match_index != -1:
                # Mise à jour moyenne
                old = current_list[match_index]
                n = old['count']
                current_list[match_index]['x'] = (old['x'] * n + x_new) / (n + 1)
                current_list[match_index]['y'] = (old['y'] * n + y_new) / (n + 1)
                current_list[match_index]['count'] += 1
                current_list[match_index]['pose'] = pose_map
            else:
                # Nouvel objet
                obj_id = len(current_list) + 1
                new_entry = {'id': obj_id, 'pose': pose_map, 'x': x_new, 'y': y_new, 'count': 1}
                current_list.append(new_entry)
                self.node.get_logger().info(f"✅ OBJET #{obj_id} (X={x_new:.2f}, Y={y_new:.2f})")
                self.publish_markers()

            self.blackboard.known_objects = current_list
            self.latest_msg = None

        except Exception as e:
            # C'est ICI que le crash était évité.
            # On affiche l'erreur complète pour comprendre, mais on renvoie RUNNING
            self.node.get_logger().error(f"ERREUR VISION (Ignorée) : {e}")
            traceback.print_exc() # Décommentez pour voir la ligne exacte en console
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def publish_markers(self):
        # Cette fonction est maintenant protégée par le try/except du dessus
        marker_array = MarkerArray()
        objects = self.blackboard.known_objects

        for i, obj in enumerate(objects):
            # SPHERE
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "known_objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = obj['pose'].pose
            marker.pose.position.x = float(obj['x']) # Force float
            marker.pose.position.y = float(obj['y'])

            marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
            marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
            marker_array.markers.append(marker)

            # TEXTE
            text = Marker()
            text.header = marker.header
            text.ns = "ids"
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = marker.pose
            text.pose.position.z += 0.4
            text.scale.z = 0.2
            text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0
            text.text = f"ID {obj['id']}"
            marker_array.markers.append(text)

        if self.marker_pub:
            self.marker_pub.publish(marker_array)