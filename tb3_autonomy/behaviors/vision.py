import rclpy
import py_trees
import math
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs


class ObjectRecorder(py_trees.behaviour.Behaviour):
    def __init__(self, name="Recorder", topic_name="/target_object_pose"):
        super(ObjectRecorder, self).__init__(name)
        self.topic_name = topic_name
        self.min_distance = 0.30  # 30 cm d'écart minimum

        # Blackboard: On y stocke une LISTE d'objets
        self.blackboard = py_trees.blackboard.Client(name="Vision")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.WRITE)

        # Initialisation de la liste vide si elle n'existe pas
        self.blackboard.known_objects = []

        self.node = None
        self.tf_buffer = None
        self.sub = None
        self.latest_msg = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.sub = self.node.create_subscription(PoseStamped, self.topic_name, self._cb, 10)

    def _cb(self, msg):
        self.latest_msg = msg

    def update(self):
        # Ce behavior renvoie toujours FAILURE pour ne pas interrompre l'exploration
        # Il travaille en "tâche de fond" (Passive monitoring)

        if self.latest_msg is None:
            return py_trees.common.Status.FAILURE

        try:
            # 1. Transformation dans la MAP
            if not self.tf_buffer.can_transform('map', self.latest_msg.header.frame_id, rclpy.time.Time()):
                return py_trees.common.Status.FAILURE

            pose_map = self.tf_buffer.transform(self.latest_msg, 'map')
            x_new = pose_map.pose.position.x
            y_new = pose_map.pose.position.y

            # 2. Vérification: Est-ce un nouvel objet ?
            is_new = True
            current_list = self.blackboard.known_objects

            for obj in current_list:
                # Calcul distance Euclidienne 2D
                dx = obj['pose'].pose.position.x - x_new
                dy = obj['pose'].pose.position.y - y_new
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < self.min_distance:
                    is_new = False
                    break  # Trop proche d'un objet existant

            # 3. Enregistrement
            if is_new:
                obj_id = len(current_list) + 1
                new_entry = {
                    'id': obj_id,
                    'pose': pose_map,  # On garde le PoseStamped complet
                    'x': x_new,
                    'y': y_new
                }
                current_list.append(new_entry)
                # On force la mise à jour du blackboard
                self.blackboard.known_objects = current_list

                self.node.get_logger().info(f"✅ OBJET #{obj_id} ENREGISTRÉ (X={x_new:.2f}, Y={y_new:.2f})")

            self.latest_msg = None  # Reset

        except Exception as e:
            pass

        return py_trees.common.Status.RUNNING