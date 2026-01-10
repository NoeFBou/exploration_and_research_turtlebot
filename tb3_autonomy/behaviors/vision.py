import rclpy
import py_trees
import math
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

class ObjectRecorder(py_trees.behaviour.Behaviour):
    def __init__(self, name="Recorder", topic_name="/target_object_pose"):
        super(ObjectRecorder, self).__init__(name)
        self.topic_name = topic_name

        # --- REGLAGE DU SEUIL ---
        # 30cm est trop strict quand le robot bouge.
        # On passe à 70cm (0.7) pour éviter les doublons dus au drift du SLAM.
        self.min_distance = 0.7

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
        # Toujours RUNNING pour ne jamais faire échouer la séquence
        if self.latest_msg is None:
            return py_trees.common.Status.RUNNING

        try:
            # t_robot = self.tf_buffer.lookup_transform(
            #     'map', 'base_link', rclpy.time.Time())
            # rob_x = t_robot.transform.translation.x
            # rob_y = t_robot.transform.translation.y
            # rob_yaw = math.atan2(
            #     2.0 * (t_robot.transform.rotation.w * t_robot.transform.rotation.z),
            #     1.0 - 2.0 * (t_robot.transform.rotation.z * t_robot.transform.rotation.z)
            # )
            #
            # # --- DEBUG 2 : Données Brutes Caméra ---
            # # X_cam = Latéral (Droite/Gauche), Z_cam = Profondeur (Devant)
            # cam_x = self.latest_msg.pose.position.x
            # cam_z = self.latest_msg.pose.position.z
            # frame_id = self.latest_msg.header.frame_id
            #
            # # --- Transformation ---
            # timeout = Duration(seconds=0.05)
            # # (Votre code de transformation existant ici...)
            # # Si besoin, l'astuce pour l'Extrapolation Error :
            # if not self.tf_buffer.can_transform('map', frame_id, rclpy.time.Time(), timeout):
            #     self.latest_msg.header.stamp = rclpy.time.Time().to_msg()
            #
            # pose_map = self.tf_buffer.transform(self.latest_msg, 'map', timeout=timeout)
            #
            # map_x = pose_map.pose.position.x
            # map_y = pose_map.pose.position.y
            #
            # # --- AFFICHAGE COMPLET DU DIAGNOSTIC ---
            # self.node.get_logger().info(f"\n"
            #                             f"--- DIAGNOSTIC VISION ---\n"
            #                             f"1. ROBOT (Map) : X={rob_x:.2f}, Y={rob_y:.2f}, Angle={math.degrees(rob_yaw):.1f}°\n"
            #                             f"2. CAMÉRA (Brut): X={cam_x:.2f}, Z={cam_z:.2f} (Frame: {frame_id})\n"
            #                             f"3. CALCULÉ (Map): X={map_x:.2f}, Y={map_y:.2f}\n"
            #                             f"-----------------------"
            #                             )
            # 1. Protection Temporelle (Extrapolation Error)
            # On vérifie si la TF est disponible pour le temps demandé
            # Si non, on utilise le temps '0' (la dernière transformation connue), c'est moins précis mais ça ne plante pas.
            timeout = Duration(seconds=0.05)
            can_transform = self.tf_buffer.can_transform(
                'map',
                self.latest_msg.header.frame_id,
                rclpy.time.Time(), # Temps 'maintenant'
                timeout
            )

            if not can_transform:
                # Si on ne peut pas transformer le message exact, on tente avec le dernier transform dispo
                # C'est une astuce pour éviter l'Extrapolation Error
                target_pose = self.latest_msg
                target_pose.header.stamp = rclpy.time.Time().to_msg() # On force le temps à 0 (latest)

            # 2. Transformation
            pose_map = self.tf_buffer.transform(self.latest_msg, 'map', timeout=timeout)

            x_new = pose_map.pose.position.x
            y_new = pose_map.pose.position.y

            # 3. Logique de Fusion (Clustering)
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
                # CAS 1 : C'est un objet déjà connu -> ON MET A JOUR (Moyenne glissante)
                # Cela permet d'affiner la position au fur et à mesure qu'on le voit
                old_obj = current_list[match_index]
                n = old_obj['count']

                # Nouvelle moyenne pondérée
                new_x = (old_obj['x'] * n + x_new) / (n + 1)
                new_y = (old_obj['y'] * n + y_new) / (n + 1)

                current_list[match_index]['x'] = new_x
                current_list[match_index]['y'] = new_y
                current_list[match_index]['count'] += 1
                current_list[match_index]['pose'] = pose_map # On garde la dernière pose valide pour l'orientation

                # On ne spamme pas les logs pour une mise à jour
                # self.node.get_logger().info(f"Mise à jour Objet #{old_obj['id']} (n={n+1})")

            else:
                # CAS 2 : C'est vraiment un nouvel objet
                obj_id = len(current_list) + 1
                new_entry = {
                    'id': obj_id,
                    'pose': pose_map,
                    'x': x_new,
                    'y': y_new,
                    'count': 1 # Nombre de fois qu'on l'a vu
                }
                current_list.append(new_entry)
                self.node.get_logger().info(f"✅ OBJET #{obj_id} DÉCOUVERT (X={x_new:.2f}, Y={y_new:.2f})")
                self.publish_markers() # On met à jour Rviz

            # Sauvegarde et nettoyage
            self.blackboard.known_objects = current_list
            self.latest_msg = None

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # On attrape l'erreur silencieusement (ou avec un warn léger) pour ne pas crasher
            # self.node.get_logger().warn(f"TF Lag (ignoré): {e}")
            pass
        except Exception as e:
            self.node.get_logger().error(f"Erreur Vision Critique: {e}")

        return py_trees.common.Status.RUNNING

    def publish_markers(self):
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
            # On force la position moyennée
            marker.pose.position.x = obj['x']
            marker.pose.position.y = obj['y']

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
            text.text = f"ID {obj['id']} (n={obj['count']})"
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)