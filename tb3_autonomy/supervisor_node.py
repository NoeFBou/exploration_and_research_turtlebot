#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import Spin, NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs # Important pour que le buffer sache transformer des Poses
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        self.declare_parameter('scan_interval', 40.0)
        self.interval = self.get_parameter('scan_interval').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self._spin_client = ActionClient(self, Spin, 'spin')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.create_subscription(PoseStamped, '/target_object_pose', self.object_detected_callback, 10)

        self.timer = self.create_timer(self.interval, self.timer_callback)

        self.object_found = False
        self.is_scanning = False
        self.approach_in_progress = False
        self._spin_goal_handle = None

    def object_detected_callback(self, msg):
       # self.get_logger().info('Object detected')
        if self.object_found:
            return
        X = msg.pose.position.x
        Z = msg.pose.position.z
        distance_objet = msg.pose.position.z
        #self.get_logger().info('Object detected')
        #self.get_logger().info('X: {}, Z: {}'.format(X, Z))
        #self.get_logger().info('distance_objet: {}'.format(distance_objet))

        if distance_objet < 2.5 and distance_objet > 0.2:
            self.get_logger().warning(f"CIBLE DETECTEE À {Z:.2f}m")
            self.get_logger().warning("ARRET DE L EXPLORATION ")

            self.object_found = True

            if self.timer:
                self.timer.cancel()
            self.toggle_exploration(False)
            if self.is_scanning and self._spin_goal_handle is not None:
                self.get_logger().warning("Annulation du Spin en cours...")
                self._spin_goal_handle.cancel_goal_async()
                time.sleep(1.0)
            self.initiate_approach(msg, distance_objet)

    def initiate_approach(self, object_pose_msg, current_distance):
        self.approach_in_progress = True
        self.get_logger().info("Calcul de la trajectoire d'approche...")

        # Distance d'arrêt avant l'objet (ex: 35 cm)
        stop_distance = 0.15
        target_z = current_distance - stop_distance

        if target_z < 0:
            self.get_logger().warning("Déjà trop proche ! Je m'arrête là.")
            return

        # Création du point cible dans le repère CAMÉRA
        approach_pose_cam = PoseStamped()
        approach_pose_cam.header = object_pose_msg.header  # Important: garde le frame_id (ex: color_optical_frame)
        approach_pose_cam.header.stamp = self.get_clock().now().to_msg()  # Rafraîchir le temps pour TF

        approach_pose_cam.pose.position.x = object_pose_msg.pose.position.x
        approach_pose_cam.pose.position.y = 0.0  # On ignore la hauteur
        approach_pose_cam.pose.position.z = target_z  # On vise devant l'objet
        approach_pose_cam.pose.orientation.w = 1.0


        try:
            pose_map = self.tf_buffer.transform(
                approach_pose_cam,
                'map',
                timeout=Duration(seconds=1.0)
            )

            self.get_logger().info(
                f"Point d'approche Carte : X={pose_map.pose.position.x:.2f}, Y={pose_map.pose.position.y:.2f}")
            current_pose = self.get_current_pose()
            self.get_logger().info(f"--- DEBUG APPROCHE ---")
            self.get_logger().info(f"Robot Actuel : X={current_pose.position.x:.2f}, Y={current_pose.position.y:.2f}")
            self.get_logger().info(f"But Calculé  : X={pose_map.pose.position.x:.2f}, Y={pose_map.pose.position.y:.2f}")
            self.get_logger().info(f"Distance à parcourir : {self.calculate_distance(current_pose, pose_map.pose):.2f}m")

            self.send_nav_goal(pose_map)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Erreur Transformation TF : {e}')
            self.object_found = False  # On réessaiera au prochain messag



    def timer_callback(self):
        if self.is_scanning or self.object_found:
            return

        self.toggle_exploration(False)
        self.send_spin_goal()

    def toggle_exploration(self, state: bool):

        msg = Bool()
        msg.data = state
        self.resume_pub.publish(msg)
        action = "REPRISE" if state else "PAUSE"
        self.get_logger().info(f'Envoi de la commande : {action}')

    def send_nav_goal(self, pose):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 non disponible !")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Envoi de l'ordre d'approche à Nav2...")
        self._send_nav_future = self._nav_client.send_goal_async(goal_msg)
        self._send_nav_future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 a refusé le but (Zone interdite ou trop proche ?)')
            return

        self.get_logger().info('Approche validée ! Le robot se déplace.')
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Le robot est arrivé devant l\'objet ! (LANCEMENT DU GRIPPER)')
            # ICI : Appeler ton service ou ton code pour fermer la pince
        else:
            self.get_logger().warning(f'Echec de l\'approche (Status: {status})')
    def send_spin_goal(self):
        if not self._spin_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Serveur Spin non disponible. Annulation.')
            self.toggle_exploration(True)
            return

        self.is_scanning = True
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = 6.28  # 360

        self._send_goal_future = self._spin_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.is_scanning = False
            self.toggle_exploration(True)
            return

        self.get_logger().info('Rotation acceptée. En cours...')
        self._spin_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if self.object_found:
            self.get_logger().info('Fin du Spin (Interrompu ou fini), mais objet trouvé. Focus sur approche.')
            self.is_scanning = False
            self._spin_goal_handle = None
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Rotation terminée avec succès.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Rotation annulée.')
        else:
            self.get_logger().info(f'Rotation finie (Status: {status}).')

        self.toggle_exploration(True)
        self.is_scanning = False
        self._spin_goal_handle = None

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = tf2_geometry_msgs.Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            return pose
        except:
            return tf2_geometry_msgs.Pose()

    def calculate_distance(self, pose1, pose2):
        import math
        return math.sqrt((pose1.position.x - pose2.position.x) ** 2 + (pose1.position.y - pose2.position.y) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()