#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import Spin, NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time
import math

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        self.declare_parameter('scan_interval', 40.0)
        self.interval = self.get_parameter('scan_interval').value

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Clients Nav2
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers / Subscribers
        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        # NOUVEAU : On a besoin de piloter les roues directement pour la fin
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(PoseStamped, '/target_object_pose', self.object_detected_callback, 10)

        # Timer Principal
        self.timer = self.create_timer(self.interval, self.timer_callback)
        # Timer de Controle (Pour l'approche finale fluide)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # États
        self.object_found = False
        self.is_scanning = False
        self.nav_approach_done = False # Est-ce qu'on a fini l'approche Nav2 ?
        self.final_approach_active = False # Est-ce qu'on est en pilotage manuel ?
        
        self._spin_goal_handle = None
        self.last_object_msg = None # Pour stocker la dernière position vue

        self.get_logger().info("Superviseur Hybride (Nav2 + Visual Servoing) Prêt.")

    def object_detected_callback(self, msg):
        # On met à jour la dernière position connue en permanence
        self.last_object_msg = msg
        distance_objet = msg.pose.position.z

        # Si on est déjà en approche finale, on laisse le control_loop gérer
        if self.final_approach_active:
            return

        # Si on a déjà fini Nav2, on ne relance pas Nav2
        if self.nav_approach_done:
            # On active juste le pilotage fin si ce n'est pas fait
            if not self.final_approach_active:
                self.get_logger().info("Nav2 fini. Passage en Pilotage Visuel (Approche Fine).")
                self.final_approach_active = True
            return

        # Si on n'a pas encore trouvé l'objet officiellement
        if not self.object_found:
            # Filtre : Entre 2.5m et 0.6m (On laisse l'approche fine gérer en dessous de 0.6)
            if distance_objet < 2.5 and distance_objet > 0.6:
                self.get_logger().warning(f"CIBLE DETECTEE À {distance_objet:.2f}m -> Lancement Nav2")
                self.start_nav2_sequence(msg, distance_objet)

    def start_nav2_sequence(self, msg, distance):
        self.object_found = True
        self.get_logger().warning("ARRET DE L'EXPLORATION")

        if self.timer: self.timer.cancel()
        self.toggle_exploration(False)
        
        if self.is_scanning and self._spin_goal_handle:
            self._spin_goal_handle.cancel_goal_async()
            time.sleep(1.0) # Petite pause pour stabiliser

        # --- CALCUL POINT D'APPROCHE NAV2 (LOIN) ---
        # On demande à Nav2 d'aller à 60cm de l'objet (Zone Sûre)
        stop_distance = 0.60 
        target_z = distance - stop_distance

        approach_pose = PoseStamped()
        approach_pose.header = msg.header
        approach_pose.header.stamp = self.get_clock().now().to_msg()
        approach_pose.pose.position.x = msg.pose.position.x
        approach_pose.pose.position.y = 0.0
        approach_pose.pose.position.z = target_z
        approach_pose.pose.orientation.w = 1.0

        try:
            pose_map = self.tf_buffer.transform(approach_pose, 'map', timeout=Duration(seconds=1.0))
            self.get_logger().info(f"Nav2 va à 60cm de la cible.")
            self.send_nav_goal(pose_map)
        except Exception as e:
            self.get_logger().error(f'Erreur TF: {e}')
            self.object_found = False # Reset si échec calcul

    # --- BOUCLE DE CONTROLE (Approche Finale) ---
    def control_loop(self):
        # Cette boucle tourne 10 fois par seconde
        if not self.final_approach_active:
            return

        if self.last_object_msg is None:
            # Sécurité: si on perd l'objet de vue, on s'arrête
            self.cmd_vel_pub.publish(Twist())
            return

        # Récupération des coordonnées (repère caméra)
        # X = Droite/Gauche, Z = Devant
        x_obj = self.last_object_msg.pose.position.x
        z_obj = self.last_object_msg.pose.position.z

        # --- LOGIQUE D'ASSERVISSEMENT ---
        twist = Twist()

        # 1. Condition d'arrêt (12cm de l'objet)
        if z_obj <= 0.12:
            self.get_logger().info("STOP ! Cible atteinte (12cm). GRIPPER !")
            self.cmd_vel_pub.publish(Twist()) # Arrêt total
            self.final_approach_active = False # Fin de la mission
            # ICI : APPELER LE GRIPPER
            return

        # 2. Avancer (Vitesse proportionnelle à la distance, mais bornée)
        # On ralentit quand on approche
        speed_x = 0.15 # Vitesse constante douce
        if z_obj < 0.3: speed_x = 0.05 # Très lent à la fin

        twist.linear.x = speed_x

        # 3. Tourner (Correction d'angle)
        # Si x_obj > 0, l'objet est à gauche (dans le repère optique standard -Y) 
        # Attention aux axes: Camera Optical X=Droite. 
        # Pour centrer l'objet (X=0), on tourne.
        # Gain P (Proportionnel) : 1.5
        twist.angular.z = -1.5 * x_obj 

        self.cmd_vel_pub.publish(twist)
        # Reset du message pour éviter d'utiliser des vieilles données si la caméra lag
        self.last_object_msg = None 

    # --- ACTIONS NAV2 ---
    def send_nav_goal(self, pose):
        if not self._nav_client.wait_for_server(timeout_sec=2.0): return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._send_nav_future = self._nav_client.send_goal_async(goal_msg)
        self._send_nav_future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 refusé -> Passage direct en manuel ?')
            # Si Nav2 refuse, on tente quand même l'approche manuelle si on voit l'objet
            self.nav_approach_done = True 
            return
        
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        # Quand Nav2 a fini (Réussite ou Echec peu importe, on est censé être plus près)
        self.get_logger().info('Fin Nav2. Activation Approche Finale.')
        self.nav_approach_done = True
        # Le control_loop prendra le relais automatiquement

    # --- RESTE DU CODE (Spin, Timer) ---
    def timer_callback(self):
        if self.is_scanning or self.object_found: return
        self.toggle_exploration(False)
        self.send_spin_goal()

    def toggle_exploration(self, state: bool):
        msg = Bool()
        msg.data = state
        self.resume_pub.publish(msg)

    def send_spin_goal(self):
        if not self._spin_client.wait_for_server(timeout_sec=1.0): return
        self.is_scanning = True
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = 6.28
        self._send_goal_future = self._spin_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_scanning = False
            self.toggle_exploration(True)
            return
        self._spin_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.object_found: return
        self.toggle_exploration(True)
        self.is_scanning = False
        self._spin_goal_handle = None

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