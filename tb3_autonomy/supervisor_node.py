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

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Clients Nav2
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers / Subscribers
        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.catch_pub = self.create_publisher(Bool, '/catch', 10)
        self.create_subscription(PoseStamped, '/target_object_pose', self.object_detected_callback, 10)

        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # États
        self.object_found = False
        self.is_scanning = False
        self.nav_approach_done = False
        self.final_approach_active = False
        self.mission_complete = False
        
        self._spin_goal_handle = None
        self.last_object_msg = None
        self.last_known_direction = 1
        self.lost_target_time = None
        self.initial_pose = None
        self.start_time = time.time()
        self.get_logger().info("Superviseur Hybride (Nav2 + Visual Servoing) Prêt.")

    def save_initial_pose(self):
        if self.initial_pose is not None:
            return

        try:
            if time.time() - self.start_time < 5.0:
                return
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.initial_pose = PoseStamped()
            self.initial_pose.header.frame_id = 'map'
            self.initial_pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose.pose.position.x = trans.transform.translation.x
            self.initial_pose.pose.position.y = trans.transform.translation.y
            self.initial_pose.pose.orientation = trans.transform.rotation

            self.get_logger().info(
                f"HOME Position Sauvegardée : X={self.initial_pose.pose.position.x:.2f}, Y={self.initial_pose.pose.position.y:.2f}")
        except Exception:
            pass


    def object_detected_callback(self, msg):
        self.save_initial_pose()
        self.last_object_msg = msg
        if msg.pose.position.x > 0:
            self.last_known_direction = -1
        else:
            self.last_known_direction = 1

        distance_objet = msg.pose.position.z
        if self.mission_complete: return
        if self.final_approach_active: return

        if self.nav_approach_done:
            if not self.final_approach_active:
                self.get_logger().info("Nav2 fini. Passage en Pilotage Visuel (Approche Fine).")
                self.final_approach_active = True
            return

        if not self.object_found:
            if distance_objet < 2.5 and distance_objet > 0.5:
                self.get_logger().warning(f"CIBLE DETECTEE À {distance_objet:.2f}m -> Nav2")
                self.start_nav2_sequence(msg, distance_objet)
            elif distance_objet <= 0.5 and distance_objet > 0.1:
                self.get_logger().warning(f"Cible très proche ({distance_objet:.2f}m) -> Manuel Direct")
                self.object_found = True
                self.stop_exploration_behaviors()
                self.nav_approach_done = True
                self.final_approach_active = True

    def start_nav2_sequence(self, msg, distance):
        self.object_found = True
        self.stop_exploration_behaviors()

        stop_distance = 0.35
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
            self.object_found = False

    def stop_exploration_behaviors(self):
        self.get_logger().warning("ARRET EXPLORATION")
        if self.timer:
            self.timer.cancel()
        self.toggle_exploration(False)
        if self.is_scanning and self._spin_goal_handle:
            self._spin_goal_handle.cancel_goal_async()
            time.sleep(1.0)


    def control_loop(self):
        self.save_initial_pose()
        if not self.final_approach_active:
            return

        twist = Twist()

        if self.last_object_msg is None:
            if self.lost_target_time is None:
                self.lost_target_time = time.time()
                self.get_logger().warning("Cible perdue ! Recherche en cours...")

            elapsed = time.time() - self.lost_target_time

            if elapsed > 6.0:
                self.get_logger().error("ECHEC : Cible introuvable après rotation.")
                self.get_logger().info("Reprise de l'exploration automatique.")
                self.reset_to_exploration()
                return
            else:
                twist.angular.z = 0.6 * self.last_known_direction
                self.cmd_vel_pub.publish(twist)
                return

        self.lost_target_time = None
        x_obj = self.last_object_msg.pose.position.x
        z_obj = self.last_object_msg.pose.position.z

        if z_obj <= 0.10:
            self.get_logger().info("STOP ! Cible atteinte (10cm). GRIPPER !")
            self.cmd_vel_pub.publish(Twist())
            self.final_approach_active = False
            self.mission_complete = True

            msg = Bool()
            msg.data = True
            self.catch_pub.publish(msg)
            self.get_logger().info("Fermeture pince en cours...")
            time.sleep(4.0)

            self.return_home()
            return

        twist.linear.x = 0.15 if z_obj > 0.3 else 0.05

        twist.angular.z = -2.0 * x_obj

        self.cmd_vel_pub.publish(twist)

        self.last_object_msg = None

    def return_home(self):
        if self.initial_pose is None:
            self.get_logger().error("Pas de position Home enregistrée ! Je reste ici.")
            return

        self.get_logger().info("OBJET ATTRAPÉ -> RETOUR À LA MAISON !")
        self.send_nav_goal(self.initial_pose, is_homing=True)
    def reset_to_exploration(self):
        self.object_found = False
        self.nav_approach_done = False
        self.final_approach_active = False
        self.cmd_vel_pub.publish(Twist())  # Stop
        self.toggle_exploration(True)
        self.timer.reset()

    def send_nav_goal(self, pose, is_homing=False):
        if not self._nav_client.wait_for_server(timeout_sec=2.0): return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Envoi Nav2 ({'HOME' if is_homing else 'CIBLE'})...")
        self._send_nav_future = self._nav_client.send_goal_async(goal_msg)

        if is_homing:
            self._send_nav_future.add_done_callback(self.homing_response_callback)
        else:
            self._send_nav_future.add_done_callback(self.nav_response_callback)
    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 refusé -> Passage direct en manuel ?')
            self.nav_approach_done = True
            return
        
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Nav2 fini (Status {status}). Passage en Manuel.')
        self.nav_approach_done = True

    def homing_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Impossible de rentrer à la maison (Chemin bloqué ?).")
            return
        self.get_logger().info("Retour maison en cours...")
        goal_handle.get_result_async().add_done_callback(self.homing_result_callback)

    def homing_result_callback(self, future):
        self.get_logger().info("MISSION ACCOMPLIE : Je suis rentré avec l'objet !")

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