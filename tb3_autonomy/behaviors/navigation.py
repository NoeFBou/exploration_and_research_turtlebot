#!/usr/bin/env python3
import rclpy
import py_trees
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class GoToDetectedTarget(py_trees.behaviour.Behaviour):
    """
    Envoie le robot vers la cible mémorisée dans le Blackboard.
    """
    def __init__(self, name="GoTo Target"):
        super(GoToDetectedTarget, self).__init__(name)
        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False

        self.blackboard = py_trees.blackboard.Client(name="Nav")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False

        target_pose = self.blackboard.target_pose_map
        if target_pose is None:
            self.node.get_logger().error("[Nav] Pas de cible dans le blackboard !")
            return

        # On attend le serveur un peu plus longtemps (5s) à cause du lag potentiel
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[Nav] Serveur Nav2 indisponible (Timeout) !")
            return

        self.server_online = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.node.get_logger().info(f"[Nav] Envoi Nav2 vers X={target_pose.pose.position.x:.2f}")

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("[Nav] Objectif rejeté par Nav2.")
            return

        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()

    def update(self):
        # 1. Si le serveur n'était pas là au début -> Echec direct
        if not self.server_online:
            return py_trees.common.Status.FAILURE

        # 2. Si la demande de but a échoué (rejet ou timeout envoi)
        if self.goal_handle is None:
            # Si la future est finie mais pas de handle => Rejeté
            if hasattr(self, 'send_goal_future') and self.send_goal_future.done():
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # 3. Attente du résultat final
        if self.get_result_future is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future.done():
            status = self.get_result_future.result().status
            if status == 4: # SUCCEEDED
                self.node.get_logger().info("[Nav] Arrivé à destination.")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().warn(f"[Nav] Echec/Annulation (Code {status}).")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self._cancel_goal()

    def _cancel_goal(self):
        if self.goal_handle is not None and self.get_result_future is not None and not self.get_result_future.done():
            try:
                self.goal_handle.cancel_goal_async()
            except: pass
        self.goal_handle = None


class GoToHome(py_trees.behaviour.Behaviour):
    """
    Retour Base avec coordonnées Z=0.0 explicites et tolérance aux pannes.
    """
    def __init__(self, name="Retour Base", node=None):
        super(GoToHome, self).__init__(name)
        self.node = node
        self.action_client = None
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.node.get_logger().info("[Nav] Retour à la base (-2.0, -0.5)...")
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"

        # Position de départ (Spawn)
        goal_msg.pose.pose.position.x = -2.0
        goal_msg.pose.pose.position.y = -0.5
        goal_msg.pose.pose.position.z = 0.0  # <--- On force Z à 0.0 comme demandé

        # Orientation Neutre (Regarde vers l'Est / X+)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0 # <--- INDISPENSABLE

        # Timeout augmenté à 5s pour le lag
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[Nav] Serveur Nav2 introuvable (Timeout) !")
            return

        self.server_online = True

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("[Nav] Retour Base rejeté par Nav2 (Obstacle ?).")
            return

        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()

    def update(self):
        # 1. Si serveur HS, on force le SUCCESS pour finir la mission et rendre le menu
        if not self.server_online:
            self.node.get_logger().warn("[Nav] Serveur HS -> On force la fin de mission.")
            return py_trees.common.Status.SUCCESS

        # 2. Si rejeté (Goal handle None après retour future)
        if self.goal_handle is None:
            if hasattr(self, 'send_goal_future') and self.send_goal_future.done():
                self.node.get_logger().warn("[Nav] Impossible d'aller à la base (Rejet). On termine quand même.")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        # 3. Attente résultat
        if self.get_result_future is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future.done():
            self.node.get_logger().info("[Nav] Fin séquence retour base.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None