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

        self.blackboard = py_trees.blackboard.Client(name="Nav")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.goal_handle = None
        self.get_result_future = None

        target_pose = self.blackboard.target_pose_map
        if target_pose is None:
            self.node.get_logger().error("[Nav] Pas de cible dans le blackboard !")
            return

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("[Nav] Serveur Nav2 indisponible !")
            return

        goal_msg = NavigateToPose.Goal()
        # Ici on assigne tout l'objet PoseStamped d'un coup, donc pas de souci de .pose.pose
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
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING

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
            if self.goal_handle is not None and self.get_result_future is not None and not self.get_result_future.done():
                self.node.get_logger().info("[Nav] Annulation demandée (SKIP)...")
                try:
                    self.goal_handle.cancel_goal_async()
                except: pass
            self.goal_handle = None


class GoToHome(py_trees.behaviour.Behaviour):
    """
    Version BLINDÉE pour le retour base (0,0).
    Corrige l'erreur d'attribut PoseStamped.
    """
    def __init__(self, name="Retour Base", node=None):
        super(GoToHome, self).__init__(name)
        self.node = node
        self.action_client = None
        self.goal_handle = None
        self.get_result_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.node.get_logger().info("[Nav] Retour à la base (0,0)...")
        self.goal_handle = None
        self.get_result_future = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"


        goal_msg.pose.pose.position.x = -2.05
        goal_msg.pose.pose.position.y = -0.64
        goal_msg.pose.pose.orientation.w = 0.00

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("[Nav] Serveur Nav2 introuvable pour le retour base !")
            return

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("[Nav] Retour Base rejeté.")
            return

        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()

    def update(self):
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future.done():
            self.node.get_logger().info("[Nav] Arrivé à la base.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None