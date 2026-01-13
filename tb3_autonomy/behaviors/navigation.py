#!/usr/bin/env python3
import rclpy
import py_trees
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class GoToDetectedTarget(py_trees.behaviour.Behaviour):
    """
    Envoie le robot vers la cible mémorisée dans le Blackboard (target_pose_map).
    Gère proprement l'annulation (SKIP).
    """
    def __init__(self, name="GoTo Target"):
        super(GoToDetectedTarget, self).__init__(name)
        self.node = None
        self.action_client = None
        self.goal_handle = None

        # Blackboard
        self.blackboard = py_trees.blackboard.Client(name="Nav")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.node.get_logger().info(f"[Nav] Prépare la navigation...")

        # 1. Vérifier la cible
        target_pose = self.blackboard.target_pose_map
        if target_pose is None:
            self.node.get_logger().error("[Nav] Pas de cible dans le blackboard !")
            return

        # 2. Vérifier le serveur Nav2
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("[Nav] Serveur Nav2 indisponible !")
            return

        # 3. Envoyer l'objectif
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.node.get_logger().info(f"[Nav] Envoi Nav2 vers X={target_pose.pose.position.x:.2f}")

        # Envoi asynchrone
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_handle = None # Reset du handle

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("[Nav] Objectif rejeté par Nav2.")
            return

        #self.node.get_logger().info("[Nav] Objectif accepté, en route.")
        self.goal_handle = goal_handle

        # On demande le résultat final
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Cette fonction est appelée quand Nav2 a fini (Succès, Echec ou Annulé)
        # On ne fait rien de spécial ici, c'est update() qui gère le statut
        pass

    def update(self):
        # Si on n'a pas encore envoyé ou reçu l'acceptation
        if self.goal_handle is None:
            # On pourrait vérifier si send_goal_future a échoué
            return py_trees.common.Status.RUNNING

        # Vérifier si l'action est terminée
        if self.get_result_future.done():
            status = self.get_result_future.result().status
            # STATUS_SUCCEEDED = 4
            if status == 4:
                self.node.get_logger().info("[Nav] Arrivé à destination (Nav2).")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().warn(f"[Nav] Echec ou Annulation Nav2 (Code {status}).")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        C'est ICI que la magie opère pour le SKIP.
        Si le behavior est interrompu (new_status == INVALID ou FAILURE), on doit annuler Nav2.
        """
        if new_status == py_trees.common.Status.INVALID:
            # Si on a un objectif en cours, on l'annule
            if self.goal_handle is not None and not self.get_result_future.done():
                self.node.get_logger().info("[Nav] Annulation demandée (SKIP)...")
                try:
                    # On annule de manière asynchrone pour ne pas bloquer l'arbre
                    self.goal_handle.cancel_goal_async()
                except Exception as e:
                    self.node.get_logger().warn(f"[Nav] Erreur lors de l'annulation: {e}")

            self.goal_handle = None # Nettoyage pour la prochaine fois


class GoToHome(py_trees.behaviour.Behaviour):
    """
    Version simplifiée pour le retour base (Hardcodé à 0,0).
    """
    def __init__(self, name="Retour Base", node=None):
        super(GoToHome, self).__init__(name)
        self.node = node
        self.action_client = None
        self.goal_handle = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.node.get_logger().info("[Nav] Retour à la base (0,0)...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_cb)
        self.goal_handle = None

    def goal_cb(self, future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.result_future = self.goal_handle.get_result_async()

    def update(self):
        if self.goal_handle and self.result_future.done():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass # Pas besoin d'annulation complexe pour le retour home (fin de mission)

class SpinAction(py_trees.behaviour.Behaviour):
    """Réalise le tour sur soi-même pour scanner"""

    def __init__(self, name, target_yaw=6.28):
        super(SpinAction, self).__init__(name)
        self.target_yaw = target_yaw
        self.node = None
        self.client = None
        self.goal_handle = None
        self.sent = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.client = ActionClient(self.node, Spin, 'spin')
        self.client.wait_for_server()

    def initialise(self):
        self.sent = False
        self.goal_handle = None
        goal = Spin.Goal()
        goal.target_yaw = self.target_yaw
        self.future = self.client.send_goal_async(goal)
        self.sent = True
        self.node.get_logger().info("[Nav] Début du Spin...")

    def update(self):
        if not self.sent: return py_trees.common.Status.FAILURE

        if self.goal_handle is None:
            if self.future.done():
                self.goal_handle = self.future.result()
                if not self.goal_handle.accepted:
                    return py_trees.common.Status.FAILURE
                self.res_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.res_future.done():
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.goal_handle:
            self.goal_handle.cancel_goal_async()