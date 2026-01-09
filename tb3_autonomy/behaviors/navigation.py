import rclpy
from rclpy.action import ActionClient
import py_trees
from nav2_msgs.action import NavigateToPose, Spin
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped


class GoToHome(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(GoToHome, self).__init__(name)
        self.node = node
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def setup(self, **kwargs):
        pass  # Déjà fait dans init ou via l'arbre

    def initialise(self):
        self.node.get_logger().info("[Nav] RETOUR BASE (0,0) !")
        goal = NavigateToPose.Goal()

        # Position (0,0) dans la map
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.future = self.client.send_goal_async(goal)
        self.sent = True
        self.goal_handle = None

    def update(self):
        # ... (Exactement la même logique que GoToDetectedTarget pour gérer le statut)
        # Copiez la logique de vérification (RUNNING/SUCCESS) ici
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

class GoToDetectedTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GoToDetectedTarget, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Nav")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.sent_goal = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.node.get_logger().info("[Nav] En attente du serveur Nav2...")
        self.action_client.wait_for_server()

    def initialise(self):
        """Appelé quand ce behavior devient actif (première fois)"""
        self.sent_goal = False
        self.goal_handle = None

        target = self.blackboard.target_pose_map
        if target is None:
            self.node.get_logger().warn("[Nav] Pas de cible dans le blackboard !")
            return

        # Création du Goal (Approche à 60cm comme dans votre code)
        # Note : Idéalement on calcule le point d'approche ici,
        # pour l'instant on envoie la pose brute ou modifiée
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target

        self.node.get_logger().info(f"[Nav] Envoi Nav2 vers X={target.pose.position.x:.2f}")
        self.future = self.action_client.send_goal_async(goal_msg)
        self.sent_goal = True

    def update(self):
        """Surveille l'avancement"""
        if not self.sent_goal:
            return py_trees.common.Status.FAILURE

        # Vérifier si le goal a été accepté
        if self.goal_handle is None:
            if self.future.done():
                self.goal_handle = self.future.result()
                if not self.goal_handle.accepted:
                    self.node.get_logger().error("[Nav] Goal rejeté")
                    return py_trees.common.Status.FAILURE
                self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Vérifier si le robot est arrivé
        if self.result_future.done():
            status = self.result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Annuler si l'arbre décide de faire autre chose (ex: Urgence)"""
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.node.get_logger().info("[Nav] Annulation Nav2")


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