import rclpy
import py_trees
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32, String
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# =============================================================================
# CLASSES DE NAVIGATION & MOUVEMENT
# =============================================================================

class RotateToTarget(py_trees.behaviour.Behaviour):
    """
    Tourne sur place pour aligner le robot face à la cible.
    Utilise Time(0) pour éviter les blocages TF.
    """
    def __init__(self, name="Alignement", threshold=0.05):
        super(RotateToTarget, self).__init__(name)
        self.threshold = threshold # ~3 degrés
        self.node = None
        self.cmd_vel_pub = None
        self.tf_buffer = None
        self.tf_listener = None

        self.blackboard = py_trees.blackboard.Client(name="Align")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.node.get_logger().info("[Align] Début de la rotation sur place...")

    def update(self):
        target_map = self.blackboard.target_pose_map
        if target_map is None:
            return py_trees.common.Status.FAILURE

        try:
            # CORRECTION TF : Time(seconds=0)
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time(seconds=0)
            )
            target_local = do_transform_pose(target_map.pose, transform)
        except Exception as e:
            return py_trees.common.Status.RUNNING

        x = target_local.position.x
        y = target_local.position.y
        angle_error = math.atan2(y, x)

        if abs(angle_error) < self.threshold:
            self.node.get_logger().info("[Align] Cible centrée ! Stop.")
            self.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        twist = Twist()
        twist.linear.x = 0.0
        k_p = 1.5
        angular_speed = k_p * angle_error
        max_speed = 0.5
        twist.angular.z = max(min(angular_speed, max_speed), -max_speed)

        if abs(twist.angular.z) < 0.1:
            twist.angular.z = 0.1 if angle_error > 0 else -0.1

        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.cmd_vel_pub.publish(Twist())


class VisualServoingApproach(py_trees.behaviour.Behaviour):
    """
    Approche finale 'manuelle'. Avance tout droit avec correction légère.
    """
    def __init__(self, name="Visual Servoing", min_dist=0.15):
        super(VisualServoingApproach, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Action")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)
        self.node = None
        self.cmd_vel_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.target_min_dist = min_dist
        self.kp_linear = 0.5

    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.node.get_logger().info("[Action] Démarrage approche visuelle...")

    def update(self):
        target_map = self.blackboard.target_pose_map
        if target_map is None:
            return py_trees.common.Status.FAILURE

        try:
            # Ici on utilise Time() normal car on avance, c'est moins critique que la rotation
            # Mais on pourrait aussi mettre Time(0) si ça lag encore.
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time(seconds=0) # Sécurité Time(0) ici aussi
            )
            target_local = do_transform_pose(target_map.pose, transform)
        except Exception:
            return py_trees.common.Status.RUNNING

        x_dist = target_local.position.x
        y_lat  = target_local.position.y

        # Condition de target_min_dist
        if x_dist <= self.target_min_dist:
            self.node.get_logger().info(f"[Visual] Stop final (Dist={x_dist:.2f}m).")
            self.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        # Commande Moteur SIMPLIFIÉE (Car on a déjà fait la rotation avant)
        twist = Twist()
        # On avance
        twist.linear.x = 0.15 if x_dist > 0.4 else 0.05
        # Correction d'angle douce
        twist.angular.z = 0.8 * y_lat

        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

        # --- J'AI SUPPRIMÉ TOUT LE CODE MORT QUI ÉTAIT ICI ---

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.cmd_vel_pub.publish(Twist())


class BackUp(py_trees.behaviour.Behaviour):
    def __init__(self, name="Reculer", duration=2.0, speed=-0.1):
        super(BackUp, self).__init__(name)
        self.duration = duration
        self.speed = speed
        self.start_time = None
        self.node = None
        self.pub = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def initialise(self):
        self.start_time = self.node.get_clock().now()
        self.node.get_logger().info("[Nav] Recul en cours...")

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed > self.duration:
            self.pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        msg = Twist()
        msg.linear.x = self.speed
        self.pub.publish(msg)
        return py_trees.common.Status.RUNNING
# Dans actions.py

class ManualRecovery(py_trees.behaviour.Behaviour):
    """
    Active le mode manuel sur le contrôleur et attend le succès.
    """
    def __init__(self, name="Récupération Manuelle"):
        super(ManualRecovery, self).__init__(name)
        self.node = None
        self.pub_status = None
        self.pub_catch = None
        self.sub_conf = None
        self.confirmation = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)
        # On a besoin de publier sur catch pour l'ouvrir au début
        self.pub_catch = self.node.create_publisher(Bool, '/catch', 10)
        self.sub_conf = self.node.create_subscription(Bool, '/mission/confirmation', self._cb, 10)

    def initialise(self):
        self.confirmation = None

        # 1. On ouvre la pince par sécurité avant de donner la main
        self.node.get_logger().info("[Manual] Ouverture pince...")
        self.pub_catch.publish(Bool(data=False))

        # 2. On signale au controller de passer en mode TELEOP
        msg = String()
        msg.data = "MANUAL_RECOVERY"
        self.pub_status.publish(msg)
        self.node.get_logger().info("[Manual] En attente de l'opérateur humain...")

    def _cb(self, msg):
        # Le controller renverra True si l'utilisateur dit "Succès"
        # Il renverra False si l'utilisateur dit "Abandon"
        self.confirmation = msg.data

    def update(self):
        if self.confirmation is None:
            return py_trees.common.Status.RUNNING

        if self.confirmation:
            self.node.get_logger().info("[Manual] L'opérateur a confirmé la prise !")
            return py_trees.common.Status.SUCCESS  # Succès -> On sort de la boucle et on rentre
        else:
            self.node.get_logger().info("[Manual] L'opérateur a abandonné.")
            return py_trees.common.Status.FAILURE

# =============================================================================
# CLASSES DE LOGIQUE & SIGNAUX
# =============================================================================

class WaitDuration(py_trees.behaviour.Behaviour):
    def __init__(self, name="Timer", duration=90.0):
        super(WaitDuration, self).__init__(name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForUserSelection(py_trees.behaviour.Behaviour):
    def __init__(self, name="Attente Choix"):
        super(WaitForUserSelection, self).__init__(name)
        self.node = None
        self.sub = None
        self.pub_status = None
        self.received_id = -1

        self.blackboard = py_trees.blackboard.Client(name="Interface")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Int32, '/mission/select_target', self._msg_callback, 10)
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)

    def _msg_callback(self, msg):
        self.node.get_logger().info(f"[UI] Commande reçue : ID {msg.data}")
        self.received_id = msg.data

    def initialise(self):
        # CORRECTION : ON NE RESET PAS L'ID ICI (Mémoire tampon)
        self.node.get_logger().info("[UI] En attente d'une sélection...")
        msg = String()
        msg.data = "IDLE"
        self.pub_status.publish(msg)

    def update(self):
        if self.received_id == -1:
            return py_trees.common.Status.RUNNING

        objects = self.blackboard.known_objects
        if not objects:
            return py_trees.common.Status.FAILURE

        target_id = self.received_id
        selected_obj = next((o for o in objects if o['id'] == target_id), None)

        if selected_obj:
            self.node.get_logger().info(f"[UI] Cible #{target_id} validée.")
            self.blackboard.target_pose_map = selected_obj['pose']
            # CORRECTION : ON CONSOMME L'ID MAINTENANT
            self.received_id = -1
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn(f"[UI] ID {target_id} invalide ou inconnu !")
            self.received_id = -1
            self.pub_status.publish(String(data="IDLE"))
            return py_trees.common.Status.RUNNING


class WaitForSkipSignal(py_trees.behaviour.Behaviour):
    def __init__(self, name="Attente Skip"):
        super(WaitForSkipSignal, self).__init__(name)
        self.node = None
        self.sub = None
        self.skip_received = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Bool, '/mission/skip_nav', self._cb, 10)

    def initialise(self):
        self.skip_received = False

    def _cb(self, msg):
        # CORRECTION : On vérifie que le behavior est RUNNING
        if self.status == py_trees.common.Status.RUNNING:
            if msg.data:
                self.node.get_logger().info(f"[{self.name}] SKIP REÇU !")
                self.skip_received = True

    def update(self):
        if self.skip_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForAbortSignal(py_trees.behaviour.Behaviour):
    def __init__(self, name="Attente Abort"):
        super(WaitForAbortSignal, self).__init__(name)
        self.node = None
        self.sub = None
        self.abort_received = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Bool, '/mission/abort', self._cb, 10)

    def initialise(self):
        self.abort_received = False

    def _cb(self, msg):
        # CORRECTION : On vérifie que le behavior est RUNNING
        if self.status == py_trees.common.Status.RUNNING:
            if msg.data:
                self.node.get_logger().info(f"[{self.name}] ABORT REÇU !")
                self.abort_received = True

    def update(self):
        if self.abort_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForStartSignal(py_trees.behaviour.Behaviour):
    def __init__(self, name="Attente Start"):
        super(WaitForStartSignal, self).__init__(name)
        self.node = None
        self.sub = None
        self.start_received = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Bool, '/mission/start', self._cb, 10)

    def initialise(self):
        self.start_received = False
        self.node.get_logger().info("[Superviseur] En attente du signal START...")

    def _cb(self, msg):
        if msg.data:
            self.start_received = True

    def update(self):
        return py_trees.common.Status.SUCCESS if self.start_received else py_trees.common.Status.RUNNING


class WaitForConfirmation(py_trees.behaviour.Behaviour):
    def __init__(self, name="Validation Humaine", status_msg="WAITING_CONFIRMATION"):
        super(WaitForConfirmation, self).__init__(name)
        self.node = None
        self.pub_status = None
        self.sub_conf = None
        self.confirmation = None
        self.status_msg = status_msg # Message personnalisé

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)
        self.sub_conf = self.node.create_subscription(Bool, '/mission/confirmation', self._cb, 10)

    def initialise(self):
        self.confirmation = None
        msg = String()
        msg.data = self.status_msg
        self.pub_status.publish(msg)
        self.node.get_logger().info(f"[Superviseur] {self.name} : Attente validation...")

    def _cb(self, msg):
        self.confirmation = msg.data

    def update(self):
        if self.confirmation is None:

            return py_trees.common.Status.RUNNING

        if self.confirmation:
            self.node.get_logger().info(f"[Superviseur] {self.name} : Validé (OUI).")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"[Superviseur] {self.name} : Refusé (NON).")
            return py_trees.common.Status.FAILURE

class CatchObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="Catch Gripper"):
        super(CatchObject, self).__init__(name)
        self.node = None
        self.pub_catch = None
        self.start_time = None
        self.duration = 4.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_catch = self.node.create_publisher(Bool, '/catch', 10)

    def initialise(self):
        self.node.get_logger().info("[Action] ACTIVATION PINCE !")
        msg = Bool()
        msg.data = True
        self.pub_catch.publish(msg)
        self.start_time = self.node.get_clock().now()

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            self.node.get_logger().info("[Action] Objet attrapé.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ToggleExploration(py_trees.behaviour.Behaviour):
    def __init__(self, name="Toggle Explore", enable=True, min_publish_time=0.5):
        super().__init__(name)
        self.enable = enable
        self.min_publish_time = min_publish_time
        self.node = None
        self.pub = None
        self.start_time = None
        self.has_logged = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(Bool, 'explore/resume', 10)

    def initialise(self):
        self.start_time = self.node.get_clock().now()
        self.has_logged = False

    def update(self):
        # Publie à chaque tick (10 Hz chez toi)
        msg = Bool()
        msg.data = self.enable
        self.pub.publish(msg)

        if not self.has_logged:
            state = "ON" if self.enable else "OFF"
            self.node.get_logger().info(f"[Action] Exploration {state} (publication en cours...)")
            self.has_logged = True

        # Attend qu'il y ait au moins 1 subscriber (explore_lite)
        if self.pub.get_subscription_count() == 0:
            return py_trees.common.Status.RUNNING

        # Et publie pendant un court délai pour être sûr que le message passe
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.min_publish_time:
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS

class ForceFailure(py_trees.behaviour.Behaviour):
    def __init__(self, name="Force Fail"):
        super(ForceFailure, self).__init__(name)

    def update(self):
        return py_trees.common.Status.FAILURE

class OpenGripper(py_trees.behaviour.Behaviour):
    """
    Envoie le signal False sur /catch pour ouvrir la pince.
    """
    def __init__(self, name="Ouvrir Pince"):
        super(OpenGripper, self).__init__(name)
        self.node = None
        self.pub_catch = None
        self.start_time = None
        self.duration = 2.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_catch = self.node.create_publisher(Bool, '/catch', 10)

    def initialise(self):
        self.node.get_logger().info("[Action] OUVERTURE PINCE !")
        msg = Bool()
        msg.data = False # False = Ouvrir
        self.pub_catch.publish(msg)
        self.start_time = self.node.get_clock().now()

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING