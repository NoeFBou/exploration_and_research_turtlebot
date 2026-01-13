import rclpy
import py_trees
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32,String
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import time

class WaitDuration(py_trees.behaviour.Behaviour):
    def __init__(self, name="Timer", duration=360.0):
        super(WaitDuration, self).__init__(name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            return py_trees.common.Status.SUCCESS # <--- C'est la clé !
        return py_trees.common.Status.RUNNING

class VisualServoingApproach(py_trees.behaviour.Behaviour):
    """
    Approche finale 'manuelle' basée sur la position de l'objet.
    Remplace la 'control_loop' de votre ancien supervisor_node.
    """

    def __init__(self, name="Visual Servoing"):
        super(VisualServoingApproach, self).__init__(name)
        # On lit la position de l'objet (sauvegardée dans la MAP) depuis le blackboard
        self.blackboard = py_trees.blackboard.Client(name="Action")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

        self.node = None
        self.cmd_vel_pub = None
        self.tf_buffer = None
        self.tf_listener = None

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
            # On cherche la transformation map -> base_link
            # Attention : pour savoir où est l'objet par rapport au robot,
            # on transforme de MAP vers BASE_LINK
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time()
            )
            target_local = do_transform_pose(target_map.pose, transform)
        except Exception as e:
            # Si TF échoue, on continue d'essayer sans planter
            return py_trees.common.Status.RUNNING

        # Coordonnées dans le repère du robot (X=Devant, Y=Gauche)
        x_dist = target_local.position.x  # Distance devant
        y_lat  = target_local.position.y  # Décalage latéral

        # --- DEBUG LOG ---
        # On affiche la distance pour comprendre pourquoi il s'arrête
        # self.node.get_logger().info(f"[Visual] Dist X={x_dist:.2f}m, Y={y_lat:.2f}m")

        # Condition de Succès (22cm)
        if x_dist <= 0.22:
            self.node.get_logger().info(f"[Visual] Stop final (Dist={x_dist:.2f}m).")
            self.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        # Sécurité : Si l'objet est DERRIERE le robot (X négatif) ou trop loin (>3m en visuel)
        if x_dist < 0:
            # On recule doucement ou on pivote ? Pour l'instant on s'arrête pour pas faire de bêtise
            self.cmd_vel_pub.publish(Twist())
            self.node.get_logger().warn("[Visual] Objet derrière ! Echec approche.")
            return py_trees.common.Status.FAILURE

        # Commande Moteur (Asservissement simple)
        twist = Twist()
        twist.linear.x = 0.15 if x_dist > 0.4 else 0.05
        twist.angular.z = 1.5 * y_lat # Gain proportionnel

        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Si on est interrompu ou qu'on a fini, on arrête les moteurs
        if new_status != py_trees.common.Status.RUNNING:
            self.cmd_vel_pub.publish(Twist())


class CatchObject(py_trees.behaviour.Behaviour):
    """
    Active le noeud 'catch_node' et attend la fin de l'action.
    """

    def __init__(self, name="Catch Gripper"):
        super(CatchObject, self).__init__(name)
        self.node = None
        self.pub_catch = None
        self.start_time = None
        self.duration = 4.0  # Temps pour fermer la pince (sec)

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
        # On attend simplement X secondes que la pince se ferme
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed > self.duration:
            self.node.get_logger().info("[Action] Objet attrapé (timeout).")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


# class ToggleExploration(py_trees.behaviour.Behaviour):
#     """
#     Active/Désactive explore_lite.
#     """
#
#     def __init__(self, name="Toggle Explore", enable=True):
#         super(ToggleExploration, self).__init__(name)
#         self.enable = enable
#         self.node = None
#         self.pub = None
#
#     def setup(self, **kwargs):
#         self.node = kwargs.get('node')
#         self.pub = self.node.create_publisher(Bool, 'explore/resume', 10)
#
#     def initialise(self):
#         msg = Bool()
#         msg.data = self.enable
#         self.pub.publish(msg)
#         state = "ON" if self.enable else "OFF"
#         self.node.get_logger().info(f"[Action] Exploration {state}")
#
#     def update(self):
#         # Une fois le message envoyé, l'action est considérée comme faite
#         return py_trees.common.Status.SUCCESS


class WaitForUserSelection(py_trees.behaviour.Behaviour):
    """
    Attend qu'un ID soit reçu sur le topic '/mission/select_target'.
    Non-bloquant pour l'arbre.
    """

    def __init__(self, name="Attente Choix"):
        super(WaitForUserSelection, self).__init__(name)
        self.node = None
        self.sub = None
        self.received_id = -1  # -1 signifie "rien reçu"

        self.blackboard = py_trees.blackboard.Client(name="Interface")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # On écoute les commandes de l'utilisateur
        self.sub = self.node.create_subscription(
            Int32,
            '/mission/select_target',
            self._msg_callback,
            10
        )

    def _msg_callback(self, msg):
        self.node.get_logger().info(f"[UI] Commande reçue : ID {msg.data}")
        self.received_id = msg.data

    def initialise(self):
        self.received_id = -1
        self.node.get_logger().info("[UI] En attente d'une sélection sur /mission/select_target ...")

    def update(self):
        # 1. Si on n'a rien reçu, on renvoie RUNNING (l'arbre continue de vivre)
        if self.received_id == -1:
            return py_trees.common.Status.RUNNING

        # 2. Si on a reçu un ID, on vérifie s'il est valide
        objects = self.blackboard.known_objects
        if not objects:
            return py_trees.common.Status.FAILURE

        target_id = self.received_id
        selected_obj = next((o for o in objects if o['id'] == target_id), None)

        if selected_obj:
            self.node.get_logger().info(f"[UI] Cible #{target_id} validée via Topic.")
            self.blackboard.target_pose_map = selected_obj['pose']
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn(f"[UI] ID {target_id} invalide ou inconnu !")
            self.received_id = -1  # On reset et on attend une nouvelle commande
            return py_trees.common.Status.RUNNING


# Mettez à jour ces deux classes dans actions.py

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
        # CORRECTION : On remet à False quand ce behavior (re)commence
        # Si l'arbre plante et redémarre, le robot attendra un nouvel appui sur Entrée
        # au lieu de devenir fou.
        self.start_received = False
        self.node.get_logger().info("[Superviseur] En attente du signal START...")

    def _cb(self, msg):
        if msg.data:
            self.start_received = True

    def update(self):
        return py_trees.common.Status.SUCCESS if self.start_received else py_trees.common.Status.RUNNING

# Ajoutez String aux imports en haut
from std_msgs.msg import Bool, Int32, String

class WaitForConfirmation(py_trees.behaviour.Behaviour):
    """
    Signale au contrôleur que le robot est arrivé et attend une validation (Oui/Non).
    """
    def __init__(self, name="Validation Humaine"):
        super(WaitForConfirmation, self).__init__(name)
        self.node = None
        self.pub_status = None
        self.sub_conf = None
        self.confirmation = None # None = attente, True = Oui, False = Non

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # Topic pour dire au contrôleur "Je suis là, demande à l'humain"
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)
        # Topic pour recevoir la réponse de l'humain
        self.sub_conf = self.node.create_subscription(Bool, '/mission/confirmation', self._cb, 10)

    def initialise(self):
        self.confirmation = None
        # On envoie le signal "ARRIVED"
        msg = String()
        msg.data = "WAITING_CONFIRMATION"
        self.pub_status.publish(msg)
        self.node.get_logger().info("[Superviseur] Arrivé devant l'objet. Attente validation...")

    def _cb(self, msg):
        self.confirmation = msg.data

    def update(self):
        if self.confirmation is None:
            # On renvoie le statut régulièrement au cas où le contrôleur l'ait raté
            # msg = String()
            # msg.data = "WAITING_CONFIRMATION"
            # self.pub_status.publish(msg)
            return py_trees.common.Status.RUNNING

        if self.confirmation:
            self.node.get_logger().info("[Superviseur] Validation reçue ! On attrape.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("[Superviseur] Refusé. Retour au menu.")
            return py_trees.common.Status.FAILURE

class ToggleExploration(py_trees.behaviour.Behaviour):
    """
    Active/Désactive explore_lite.
    Envoie le message en continu (si appelé en boucle) pour gérer les retards au démarrage,
    mais ne spamme pas les logs.
    """
    def __init__(self, name="Toggle Explore", enable=True):
        super(ToggleExploration, self).__init__(name)
        self.enable = enable
        self.node = None
        self.pub = None
        self.has_logged = False # Mémoire interne pour le log

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(Bool, 'explore/resume', 10)

    def initialise(self):
        # On ne reset PAS has_logged ici si on veut que ça reste silencieux
        # tant qu'on est dans la même phase.
        pass

    def update(self):
        # 1. On publie TOUJOURS (Martèlement)
        msg = Bool()
        msg.data = self.enable
        self.pub.publish(msg)

        # 2. On loggue UNE SEULE FOIS
        if not self.has_logged:
            state = "ON" if self.enable else "OFF"
            self.node.get_logger().info(f"[Action] Exploration {state} (commande envoyée)")
            self.has_logged = True

        return py_trees.common.Status.SUCCESS

class WaitForSkipSignal(py_trees.behaviour.Behaviour):
    """
    Attend un signal sur '/mission/skip_nav' pour renvoyer SUCCESS immédiatement.
    Utilisé en parallèle avec la navigation pour permettre une interruption manuelle.
    """
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
        if msg.data:
            self.node.get_logger().info("[Superviseur] SKIP REÇU ! Annulation de la navigation...")
            self.skip_received = True

    def update(self):
        if self.skip_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING