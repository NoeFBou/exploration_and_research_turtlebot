import rclpy
import py_trees
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


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
        # 1. Récupérer la cible mémorisée (dans le repère MAP)
        target_map = self.blackboard.target_pose_map
        if target_map is None:
            self.cmd_vel_pub.publish(Twist())  # Stop par sécurité
            return py_trees.common.Status.FAILURE

        # 2. Transformer cette cible dans le repère du ROBOT (base_link)
        # Cela nous permet de savoir où est l'objet PAR RAPPORT au robot maintenant
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time()
            )
            target_local = do_transform_pose(target_map.pose, transform)
        except Exception as e:
            self.node.get_logger().warn(f"[Action] Erreur TF: {e}")
            self.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.RUNNING

        # 3. Extraire les coordonnées locales (comme dans votre ancien code)
        # Note : target_local est une Pose, pas un PoseStamped ici
        x_obj = target_local.position.y  # Attention : Y robot = gauche/droite (latéral)
        z_obj = target_local.position.x  # Attention : X robot = devant (profondeur)

        # Note sur les axes :
        # Dans votre code Vision, vous utilisiez le repère caméra optique (Z=profondeur, X=latéral).
        # Ici on a transformé en 'base_link' (convention ROS standard : X=devant, Y=gauche).
        # Donc "Profondeur" = X du robot, "Latéral" = Y du robot.

        # 4. Condition de Succès (On est assez près)
        if z_obj <= 0.22:  # 22cm (Marge de sécurité pince)
            self.node.get_logger().info("[Action] Cible atteinte ! Stop.")
            self.cmd_vel_pub.publish(Twist())  # Arrêt complet
            return py_trees.common.Status.SUCCESS

        # 5. Calcul de la commande (Votre logique P-Controller)
        twist = Twist()

        # Avancer (Linear X)
        # Si on est loin (>30cm), on avance à 0.15 m/s, sinon 0.05 m/s
        twist.linear.x = 0.15 if z_obj > 0.3 else 0.05

        # Tourner (Angular Z)
        # Correction proportionnelle à l'erreur latérale (Y)
        # Le gain 2.0 peut être ajusté
        twist.angular.z = 2.0 * x_obj

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


class ToggleExploration(py_trees.behaviour.Behaviour):
    """
    Active/Désactive explore_lite.
    """

    def __init__(self, name="Toggle Explore", enable=True):
        super(ToggleExploration, self).__init__(name)
        self.enable = enable
        self.node = None
        self.pub = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(Bool, 'explore/resume', 10)

    def initialise(self):
        msg = Bool()
        msg.data = self.enable
        self.pub.publish(msg)
        state = "ON" if self.enable else "OFF"
        self.node.get_logger().info(f"[Action] Exploration {state}")

    def update(self):
        # Une fois le message envoyé, l'action est considérée comme faite
        return py_trees.common.Status.SUCCESS


# Ajoutez ceci à la fin de tb3_autonomy/behaviors/actions.py

class WaitForUserSelection(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(WaitForUserSelection, self).__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="Interface")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.WRITE)
        self.asked = False

    def initialise(self):
        self.asked = False

    def update(self):
        objects = self.blackboard.known_objects

        if not objects:
            self.node.get_logger().error("Aucun objet trouvé ! Fin de mission.")
            return py_trees.common.Status.FAILURE

        if not self.asked:
            print("\n" + "=" * 40)
            print(f"EXPLORATION TERMINÉE. {len(objects)} objets trouvés.")
            for obj in objects:
                print(f" -> ID [{obj['id']}] : X={obj['x']:.2f}, Y={obj['y']:.2f}")
            print("=" * 40)
            self.asked = True

        # Note : input() est bloquant. Dans un vrai robot, on ferait ça en asynchrone,
        # mais pour ce projet, ça mettra l'arbre en pause, ce qui est acceptable ici.
        try:
            choice = input("Entrez l'ID de l'objet à récupérer : ")
            target_id = int(choice)
            selected_obj = next((o for o in objects if o['id'] == target_id), None)

            if selected_obj:
                print(f"Cible #{target_id} validée. On y va !")
                self.blackboard.target_pose_map = selected_obj['pose']
                return py_trees.common.Status.SUCCESS
            else:
                print("ID inconnu, réessayez.")
                return py_trees.common.Status.RUNNING
        except ValueError:
            return py_trees.common.Status.RUNNING