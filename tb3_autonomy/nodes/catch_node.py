import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CatchNode(Node):
    def __init__(self):
        super().__init__('catch_node')

        self.catch_subscriber = self.create_subscription(
            Bool, 'catch', self.catch_callback, 10
        )

        self.gripper_publisher = self.create_publisher(
            JointTrajectory, '/gripper_controller/commands', 10
        )

        self.get_logger().info('Catch node initialized')

        # === CORRECTION CRASH ===
        # On stocke le timer dans une variable 'self.startup_timer'
        self.startup_timer = self.create_timer(1.0, self.startup_open)

    def startup_open(self):
        self.get_logger().info("Initialisation : Ouverture de la pince")
        self.open_gripper()

        # === CORRECTION CRASH ===
        # On annule la variable spécifique, pas la liste globale
        if self.startup_timer:
            self.startup_timer.cancel()
            self.startup_timer = None

    def catch_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Signal CATCH (True) -> Fermeture')
            self.close_gripper()
        else:
            self.get_logger().info('Signal RELEASE (False) -> Ouverture')
            self.open_gripper()

    def close_gripper(self):
        self.publish_trajectory(left=-0.01, right=0.01)

    def open_gripper(self):
        self.publish_trajectory(left=0.04, right=-0.04) # Valeurs ajustées pour bien ouvrir

    def publish_trajectory(self, left: float, right: float):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']

        point = JointTrajectoryPoint()
        point.positions = [left, right]
        point.time_from_start = Duration(sec=1)

        trajectory.points.append(point)
        self.gripper_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = CatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()