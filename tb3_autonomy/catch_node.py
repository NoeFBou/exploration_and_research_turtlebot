import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CatchNode(Node):
    def __init__(self):
        super().__init__('catch_node')
        
        # Subscriber for catch topic
        self.catch_subscriber = self.create_subscription(
            Bool,
            'catch',
            self.catch_callback,
            10
        )
        
        # Publisher for gripper control
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/commands',
            10
        )
        
        self.get_logger().info('Catch node initialized')
    
    def catch_callback(self, msg: Bool):
        """Callback when boolean message is received on catch topic"""
        if msg.data:
            self.get_logger().info('Catch signal received - closing gripper')
            self.close_gripper()
        else:
            self.get_logger().info('Opening gripper')
            self.open_gripper()
    
    def close_gripper(self):
        """Close the gripper"""
        trajectory = self.create_gripper_trajectory(position=0.0)
        self.gripper_publisher.publish(trajectory)
    
    def open_gripper(self):
        """Open the gripper"""
        trajectory = self.create_gripper_trajectory(position=0.019)
        self.gripper_publisher.publish(trajectory)
    
    def create_gripper_trajectory(self, position: float) -> JointTrajectory:
        """Create a JointTrajectory message for gripper"""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1)
        
        trajectory.points.append(point)
        return trajectory


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
