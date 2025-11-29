#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import Spin
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool  # <--- NOUVEL IMPORT


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        self.declare_parameter('scan_interval', 20.0)
        self.interval = self.get_parameter('scan_interval').value


        self._spin_client = ActionClient(self, Spin, 'spin')

        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)

        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.is_scanning = False

    def timer_callback(self):
        if self.is_scanning:
            return

        self.toggle_exploration(False)

        self.send_spin_goal()

    def toggle_exploration(self, state: bool):

        msg = Bool()
        msg.data = state
        self.resume_pub.publish(msg)
        action = "REPRISE" if state else "PAUSE"
        self.get_logger().info(f'Envoi de la commande : {action}')

    def send_spin_goal(self):
        if not self._spin_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Serveur Spin non disponible. Annulation.')
            self.toggle_exploration(True)
            return

        self.is_scanning = True
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = 6.28  # 360

        self._send_goal_future = self._spin_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.is_scanning = False
            self.toggle_exploration(True)
            return

        self.get_logger().info('Rotation acceptée. En cours...')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Rotation  succès.')
        else:
            self.get_logger().info(f'Rotation (Status: {status}).')

        self.toggle_exploration(True)
        self.is_scanning = False


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