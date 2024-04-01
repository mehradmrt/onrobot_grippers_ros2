#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput
from sensor_msgs.msg import JointState
import numpy as np

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('rg2_joint_state_publisher')
        self.subscription = self.create_subscription(
            OnRobotRGOutput,
            '/onrobot_rg_output',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def listener_callback(self, msg):
        joint_state_msg = JointState()
        joint_state_msg.name = ['finger_joint']

        g_wdf = msg.g_wdf

        if g_wdf == 65531:
            joint_value = -0.56
        else:
            joint_value = np.interp(g_wdf, [0, 1100], [-0.56, 0.79])

        joint_state_msg.position = [joint_value]

        self.get_logger().info(f'Publishing joint state: {joint_value} for g_wdf: {g_wdf}')
        self.publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()