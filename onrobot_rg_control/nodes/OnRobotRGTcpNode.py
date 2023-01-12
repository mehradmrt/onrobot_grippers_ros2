#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import onrobot_rg_modbus_tcp.comModbusTcp
from onrobot_rg_control.baseOnRobotRG import OnRobotBaseRG
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput


class OnRobotRGTcpNode(OnRobotBaseRG):

    def __init__(self):
        # init the node and fetch some params
        super().__init__('OnRobotRGTcpNode')

        self.ip = self.declare_parameter('/onrobot/ip', '192.168.1.1')
        self.port = self.declare_parameter('/onrobot/port', '502')
        self.dummy = self.declare_parameter('/onrobot/dummy', False)

        # Gripper is a RG gripper with a Modbus/TCP connection
        self.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(self.dummy)
        self.prev_msg = []

        # the communication lib needs access to the node's logger
        self.logger = self.get_logger()

        # Connects to the ip address received as an argument
        self.client.connectToDevice(self.ip, self.port)

        # The Gripper status is published on the topic named 'OnRobotRGInput'
        self.pub = self.create_publisher(OnRobotRGInput, 'OnRobotRGInput', 1)

        timer_period = 3  # seconds
        #timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # The Gripper command is received from the topic named 'OnRobotRGOutput'
        self.sub = self.create_subscription(
            OnRobotRGOutput,
            'OnRobotRGOutput',
            self.refreshCommand,
            10)

    def timer_callback(self):
        status = self.getStatus()
        self.pub.publish(status)

        # Send the most recent command
        if not int(format(status.g_sta, '016b')[-1]):  # not busy
            if not self.prev_msg == self.message:       # find new message
                self.get_logger().info(self.get_name()+": Sending message.")
                self.sendCommand()
        self.prev_msg = self.message

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGTcpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

