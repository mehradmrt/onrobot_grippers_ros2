#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGOutput


def genCommand(char, command):
    """Updates the command according to the character entered by the user."""

    if gtype == 'rg2':
        max_force = 400
        max_width = 1100
    elif gtype == 'rg6':
        max_force = 1200
        max_width = 1600
    else:
        rospy.signal_shutdown(
            rospy.get_name() +
            ": Select the gripper type from rg2 or rg6.")

    if char == 'c':
        command.r_gfr = 400
        command.r_gwd = 0
        command.r_ctr = 16
    elif char == 'o':
        command.r_gfr = 400
        command.r_gwd = max_width
        command.r_ctr = 16
    elif char == 'i':
        command.r_gfr += 25
        command.r_gfr = min(max_force, command.r_gfr)
        command.r_ctr = 16
    elif char == 'd':
        command.r_gfr -= 25
        command.r_gfr = max(0, command.r_gfr)
        command.r_ctr = 16
    else:
        # If the command entered is a int, assign this value to r_gwd
        try:
            command.r_gfr = 400
            command.r_gwd = min(max_width, int(char))
            command.r_ctr = 16
        except ValueError:
            pass

    return command


def askForCommand(command):
    """Asks the user for a command to send to the gripper."""

    currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
    currentCommand += ' r_gfr = ' + str(command.r_gfr)
    currentCommand += ', r_gwd = ' + str(command.r_gwd)
    currentCommand += ', r_ctr = ' + str(command.r_ctr)

    rospy.loginfo(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0 - max width): Go to that position\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """Main loop which requests new commands and
       publish them on the OnRobotRGOutput topic.
    """

    rospy.init_node('OnRobotRGSimpleController', log_level=rospy.DEBUG)
    pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    command = OnRobotRGOutput()

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    gtype = rospy.declare_parameter('/onrobot/gripper', 'rg6')
    publisher()



def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGStatusListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

