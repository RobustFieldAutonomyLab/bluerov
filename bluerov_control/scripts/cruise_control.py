#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

from bluerov_bridge import Bridge


def job_callback(msg):
    x = 1500 + int(msg.axes[3] * limit)
    y = 1500
    yaw = 1500 + int(msg.axes[4] * limit)

    bridge.set_cmd_vel(x, y, yaw)


if __name__ == '__main__':
    rospy.init_node('bluerov_cruise_control_node')

    limit = rospy.get_param('pwm_limit', 100)

    device = 'udp:192.168.2.1:14553'
    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(
                    device))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)
    bridge.update()

    joy_sub = rospy.Subscribe('/joy', Joy, joy_callback, queue_size=100)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()
