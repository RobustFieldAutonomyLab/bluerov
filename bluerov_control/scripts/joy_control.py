#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from bluerov_bridge import Bridge


max_vel = 0.2
max_omega = 0.15
vel_to_cmd = 60 / 0.2
omega_to_cmd = 60 / 0.15


def cmd_vel_sub(msg):
    vel_x, vel_y = msg.linear.x, msg.linear.y
    omega_z = msg.angular.z

    vel_x = max(-max_vel, min(max_vel, vel_x))
    vel_y = max(-max_vel, min(max_vel, vel_y))
    omega_z = max(-max_omega, min(max_omega, omega_z))

    x = 1500 + int(vel_to_cmd * vel_x)
    y = 1500 + int(vel_to_cmd * vel_y)
    z = 65535
    yaw = 1500 + int(omega_to_cmd * omega_z)
    bridge.set_cmd_vel(x, y, z, yaw)


def joy_callback(msg):
    # Arm / disarm
    if msg.buttons[0]:
        bridge.arm_throttle(True)
    if msg.buttons[1]:
        bridge.arm_throttle(False)

    # Depth hold / manual
    if msg.buttons[2]:
        bridge.set_mode('alt_hold')
    if msg.buttons[9]:
        bridge.set_mode('manual')

    # Movement
    x1 = 1500 + int(msg.axes[1] * limit)
    y1 = 1500 + int(msg.axes[0] * limit)
    z = 1500 + int(msg.axes[5] * limit)
    yaw1 = 1500 - int(msg.axes[2] * limit)

    # Cruise control
    x2 = 1500 + int(msg.axes[3] * limit)
    y2 = 1500
    yaw2 = 1500 - int(msg.axes[4] * limit)

    # Normal movement has higher priority
    x = x1 if x1 != 1500 else x2
    y = y1 if y1 != 1500 else y2
    yaw = yaw1 if yaw1 != 1500 else yaw2

    bridge.set_cmd_vel(x, y, z, yaw)


if __name__ == '__main__':
    rospy.init_node('bluerov_cruise_control_node')

    limit = rospy.get_param('~pwm_limit', 100)

    device = 'udp:192.168.2.1:14553'
    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(device)
            )
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)
    bridge.wait_conn()

    joy_sub = rospy.Subscriber('/joy', Joy, joy_callback, queue_size=10)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_sub)

    while not rospy.is_shutdown():
        bridge.set_mode('manual')
        bridge.arm_throttle(False)
        mode, arm = bridge.get_mode()
        if mode == 'MANUAL' and not arm:
            break
        rospy.sleep(0.5)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        bridge.update()
        rate.sleep()

    while not rospy.is_shutdown():
        bridge.set_mode('manual')
        mode, arm = bridge.arm_throttle(False)
        if mode == 'MANUAL' and not arm:
            break
        rospy.sleep(0.5)
