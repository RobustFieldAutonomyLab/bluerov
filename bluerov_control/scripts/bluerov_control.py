#!/usr/bin/env python
from pymavlink import mavutil
import rospy
from geometry_msgs.msg import Twist


def normalize(vel, min_vel, max_vel, min_pwm, max_pwm):
    if abs(max_vel) < 1e-3:
        return 1500
    return int(vel / max_vel * (max_pwm - 1500) + 1500)


def cmd_vel_sub(msg):
    # Ignore vz, wx, wy
    vel_x, vel_y, _ = msg.linear.x, msg.linear.y, msg.linear.y
    _, _, vel_theta = msg.angular.x, msg.angular.y, msg.angular.z

    pwm = [
        1500, 1500, 1500,
        normalize(vel_theta, min_vel_theta, max_vel_theta, min_pwm_theta,
                  max_pwm_theta),
        normalize(vel_x, min_vel_x, max_vel_x, min_pwm_x, max_pwm_x),
        normalize(vel_y, min_vel_y, max_vel_y, min_pwm_y, max_pwm_y), 1100,
        1500
    ]
    if pwm[3] > 1520:
        pwm[3] = 1560
    if pwm[3] < 1480:
        pwm[3] = 1440
    rospy.loginfo('pwm {}'.format(pwm))

    conn.mav.rc_channels_override_send(conn.target_system,
                                       conn.target_component, *pwm)


def test():
    rate = rospy.Rate(100)
    pwm = [1500, 1500, 1500, 1440, 1500, 1500, 1100, 1500]
    for _ in range(500):
        conn.mav.rc_channels_override_send(conn.target_system,
                                           conn.target_component, *pwm)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('bluerov_control_node')

    device = rospy.get_param('device', 'udp:192.168.2.1:14553')
    cmd_topic = rospy.get_param('cmd_vel', '/cmd_vel')
    min_pwm_x = rospy.get_param('min_pwm_x', 1300)
    max_pwm_x = rospy.get_param('max_pwm_x', 1600)
    rospy.loginfo('pwm_x: {}, {}'.format(min_pwm_x, max_pwm_x))
    min_pwm_y = rospy.get_param('min_pwm_y', 1400)
    max_pwm_y = rospy.get_param('max_pwm_y', 1600)
    rospy.loginfo('pwm_y: {}, {}'.format(min_pwm_y, max_pwm_y))
    min_pwm_theta = rospy.get_param('min_pwm_theta', 1400)
    max_pwm_theta = rospy.get_param('max_pwm_theta', 1600)
    rospy.loginfo('pwm_theta: {}, {}'.format(min_pwm_theta, max_pwm_theta))

    planner = '/move_base/DWAPlannerROS/'
    min_vel_x = rospy.get_param(planner + 'min_vel_x', -0.3)
    max_vel_x = rospy.get_param(planner + 'max_vel_x', 0.3)
    rospy.loginfo('cmd_x: {}, {}'.format(min_vel_x, max_vel_x))
    min_vel_y = rospy.get_param(planner + 'min_vel_y', -0.3)
    max_vel_y = rospy.get_param(planner + 'max_vel_y', 0.3)
    rospy.loginfo('cmd_y: {}, {}'.format(min_vel_y, max_vel_y))
    min_vel_theta = rospy.get_param(planner + 'min_rot_vel', -1.0)
    max_vel_theta = rospy.get_param(planner + 'max_rot_vel', 1.0)
    rospy.loginfo('cmd_theta: {}, {}'.format(min_vel_theta, max_vel_theta))

    conn = mavutil.mavlink_connection(device, write=True, autoreconnect=True)
    while not rospy.is_shutdown():
        msg = conn.recv_match()
        if msg is not None:
            rospy.loginfo('Connected to device {}'.format(device))
            break
        else:
            rospy.sleep(1.0)
    # test()
    # import sys
    # sys.exit(0)

    cmd_vel_sub = rospy.Subscriber(cmd_topic, Twist, cmd_vel_sub)

    rospy.spin()
    conn.close()
