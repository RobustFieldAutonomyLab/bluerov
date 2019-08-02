#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from message_filters import Subscriber, ApproximateTimeSynchronizer

import mavutil
from bluerov_bridge import Bridge


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


##########################################################################
# https://github.com/bluerobotics/companion/blob/master/tools/ping1d_mavlink_driver.py
# https://www.ardusub.com/developers/pymavlink.html
##########################################################################
def dvl_depth_callback(dvl, depth):
    a, d = dvl.altitude, depth.d
    if a + d > max_depth:
        a = 0

    bridge.conn.mav.distance_sensor_send(
        int(depth.time * 1000),
        min_altitude * 100,
        max_depth * 100,
        a * 100,
        mavutils.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,
        1,
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        0
    )


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

    joy_sub = rospy.Subscriber('/joy', Joy, joy_callback, queue_size=10)

    enable_dvl = rospy.get_param('enable_dvl', False)
    max_depth = rospy.get_param('max_depth', 10.0)
    min_altitude = rospy.get_param('min_altitude', 0.5)
    if enable_dvl:
        dvl_sub = Subscriber('/rti/body_velocity/raw', DVL)
        depth_sub = Subscriber('/bar30/depth/raw', Depth)
        sync = ApproximateTimeSynchronizer([dvl_sub, depth_sub], 10, 0.5)
        sync.registerCallback(dvl_depth_callback)

        bridge.conn.mav.param_set_send( 1, 1, "RNGFND_TYPE", 10, mavutil.mavlink.MAV_PARAM_TYPE_INT8)

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