#!/usr/bin/env python
import sys
import socket
from pymavlink import mavutil
import rospy
from bar30_depth.msg import Depth

if __name__ == '__main__':
    rospy.init_node('bar30_depth_node')
    device = rospy.get_param('~device', 'udp:192.168.2.1:14552')
    water = rospy.get_param('~water', 'fresh')

    while not rospy.is_shutdown():
        try:
            conn = mavutil.mavlink_connection(
                device, write=False, autoreconnect=True)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(
                    device))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)

    while not rospy.is_shutdown():
        msg = conn.recv_match()
        if msg is not None:
            rospy.loginfo('Connected to device {}'.format(device))
            break
        else:
            rospy.sleep(1.0)
    if rospy.is_shutdown():
        sys.exit(-1)

    depth_pub = rospy.Publisher('/bar30/depth/raw', Depth, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = conn.recv_match(type='SCALED_PRESSURE2')
        if msg is not None:
            d = Depth()
            d.header.stamp = rospy.Time.now()
            d.time = msg.time_boot_ms / 1000.0
            d.pressure_abs = msg.press_abs
            d.pressure_diff = msg.press_diff
            d.temperature = msg.temperature / 100.0

            if water == 'fresh':
                d.depth = d.pressure_diff / 98.1
            elif water == 'salt':
                d.depth = d.pressure_diff / 100.05
            depth_pub.publish(d)
        rate.sleep()
    conn.close()
