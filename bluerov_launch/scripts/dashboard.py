#!/usr/bin/env python
import time
import curses

import rospy
from rostopic import ROSTopicHz
from rti_dvl.msg import DVL
from sensor_msgs.msg import Imu
from bar30_depth.msg import Depth

from bridge import Bridge

dvl, depth, imu = DVL(), Depth(), Imu()
hz = ROSTopicHz(-1)
dvl_hz, depth_hz, imu_hz = 0, 0, 0

def dvl_callback(msg):
    global dvl
    dvl = msg
    hz.callback_hz(msg, 'dvl')


def depth_callback(msg):
    global depth
    depth = msg
    hz.callback_hz(msg, 'depth')


def imu_callback(msg):
    global imu
    imu = msg
    hz.callback_hz(msg, 'imu')


if __name__ == "__main__":
    rospy.init_node('bluerov_dashboard')

    rov = Bridge('udp:192.168.2.1:14554', write=False)

    dvl_sub = rospy.Subscriber('/rti/body_velocity/raw', DVL, dvl_callback, queue_size=100)
    depth_sub = rospy.Subscriber('/bar30/depth/raw', Depth, depth_callback, queue_size=100)
    imu_sub = rospy.Subscriber('/vn100/imu/raw', Imu, imu_callback, queue_size=1000)

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    curses.curs_set(0)

    stdscr.addstr(2, 5, 'BlueROV2 Dashboard')

    n, rate = 0, rospy.Rate(10)
    while not rospy.is_shutdown():
        rov.update()

        voltage, current = rov.get_battery()
        mode, arm = rov.get_mode()
        arm = 'ARM' if arm else 'DISARM'

        # 1 Hz
        if n % 10 == 0:
            ret = hz.get_hz('dvl')
            dvl_hz = ret[0] if ret else 0.0
            ret = hz.get_hz('depth')
            depth_hz = ret[0] if ret else 0.0
            ret = hz.get_hz('imu')
            imu_hz = ret[0] if ret else 0.0
        n += 1

        stdscr.addstr(4, 5, 'Time     | ' + time.ctime())
        stdscr.addstr(5, 5, 'Battery  | {:.1f}V {:.1f}A'.format(voltage, current))
        stdscr.addstr(6, 5, 'Mode     | {} {}'.format(mode, arm))

        stdscr.addstr(8, 5, 'DVL         | {:>5.1f} Hz'.format(dvl_hz))
        stdscr.addstr(9, 5, 'Depth       | {:>5.1f} Hz'.format(depth_hz))
        stdscr.addstr(10, 5, 'IMU        | {:>5.1f} Hz'.format(imu_hz))

        stdscr.addstr(12, 5, 'Depth      | {:>5.1f} m'.format(depth.depth))
        stdscr.addstr(13, 5, 'Altitude   | {:>5.1f} m'.format(dvl.altitude))

        stdscr.addstr(15, 5, 'Velocity x | {:>5.1f} m'.format(dvl.velocity.x))
        stdscr.addstr(16, 5, '         y | {:>5.1f} m'.format(dvl.velocity.y))
        stdscr.addstr(17, 5, '         z | {:>5.1f} m'.format(dvl.velocity.z))

        stdscr.addstr(19, 5, 'Orientation roll | {:>5.1f} m'.format(depth.depth))
        stdscr.addstr(20, 5, 'Altitude  | {:>5.1f} m'.format(dvl.altitude))

        stdscr.refresh()
        rate.sleep()
    

    curses.nocbreak(); stdscr.keypad(0); curses.echo()
    curses.endwin()
    



