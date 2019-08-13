#!/usr/bin/env python
import rospy
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from message_filters import Subscriber, ApproximateTimeSynchronizer

from pymavlink import mavutil
from bluerov_bridge import Bridge


##########################################################################
# https://github.com/bluerobotics/companion/blob/master/tools/ping1d_mavlink_driver.py
# https://www.ardusub.com/developers/pymavlink.html
##########################################################################
def dvl_depth_callback(dvl, depth):
    a, d = dvl.altitude, depth.depth
    if a + d > max_depth:
        a = 0

    rospy.loginfo('send altitude {}, {}, {}'.format(a, min_altitude, max_depth))
    bridge.conn.mav.distance_sensor_send(
        int(depth.time * 1000),
        20, 50000,
        # int(min_altitude * 100),
        # int(max_depth * 100),
        int(a * 100),
        2,
        1,
        25,
        0
    )


if __name__ == '__main__':
    rospy.init_node('altitude_hold_node')

    device = 'udpout:192.168.2.1:14555'
    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device, source_system=1, source_component=192)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(
                    device))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)
    bridge.wait_conn()

    max_depth = rospy.get_param('~max_depth', 10.0)
    min_altitude = rospy.get_param('~min_altitude', 0.5)
    dvl_sub = Subscriber('/rti/body_velocity/raw', DVL)
    depth_sub = Subscriber('/bar30/depth/raw', Depth)
    sync = ApproximateTimeSynchronizer([dvl_sub, depth_sub], 10, 0.5)
    sync.registerCallback(dvl_depth_callback)

    # bridge.conn.mav.param_set_send( 1, 1, "RNGFND_TYPE", 10, mavutil.mavlink.MAV_PARAM_TYPE_INT8)

    rospy.spin()