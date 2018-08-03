#!/usr/bin/env python
import random
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fake_tf_node')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0), [0, 0, 0, 1], rospy.Time.now(),
                         'base_link', 'map')
        rate.sleep()
