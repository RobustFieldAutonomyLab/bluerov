#!/usr/bin/env python
import rospy
import subprocess

if __name__ == '__main__':
    rospy.init_node('connect_rti_dvl')

    ip = rospy.get_param('ip', '192.168.2.2')
    port = rospy.get_param('port', 14661)
    dev = rospy.get_param('dev', '/tmp/rti_dvl')

    cmd = 'socat pty,link={},waitslave,raw,echo=0,ignoreeof tcp:{}:{}'.format(
        dev, ip, port)
    try:
        process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
    except OSError as e:
        rospy.logerr("Can't create pty. socat isn't installed?")
    rospy.loginfo('pty link created: {}'.format(dev))
    process.communicate()
