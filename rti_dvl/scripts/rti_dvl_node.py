#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from rti_dvl.msg import DVL

import pynmea2
import serial


def zeor_pressure(req):
    rospy.loginfo('Zero pressure sensor')

    dvl.write('CPZ\r\n')
    return TriggerResponse(success=True)


def toggle_status(req):
    global running
    if running:
        rospy.loginfo('Stop DVL')
        dvl.write('STOP\r\n')
        running = False
        return TriggerResponse(success=True, message='Stop')
    else:
        rospy.loginfo('Start DVL')
        dvl.write('START\r\n')
        running = True
        return TriggerResponse(success=True, message='Start')


if __name__ == '__main__':
    rospy.init_node('rti_dvl_node')

    dev = rospy.get_param('dev', '/tmp/rti_dvl')

    # eoutput = rospy.get_param('eoutput', 5)
    # ei = rospy.get_param('ei', 0.25)

    # btbb_pulse_type = rospy.get_param('btbb_pulse_type', 4)
    # btbb_lag_length = rospy.get_param('btbb_lag_length', 4.0)
    # btbb_depth_switch = rospy.get_param('btbb_depth_switch', 30.0)
    # btbb_beam_multiplex = rospy.get_param('btbb_beam_multiplex', 4)
    # btmx_depth = rospy.get_param('btmx_depth', 5.0)
    # btst_correlation = rospy.get_param('btst_correlation', 0.9)
    # btst_error_velocity = rospy.get_param('btst_error_velocity', 10.0)
    # btst_velocity = rospy.get_param('btst_velocity', 10.0)
    # btbl_shallow = rospy.get_param('btbl_shallow', 0.1)
    # btbl_deep = rospy.get_param('btbl_deep', 10.0)
    # bttbp_interval = rospy.get_param('bttbp_inverval', 0.13)
    # btt_snr_shallow = rospy.get_param('btt_snr_shallow', 30.0)
    # btt_depth_shallow_switch = rospy.get_param('btt_depth_shallow_switch',
    #                                            50.0)
    # btt_snr_deep = rospy.get_param('btt_snr_deep', 20.0)
    # btt_depth_deep_switch = rospy.get_param('btt_depth_deep_switch', 4.0)

    # wssc_temperature = rospy.get_param('cwssc_temperature', 1)
    # wssc_depth = rospy.get_param('cwssc_depth', 1)
    # wssc_salinity = rospy.get_param('cwssc_salinity', 0)
    # wssc_sound_speed = rospy.get_param('cwssc_sound_speed', 2)
    # ws = rospy.get_param('cws', 0.0)
    # td = rospy.get_param('ctd', 0.0)
    # wt = rospy.get_param('cwt', 15.0)
    # wss = rospy.get_param('cwss', 1500.0)

    # zps_srv = rospy.Service('~zero_pressure', Trigger, zeor_pressure)
    # ts_srv = rospy.Service('~toggle_status', Trigger, toggle_status)

    global running
    running = False
    while not rospy.is_shutdown():
        try:
            rospy.loginfo('Open device {}'.format(dev))
            dvl = serial.Serial(dev, dsrdtr=True, rtscts=True, timeout=1.0)
        except serial.SerialException:
            rospy.logerr('Fail to open device {}'.format(dev))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)

    # #DVL Commands
    # http://rowetechinc.co/wiki/index.php?title=ADCP_Commands
    # dvl.write('C485OUT {:1d}\r\rn'.format(eoutput))
    # dvl.write('CEI 00:00:' + '%05.2f\r\n' % ei)
    # dvl.write('CBTBB[0] {:1d},{:.2f},{:.2f},{:1d}\r\n'.format(
    #     btbb_pulse_type, btbb_lag_length, btbb_depth_switch,
    #     btbb_beam_multiplex))
    # dvl.write('CBTST[0] {:.3f},{:.3f},{:.3f}\r\n'.format(
    #     btst_correlation, btst_error_velocity, btst_velocity))
    # dvl.write('CBTBL[0] {:2.f},{:.2f}\r\n'.format(btbl_shallow, btbl_deep))
    # dvl.write('CBTMX[0] {:.2f}\r\n'.format(btmx_depth))
    # dvl.write('CBTTBP[0] {:.2f}\r\n'.format(bttbp_interval))
    # dvl.write('CBTT[0] {:.1f},{:.1f},{:.1f},{:.1f}\r\n'.format(
    #     btt_snr_shallow, btt_depth_shallow_switch, btt_snr_deep,
    #     btt_depth_deep_switch))
    # dvl.write('CWSSC {:1d},{:1d},{:1d},{:1d}\r\n'.format(
    #     wssc_temperature, wssc_depth, wssc_salinity, wssc_sound_speed))
    # dvl.write('CWS {:.2f}\r\n'.format(ws))
    # dvl.write('CWT {:.2f}\r\n'.format(wt))
    # dvl.write('CTD {:.2f}\r\n'.format(td))
    # dvl.write('CWSS {:.2f}\r\n'.format(wss))
    # dvl.write('CSAVE\r\n')

    # running = True
    # if not toggle_status(TriggerRequest).success:
    #     rospy.logerr('Fail to stop DVL')
    # else:
    #     running = False

    dvl_pub = rospy.Publisher('/rti/body_velocity/raw', DVL, queue_size=100)
    reader = pynmea2.NMEAStreamReader(errors='ignore')

    while not rospy.is_shutdown():
        char = dvl.read()
        for msg in reader.next(char):
            if isinstance(msg, pynmea2.types.rti.RTI01):
                d = DVL()
                d.header.stamp = rospy.Time.now()
                d.velocity.x = msg.x / 1000.0
                d.velocity.y = msg.y / 1000.0
                d.velocity.z = msg.z / 1000.0
                d.temperature = msg.temperature / 100.0
                d.transducer_depth = msg.depth / 1000.0
                d.time = msg.time / 100.0
                dvl_pub.publish(d)
    dvl.close()
