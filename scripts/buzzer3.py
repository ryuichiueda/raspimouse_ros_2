#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
def write_freq(hz=0):
    bfile = "/dev/rtbuzzer0"
    try:
        with open(bfile,"w") as f:
            f.write(str(hz) + "\n")
    except IOError:
        rospy.logerr("can't write to " + bfile)
def recv_buzzer(data):
    write_freq(data.data)
if __name__ == '__main__':
    rospy.init_node('buzzer')
    rospy.Subscriber("buzzer", UInt16, recv_buzzer)
    rospy.spin()
