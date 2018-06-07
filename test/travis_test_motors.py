#!/usr/bin/env python
#encoding: utf8
import unittest, rostest
import rosnode, rospy
import time
from raspimouse_ros_2.msg import MotorFreqs
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros_2.srv import TimedMotion                         # 追加

class MotorTest(unittest.TestCase):
    def setUp(self):
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.wait_for_service('/timed_motion')                 # 追加
        on = rospy.ServiceProxy('/motor_on', Trigger)
        ret = on()

    def file_check(self,dev,value,message):
        with open("/dev/" + dev,"r") as f:
            s = f.readline()
            self.assertEqual(s,str(value)+"\n",message)

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/motors', nodes, "node does not exist")

    def test_put_freq(self):
        pub = rospy.Publisher('/motor_raw', MotorFreqs)
        m = MotorFreqs()
        m.left_hz = 123 
        m.right_hz = 456 
        for i in range(10):
            pub.publish(m)
            time.sleep(0.1)

        self.file_check("rtmotor_raw_l0",m.left_hz,"wrong left value from motor_raw")
        self.file_check("rtmotor_raw_r0",m.right_hz,"wrong right value from motor_raw")

    def test_put_cmd_vel(self):
        pub = rospy.Publisher('/cmd_vel', Twist)
        m = Twist()
        m.linear.x = 0.1414
        m.angular.z = 1.57
        for i in range(10):
            pub.publish(m)
            time.sleep(0.1)

        self.file_check("rtmotor_raw_l0",200,"wrong left value from cmd_vel")
        self.file_check("rtmotor_raw_r0",600,"wrong right value from cmd_vel")

        time.sleep(1.1)
        self.file_check("rtmotor_raw_r0",0,"don't stop after 1[s]")
        self.file_check("rtmotor_raw_l0",0,"don't stop after 1[s]")

    def test_on_off(self):
        off = rospy.ServiceProxy('/motor_off', Trigger)
        ret = off()
        self.assertEqual(ret.success, True, "motor off does not succeeded")
        self.assertEqual(ret.message, "OFF", "motor off wrong message")
        with open("/dev/rtmotoren0","r") as f:
            data = f.readline()
            self.assertEqual(data,"0\n","wrong value in rtmotor0 at motor off")

        on = rospy.ServiceProxy('/motor_on', Trigger)
        ret = on()
        self.assertEqual(ret.success, True, "motor on does not succeeded")
        self.assertEqual(ret.message, "ON", "motor on wrong message")
        with open("/dev/rtmotoren0","r") as f:
            data = f.readline()
            self.assertEqual(data,"1\n","wrong value in rtmotor0 at motor on")

    def test_put_value_timed(self):                #このメソッドを追加
        tm = rospy.ServiceProxy('/timed_motion', TimedMotion)
        tm(-321,654,1500)
        with open("/dev/rtmotor0","r") as f:
            data = f.readline()
            self.assertEqual(data,"-321 654 1500\n","value does not written to rtmotor0")

if __name__ == '__main__':
    rospy.init_node('travis_test_motors')
    rostest.rosrun('raspimouse_ros_2','travis_test_motors', MotorTest)
