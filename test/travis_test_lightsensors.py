#!/usr/bin/env python
#encoding: utf8
import unittest, rostest
import rosnode, rospy
import time
from raspimouse_ros_2.msg import LightSensorValues
        
class LightsensorTest(unittest.TestCase):
    def setUp(self):
        self.count = 0
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)
        self.values = LightSensorValues()
        
    def callback(self,data):
        self.count += 1
        self.values = data

    def check_values(self,lf,ls,rs,rf):
        vs = self.values
        self.assertEqual(vs.left_forward, lf, "different value: left_forward")
        self.assertEqual(vs.left_side, ls, "different value: left_side")
        self.assertEqual(vs.right_side, rs, "different value: right_side")
        self.assertEqual(vs.right_forward, rf, "different value: right_forward")
        self.assertEqual(vs.sum_all, lf+ls+rs+rf, "different value: sum_all")
        self.assertEqual(vs.sum_forward, lf+rf, "different value: sum_forward")

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/lightsensors',nodes, "node does not exist")

    def test_get_value(self):
        rospy.set_param('/lightsensors/frequency',10)    #センサの値取得の周期を10Hzに
        time.sleep(2)                              #パラメータの反映を待つ
        with open("/dev/rtlightsensor0","w") as f: #ダミーの値をダミーのファイルに
            f.write("-1 0 123 4321\n")

        time.sleep(3)
        ###コールバック関数が最低1回は呼ばれ、値が取得できているかを確認###
        self.assertFalse(self.count == 0,"cannot subscribe the topic") 
        self.check_values(4321,123,0,-1) 

    def test_change_parameter(self):
        rospy.set_param('/lightsensors/frequency',1)    #センサの値取得の周期を1Hzに
        time.sleep(2)                             #反映を待つ
        c_prev = self.count                       #callbackが呼ばれた回数を記録
        time.sleep(3) 
        ###コールバック関数が3秒間で最低1回、最高でも4回しか呼ばれてないことを確認###
        self.assertTrue(self.count < c_prev + 4,"freq does not change")
        self.assertFalse(self.count == c_prev,"subscriber is stopped")

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('travis_test_lightsensors')
    rostest.rosrun('raspimouse_ros_2','travis_test_lightsensors',LightsensorTest)
