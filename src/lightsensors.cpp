#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/LightSensorValues.h"
#include <fstream>
using namespace ros;

int main(int argc, char **argv)
{
	init(argc,argv,"lightsensors");
	NodeHandle n;

	Publisher pub = n.advertise<raspimouse_ros_2::LightSensorValues>("lightsensors", 5);

	ros::Rate loop_rate(10);
	raspimouse_ros_2::LightSensorValues msg;
	while(ok()){
		std::ifstream ifs("/dev/rtlightsensor0");
		ifs >> msg.right_forward >> msg.right_side >> msg.left_side >> msg.left_forward;
		msg.sum_forward = msg.left_forward + msg.right_forward;
		msg.sum_all = msg.sum_forward + msg.left_side + msg.right_side;

		pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

