#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/LedValues.h"
#include <fstream>
using namespace ros;

void output(std::ofstream *ofs, bool input)
{
	*ofs << (input ? '1' : '0') << std::endl;
}

void cb(const raspimouse_ros_2::LedValues::ConstPtr& msg)
{
	std::ofstream ofs0("/dev/rtled0");
	output(&ofs0, msg->right_side);
	std::ofstream ofs1("/dev/rtled1");
	output(&ofs1, msg->right_forward);
	std::ofstream ofs2("/dev/rtled2");
	output(&ofs2, msg->left_forward);
	std::ofstream ofs3("/dev/rtled3");
	output(&ofs3, msg->left_side);
}


int main(int argc, char **argv)
{
	init(argc,argv,"leds");
	NodeHandle n;
	Subscriber sub = n.subscribe("leds", 10, cb);

	ros::spin();
	exit(0);
}

