#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/MotorFreqs.h"
#include "std_srvs/Trigger.h"
#include <fstream>
using namespace ros;

bool setPower(bool on)
{
	std::ofstream ofs("/dev/rtmotoren0");
	if(not ofs.is_open())
		return false;

	ofs << (on ? '1' : '0') << std::endl;
	return true;
}

bool callbackOn(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	if(not setPower(true))
		return false;

	response.message = "ON";
	response.success = true;
	return true;
}

bool callbackOff(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	if(not setPower(false))
		return false;

	response.message = "OFF";
	response.success = true;
	return true;
}


int main(int argc, char **argv)
{
	init(argc,argv,"motors");
	NodeHandle n;
	ros::ServiceServer srv_on = n.advertiseService("motor_on", callbackOn);
	ros::ServiceServer srv_off = n.advertiseService("motor_off", callbackOff);


	ros::service::waitForService("motor_on");
	ros::service::waitForService("motor_off");

	ros::spin();
	exit(0);
}

