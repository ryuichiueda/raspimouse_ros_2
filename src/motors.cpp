#include <ros/ros.h>
#include <signal.h>
#include <ros/package.h> 
#include "raspimouse_ros_2/MotorFreqs.h"
#include "std_srvs/Trigger.h"
#include "raspimouse_ros_2/TimedMotion.h"
#include <fstream>
using namespace ros;

bool setPower(bool);
void onSigint(int);
bool callbackOn(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackOff(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackTimedMotion(raspimouse_ros_2::TimedMotion::Request&, raspimouse_ros_2::TimedMotion::Response&);

bool is_on = false;

bool setPower(bool on)
{
	std::ofstream ofs("/dev/rtmotoren0");
	if(not ofs.is_open())
		return false;

	ofs << (on ? '1' : '0') << std::endl;
	is_on = on;
	return true;
}

void onSigint(int sig)
{
	setPower(false);
	exit(0);
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

bool callbackTimedMotion(raspimouse_ros_2::TimedMotion::Request& request, raspimouse_ros_2::TimedMotion::Response& response)
{
	if(not is_on){
		ROS_INFO("Motors are not enpowered");
		return false;
	}

	std::ofstream ofs("/dev/rtmotor0");
	if(not ofs.is_open()){
		ROS_ERROR("Cannot open /dev/rtmotor0");
		return false;
	}

	ofs << request.left_hz << ' ' << request.right_hz << ' '
		<< request.duration_ms << std::endl;

	response.success = true;
	return true;
}

int main(int argc, char **argv)
{
	init(argc,argv,"motors");
	NodeHandle n;

	setPower(false);

	signal(SIGINT, onSigint);

	ros::ServiceServer srv_on = n.advertiseService("motor_on", callbackOn);
	ros::ServiceServer srv_off = n.advertiseService("motor_off", callbackOff);
	ros::ServiceServer srv_tm = n.advertiseService("timed_motion", callbackTimedMotion); 
	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	exit(0);
}
