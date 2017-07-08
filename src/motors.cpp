#include <ros/ros.h>
#include <signal.h>
#include <ros/package.h> 
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "raspimouse_ros_2/MotorFreqs.h"
#include "raspimouse_ros_2/TimedMotion.h"
#include <fstream>
using namespace ros;

bool setPower(bool);
void setFreqs(int left, int right);

void onSigint(int);
bool callbackOn(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackOff(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackTimedMotion(raspimouse_ros_2::TimedMotion::Request&, raspimouse_ros_2::TimedMotion::Response&);
void callbackRaw(const raspimouse_ros_2::MotorFreqs::ConstPtr& msg);
void callbackCmdvel(const geometry_msgs::Twist::ConstPtr& msg);

bool is_on = false;
bool in_cmdvel = false;
Time last_cmdvel;

bool setPower(bool on)
{
	std::ofstream ofs("/dev/rtmotoren0");
	if(not ofs.is_open())
		return false;

	ofs << (on ? '1' : '0') << std::endl;
	is_on = on;
	return true;
}

void setFreqs(int left, int right)
{
	std::ofstream ofsL("/dev/rtmotor_raw_l0");
	std::ofstream ofsR("/dev/rtmotor_raw_r0");
	if( (not ofsL.is_open()) or (not ofsR.is_open()) ){
		ROS_ERROR("Cannot open /dev/rtmotor_raw_{l,r}0");
		return;
	}

	ofsL << left << std::endl;
	ofsR << right << std::endl;
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

void callbackRaw(const raspimouse_ros_2::MotorFreqs::ConstPtr& msg)
{
	setFreqs(msg->left_hz, msg->right_hz);
}

void callbackCmdvel(const geometry_msgs::Twist::ConstPtr& msg)
{
	double forward_hz = 80000.0*msg->linear.x/(9*3.141592);
	double rot_hz = 400.0*msg->angular.z/3.141592;
	ROS_INFO("%d %d\n",(int)round(forward_hz-rot_hz), (int)round(forward_hz+rot_hz));
	setFreqs(int(forward_hz-rot_hz), int(forward_hz+rot_hz));
	in_cmdvel = true;
	last_cmdvel = Time::now();
}

int main(int argc, char **argv)
{
	init(argc,argv,"motors");
	NodeHandle n;

	setPower(false);
	signal(SIGINT, onSigint);

	ServiceServer srv_on = n.advertiseService("motor_on", callbackOn);
	ServiceServer srv_off = n.advertiseService("motor_off", callbackOff);
	ServiceServer srv_tm = n.advertiseService("timed_motion", callbackTimedMotion); 
	
	Subscriber sub_raw = n.subscribe("motor_raw", 10, callbackRaw);
	Subscriber sub_cmdvel = n.subscribe("cmd_vel", 10, callbackCmdvel);

	Rate loop_rate(10);
	while(ok()){
		if(in_cmdvel and Time::now().toSec() - last_cmdvel.toSec() >= 1.0)
			setFreqs(0,0);

		spinOnce();
		loop_rate.sleep();
	}
	
	exit(0);
}
