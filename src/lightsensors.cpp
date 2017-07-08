#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/LightSensorValues.h"
#include <fstream>
using namespace ros;

int getFrequency(int old, NodeHandle *n)
{
	int f;
	if(n->getParam("frequency",f) and f > 0)
		return f;

	return old;
}

int main(int argc, char **argv)
{
	init(argc,argv,"lightsensors");
	NodeHandle n("~");

	Publisher pub = n.advertise<raspimouse_ros_2::LightSensorValues>("/lightsensors", 5);

	int freq = 10;

	ros::Rate loop_rate(freq);
	raspimouse_ros_2::LightSensorValues msg;

	unsigned int c = 0;
	while(ok()){
		if(c++ % freq == 0){ // check the parapeter every 1[s]
			unsigned int old = freq;
			freq = getFrequency(freq,&n);
			if(old != freq){
				loop_rate = ros::Rate(freq);
				ROS_INFO("Lightsensor frequency: %d", freq);
			}
		}

		std::ifstream ifs("/dev/rtlightsensor0");
		ifs >> msg.right_forward >> msg.right_side
			>> msg.left_side >> msg.left_forward;

		msg.sum_forward = msg.left_forward + msg.right_forward;
		msg.sum_all = msg.sum_forward + msg.left_side + msg.right_side;

		pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

