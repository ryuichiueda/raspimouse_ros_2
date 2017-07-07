#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/Buttons.h"
#include <fstream>
using namespace ros;

int main(int argc, char **argv)
{
	init(argc,argv,"buttons");
	NodeHandle n;

	Publisher pub = n.advertise<raspimouse_ros_2::Buttons>("buttons", 5);

	ros::Rate loop_rate(10);
	raspimouse_ros_2::Buttons msg;
	while(ok()){
		std::cout << msg << std::endl;
		spinOnce();
		loop_rate.sleep();
	}

	exit(0);
}

