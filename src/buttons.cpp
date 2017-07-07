#include "ros/ros.h"
#include <ros/package.h> 
#include "raspimouse_ros_2/ButtonValues.h"
#include <fstream>
using namespace ros;

bool readButton(const char *name)
{
	std::ifstream ifs(name);
	char c;
	ifs >> c;
	return c == '0';
}

int main(int argc, char **argv)
{
	init(argc,argv,"buttons");
	NodeHandle n;

	Publisher pub = n.advertise<raspimouse_ros_2::ButtonValues>("buttons", 5);

	ros::Rate loop_rate(10);
	raspimouse_ros_2::ButtonValues msg;
	int c[3] = {0,0,0};
	while(ok()){
		msg.front = readButton("/dev/rtswitch0");
		msg.mid = readButton("/dev/rtswitch1");
		msg.rear = readButton("/dev/rtswitch2");

		c[0] = msg.front ? 1+c[0] : 0;
		c[1] = msg.mid ? 1+c[1] : 0;
		c[2] = msg.rear ? 1+c[2] : 0;

		if(c[0] > 4){
			msg.front_toggle = not msg.front_toggle;
			c[0] = 0;
		}
		if(c[1] > 4){
			msg.mid_toggle = not msg.mid_toggle;
			c[1] = 0;
		}
		if(c[2] > 4){
			msg.rear_toggle = not msg.rear_toggle;
			c[2] = 0;
		}

		pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}
	exit(0);
}

