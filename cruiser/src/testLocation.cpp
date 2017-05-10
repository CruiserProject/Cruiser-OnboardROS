/*
 * testLocation.cpp
 *
 *  Created on: May 9, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <cruiser/LandingMove.h>
#include <stdlib.h>
int main(int argc,char **argv)
{
	ros::init(argc,argv,"publish_location");
	ros::NodeHandle nh;

	ros::Publisher pub_deltalocaion = nh.advertise<cruiser::LandingMove>("location",100);

	srand(time(0));
	ros::Rate rate_pub(0.2);
	while(ros::ok())
	{
		cruiser::LandingMove new_location;
		new_location.delta_X = 10 * double(rand())/double(RAND_MAX) - 5;
		new_location.delta_Y = 10 * double(rand())/double(RAND_MAX) - 5;
		pub_deltalocaion.publish(new_location);

		ROS_INFO_STREAM("delta_x = "<<new_location.delta_X<<" delta_y = "<<new_location.delta_Y);
		rate_pub.sleep();
	}
}



