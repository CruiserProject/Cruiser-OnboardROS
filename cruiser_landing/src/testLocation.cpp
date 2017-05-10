/*
 * testLocation.cpp
 *
 *  Created on: May 9, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <cruiser/DeltaPosition.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"test_publish_location");
	ros::NodeHandle nh;

	ros::Publisher pub_deltalocaion = nh.advertise<cruiser::DeltaPosition>("cruiser/landing_move",1);

	srand(time(0));
	ros::Rate rate_pub(1);
	while(ros::ok())
	{
		cruiser::DeltaPosition new_location;
		new_location.delta_X_meter = 10 * double(rand())/double(RAND_MAX) - 5;
		new_location.delta_Y_meter = 10 * double(rand())/double(RAND_MAX) - 5;
		pub_deltalocaion.publish(new_location);

		ROS_INFO_STREAM("delta_x = "<<new_location.delta_X_meter<<" delta_y = "<<new_location.delta_Y_meter);
		rate_pub.sleep();
	}
}



