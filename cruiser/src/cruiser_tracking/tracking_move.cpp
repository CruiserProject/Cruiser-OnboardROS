/*
 * tracking_move.cpp
 *
 *  Created on: May 10, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_drone.h>

using namespace DJI;
using namespace DJI::onboardSDK;

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);

DJIDrone *drone;


bool tracking_flag = false;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"tracking_move_node");
	ros::NodeHandle nh;

	drone = new DJIDrone(nh);
	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/tracking_move",1,&DeltaMsgCallback);
    ros::Rate rate(2);

	if(drone->request_sdk_permission_control())
		ROS_INFO_STREAM("tracking_move_node : initialization and get control.");

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{
	if (new_location.state)
	{
		if((new_location.delta_X_meter < -99) && (new_location.delta_Y_meter < -99))
		{
			drone->gimbal_angle_control(0, -450, 0, 10);
			usleep(10000);
		}

		else
		{
			drone->attitude_control(0x81,new_location.delta_X_meter,new_location.delta_Y_meter,0,0);//location
			usleep(500000);
		}

	}
}
