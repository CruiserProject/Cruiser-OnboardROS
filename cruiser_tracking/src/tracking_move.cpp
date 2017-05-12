/*
 * tracking_move.cpp
 *
 *  Created on: May 10, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <stdio.h>
#include <stdint.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "dji_sdk_lib/DJI_API.h"
#include "dji_sdk_lib/DJICommonType.h"
#include <dji_sdk/dji_sdk_node.h>
#include <dji_sdk/LocalPosition.h>
#include <cruiser/DeltaPosition.h>
#include <cruiser/Flag.h>

using namespace DJI;
using namespace DJI::onboardSDK;

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);

DJIDrone *drone;
DJI::onboardSDK::ROSAdapter *rosAdapter;

int alti_flag = 0;
int delta_pos = 0;
float Height;
float Height_Last;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"tracking_move_node");
	ros::NodeHandle nh;

	drone = new DJIDrone(nh);
	if(drone->request_sdk_permission_control())
		ROS_INFO("Get permission control Correct!");
	ROS_INFO("A");

	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/tracking_move",1,&DeltaMsgCallback);

    ros::Rate rate(1);
    while(ros::ok())
    {
      	if(alti_flag||delta_pos)
    	{
    		drone->landing();
    		drone->release_sdk_permission_control();
//    		ros::shutdown();
    	}
        ros::spinOnce();
        rate.sleep();
    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{

	if (new_location.state)
	{
		float Velocity_X = new_location.delta_X_meter;
		float Velocity_Y = new_location.delta_Y_meter;
		for(int i = 0;i < 20; i++)
		{
			drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
			usleep(20000);
		}
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "position = (" << new_location.delta_X_meter << "," << new_location.delta_Y_meter << ")");
	}
/*
	float Velociy_X_max = new_location.delta_X * 2;
	float Velociy_Y_max = new_location.delta_Y * 2;
	float deltaV = 0.2;	//手册最小精度
	int Nx = Velociy_X_max / deltaV * 2;	//量化
	int Ny = Velociy_Y_max / deltaV * 2;	//量化
	int countN = 0;
	int deltaTx = utime / Nx;		//步进时间
	int deltaTy = utime / Ny;		//步进时间
	float Velocity_X = 0;
	float Velocity_Y = 0;

	int N;
	if(Nx > Ny)N=Nx;else N=Ny;
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "position = (" << new_location.delta_X << "," << new_location.delta_Y << ")");

	for(countN = 0 ; countN < N; countN++)
	{
		if(countN <= (Nx/2))
			Velocity_X += deltaVx;
		else
			Velocity_X -= deltaVx;
		if(countN <= (Ny/2))
			Velocity_Y += deltaVy;
		else
			Velocity_Y -= deltaVy;
		drone->attitude_control(0x40,Velocity_X,0,0,0);//水平速度
		usleep(deltaTx);
		ROS_INFO_STREAM("X is done!");
		drone->attitude_control(0x40,0,Velocity_Y,0,0);//水平速度
		usleep(deltaTy);
		ROS_INFO_STREAM("Y is done!");
	}
*/
}










