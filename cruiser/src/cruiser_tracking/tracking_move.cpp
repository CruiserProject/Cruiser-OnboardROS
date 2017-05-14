/*
 * tracking_move.cpp
 *
 *  Created on: May 10, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdlib>
#include <stdlib.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_drone.h>

using namespace DJI;
using namespace DJI::onboardSDK;

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);

DJIDrone *drone;

bool alti_flag = false;
bool delta_pos = false;
float Height;
float Height_Last;
bool landing_flag = false;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"tracking_move_node");
	ros::NodeHandle nh;

	drone = new DJIDrone(nh);

	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/tracking_move",1,&DeltaMsgCallback);

    ros::Rate rate(1);

	if(drone->request_sdk_permission_control())
		ROS_INFO_STREAM("tracking_move_node : initialization and get control.");

    while(ros::ok())
    {
    	if(landing_flag)
    		drone->gimbal_angle_control(0, -450, 0, 20);
        ros::spinOnce();
        rate.sleep();
    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{
	landing_flag = true;
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










