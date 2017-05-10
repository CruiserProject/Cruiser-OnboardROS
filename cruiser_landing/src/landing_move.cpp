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

void LocalPositionCallback(const dji_sdk::LocalPosition& LocalPosition);
void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);

DJIDrone *drone;
DJI::onboardSDK::ROSAdapter *rosAdapter;

int alti_flag = 0;
int delta_pos = 0;
float Height;
float Height_Last;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"landing_move_node");
	ros::NodeHandle nh;

	drone = new DJIDrone(nh);
//	rosAdapter = new DJI::onboardSDK::ROSAdapter;

	if(drone->request_sdk_permission_control())
		ROS_INFO("Get permission control Correct!");
	ROS_INFO("A");

//	uint8_t testdata = 97;
//	uint8_t len = sizeof(testdata);
//	rosAdapter->init("/dev/ttyUSB0",230400);//can not locate it before request_permission_control
//	rosAdapter->sendToMobile(&testdata,len);

	ros::Subscriber Height = nh.subscribe("/dji_sdk/local_position",1,&LocalPositionCallback);
	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&DeltaMsgCallback);
	ROS_INFO("D");
    while(ros::ok())
    {
      	if(alti_flag||delta_pos)
    	{
    		drone->landing();
    		drone->release_sdk_permission_control();
    		ros::shutdown();
    	}
        ros::spinOnce();

    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{

	if (new_location.flag)
	{
		float Velocity_X = new_location.delta_X;
		float Velocity_Y = new_location.delta_Y;
		for(int i = 0;i < 20; i++)
		{
			drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
			usleep(20000);
		}

		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "position = (" << new_location.delta_X << "," << new_location.delta_Y << ")");

		if((new_location.delta_X < 0.2) && (new_location.delta_Y < 0.2)&&(Height < 1.9))
			delta_pos = 1;
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

void LocalPositionCallback(const dji_sdk::LocalPosition& LocalPosition)
{
	Height_Last = Height;
	Height = LocalPosition.z;
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "altitude = "  << LocalPosition.z);
	drone->request_sdk_permission_control();

	if(Height > 0.8)
	{
		for(int i = 0;i < 50; i++)
		{
			drone->attitude_control(0x40,0,0,-2,0);//水平速度
			usleep(20000);
		}
		ROS_INFO_STREAM("I changed the height!");
	}
	else
	{
		if(abs(Height_Last-Height) < 1)	alti_flag = 1;
	}

}





