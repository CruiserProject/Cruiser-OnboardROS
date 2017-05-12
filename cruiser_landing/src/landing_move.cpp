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
#include <dji_sdk/SendDataToRemoteDevice.h>

using namespace DJI;
using namespace DJI::onboardSDK;

void LocalPositionCallback(const dji_sdk::LocalPosition& LocalPosition);
void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);

DJIDrone *drone;
DJI::onboardSDK::ROSAdapter *rosAdapter;

bool alti_flag = false;
bool delta_pos = false;
float Height;
float Height_Last;
bool landing_flag = false;
unsigned char data_to_mobile[10] = {0};

int main(int argc,char **argv)
{
	ros::init(argc,argv,"landing_move_node");
	ros::NodeHandle nh;

	drone = new DJIDrone(nh);

	if(drone->request_sdk_permission_control())
		ROS_INFO("Get permission control Correct!");
	ROS_INFO("A");

	ros::Subscriber Height = nh.subscribe("/dji_sdk/local_position",1,&LocalPositionCallback);
	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&DeltaMsgCallback);
	ros::ServiceClient send_to_mobile_client = nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");
	ROS_INFO("D");
    while(ros::ok())
    {

      	if((alti_flag||delta_pos)&&landing_flag)
    	{
    		data_to_mobile[0] = 0x01;
    		data_to_mobile[1] = 0x04;
      		dji_sdk::SendDataToRemoteDevice::Request land_req;
    		memcpy(&land_req.data,data_to_mobile,10);
    		dji_sdk::SendDataToRemoteDevice::Response land_resp;
    		bool success = send_to_mobile_client.call(land_req,land_resp);
    		if(success)ROS_INFO("0104");
    		memset(data_to_mobile, 0, sizeof(data_to_mobile));
    		drone->landing();
    		drone->release_sdk_permission_control();
//    		ros::shutdown();
    		landing_flag =false;
    		data_to_mobile[0] = 0x01;
    		data_to_mobile[1] = 0x08;
      		dji_sdk::SendDataToRemoteDevice::Request land_end_req;
    		memcpy(&land_end_req.data,data_to_mobile,10);
    		dji_sdk::SendDataToRemoteDevice::Response land_end_resp;
    		bool land_success = send_to_mobile_client.call(land_end_req,land_end_resp);
    		if(success)ROS_INFO("0108");
    		memset(data_to_mobile, 0, sizeof(data_to_mobile));
    	}
        ros::spinOnce();

    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{

	if (new_location.state)
	{
		landing_flag = new_location.state;
		float Velocity_X = new_location.delta_X_meter;
		float Velocity_Y = new_location.delta_Y_meter;


		for(int i = 0;i < 20; i++)
		{
			drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
			usleep(20000);
		}

		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "position = (" << new_location.delta_X_meter << "," << new_location.delta_Y_meter << ")");

		if((new_location.delta_X_meter < 0.2) && (new_location.delta_Y_meter < 0.2)&&(Height < 1.9))
			delta_pos = true;
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
	if(landing_flag)
	{
		Height_Last = Height;
		Height = LocalPosition.z;
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "altitude = "  << LocalPosition.z);
		drone->request_sdk_permission_control();

		if(Height > 0.8)
		{
			for(int i = 0;i < 20; i++)
			{
				drone->attitude_control(0x40,0,0,-1,0);//水平速度
				usleep(20000);
			}
			ROS_INFO_STREAM("I changed the height!");
		}
		else
		{
			if(abs(Height_Last-Height) < 1)	alti_flag = true;
		}
	}
}


