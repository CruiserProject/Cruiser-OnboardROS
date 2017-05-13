#include <ros/ros.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdlib>
#include <stdlib.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>

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

	CruiserDrone* cruiser = new CruiserDrone(nh);
	drone = new DJIDrone(nh);

	if(drone->request_sdk_permission_control())
		ROS_INFO("Get permission control Correct!");
	ROS_INFO("A");

	ros::Subscriber Height = nh.subscribe("/dji_sdk/local_position",1,&LocalPositionCallback);
	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&DeltaMsgCallback);
	ros::ServiceClient send_to_mobile_client = nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");
	ROS_INFO("D");
	ros::Rate land_rate(1);
    while(ros::ok())
    {
    	if(landing_flag)drone->gimbal_angle_control(0, -1800, 0, 20);
      	if(alti_flag||delta_pos)
    	{
    		data_to_mobile[0] = 0x01;
    		data_to_mobile[1] = 0x06;
    		cruiser->SendMyDataToMobile(data_to_mobile);
    		drone->landing();
    		drone->release_sdk_permission_control();

    		data_to_mobile[0] = 0x01;
    		data_to_mobile[1] = 0x08;
    		cruiser->SendMyDataToMobile(data_to_mobile);
    		memset(data_to_mobile, 0, sizeof(data_to_mobile));
    		drone->gimbal_angle_control(0, -300, 0, 20);
    	}
        ros::spinOnce();
        land_rate.sleep();

    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{
	landing_flag = true;
	if (new_location.state)
	{
		float Velocity_X = new_location.delta_X_meter;
		float Velocity_Y = new_location.delta_Y_meter;


		for(int i = 0;i < 5; i++)
		{
			drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
			usleep(20000);
			drone->attitude_control(0x40,0,0,0,0);//水平速度
			usleep(60000);
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
	if(delta_pos)
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


