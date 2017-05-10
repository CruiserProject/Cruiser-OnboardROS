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
//#include <dji_sdk/TransparentTransmissionData.h>
#include <dji_sdk/LocalPosition.h>
#include <cruiser/DeltaPosition.h>
#include <cruiser/Flag.h>

using namespace DJI;
using namespace DJI::onboardSDK;


void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);


void LocalPositionCallback(const dji_sdk::LocalPosition& LocalPosition);

void MissionFlagCallback(const cruiser::Flag& landingFlag);
void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);
DJIDrone *drone;
DJI::onboardSDK::ROSAdapter *rosAdapter;

long int count = 0;
int alti_flag = 0;
int delta_pos = 0;
float Height;
bool Mission_flag = 0;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"landing_move_node");
	ros::NodeHandle nh;

    ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_flag",1,&DeltaMsgCallback);

    if(Mission_flag)
    {
    	drone = new DJIDrone(nh);
    	rosAdapter = new DJI::onboardSDK::ROSAdapter;

    	if(!drone->request_sdk_permission_control())
    		ROS_INFO("Get permission control Error!");
    	ROS_INFO("A");

    	/*Test part*/
    	if(!drone->takeoff())
    		ROS_INFO("Take off Error!");
    	ROS_INFO("B");

    	for(int i = 0;i < 40; i++)
    	{
    		drone->attitude_control(0x40,0,0,3,0);
    		usleep(20000);
    		ROS_INFO("C");
    	}

    	/*Test part end*/

    	uint8_t testdata = 97;
    	uint8_t len = sizeof(testdata);
    	rosAdapter->init("/dev/ttyUSB0",230400);//can not locate it before request_permission_control
    	rosAdapter->sendToMobile(&testdata,len);

    	ROS_INFO("D");

    	ros::Subscriber Height = nh.subscribe("/dji_sdk/local_position",1,&LocalPositionCallback);
        ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&DeltaMsgCallback);
    }


    ros::Rate rate(10);

    while(ros::ok())
    {
      	if((alti_flag||delta_pos)&&Mission_flag)
    	{
    		drone->landing();
    		drone->release_sdk_permission_control();
    		ros::shutdown();
    	}
        ros::spinOnce();
        rate.sleep();
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

	}
	if((new_location.delta_X < 0.2) && (new_location.delta_Y < 0.2)&&(Height < 1.9))
		delta_pos = 1;

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
	Height = LocalPosition.z;
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "altitude = "  << Height);
	if(Height > 0.8)
	{
		alti_flag = 0;
		for(int i = 0;i < 10; i++)
		{
			drone->attitude_control(0x40,0,0,-1,0);//水平速度
			usleep(20000);
		}
		ROS_INFO_STREAM("I changed the height!");
	}
	else alti_flag = 1;
}




void MissionFlagCallback(const cruiser::Flag& landingFlag)
{
	Mission_flag = landingFlag.flag;
}








//! Callback functions for Mobile Commands
void ObtainControlMobileCallback(DJIDrone *drone)
{
  drone->request_sdk_permission_control();
}

void ReleaseControlMobileCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
}

void TakeOffMobileCallback(DJIDrone *drone)
{
  drone->takeoff();
}

void LandingMobileCallback(DJIDrone *drone)
{
  drone->landing();
}

void GetSDKVersionMobileCallback(DJIDrone *drone)
{
  drone->check_version();
}

void ArmMobileCallback(DJIDrone *drone)
{
  drone->drone_arm();
}

void DisarmMobileCallback(DJIDrone *drone)
{
  drone->drone_disarm();
}

void GoHomeMobileCallback(DJIDrone *drone)
{
  drone->gohome();
}

void TakePhotoMobileCallback(DJIDrone *drone)
{
  drone->take_picture();
}

void StartVideoMobileCallback(DJIDrone *drone)
{
  drone->start_video();
}

void StopVideoMobileCallback(DJIDrone *drone)
{
  drone->stop_video();
}
