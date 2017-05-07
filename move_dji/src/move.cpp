#include <ros/ros.h>
#include <stdio.h>
#include <stdint.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_dji/location.h>
#include "dji_sdk_lib/DJI_API.h"
#include "dji_sdk_lib/DJICommonType.h"
#include <dji_sdk/dji_sdk_node.h>
#include <dji_sdk/TransparentTransmissionData.h>

using namespace DJI;
using namespace DJI::onboardSDK;

void LocationMessageCallback(const move_dji::location& new_location);
void global_position_subscriber_callback(dji_sdk::GlobalPosition global_position);
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
void Get_mobile_callback(const dji_sdk::TransparentTransmissionData &transparent_transmission_data);

DJIDrone* drone;
DJI::onboardSDK::ROSAdapter *rosAdapter;

long int count = 0;
int alti_flag = 0;
int delta_pos = 0;
float Height;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"move_dji");
	ros::NodeHandle nh;
	drone = new DJIDrone(nh);

	int i = 0;

	rosAdapter = new DJI::onboardSDK::ROSAdapter;

    if(!drone->request_sdk_permission_control())
    	ROS_INFO("permission_control_error!!");
    if(!drone->takeoff())
    	ROS_INFO("takeoff_error!!");

    for(i = 0;i < 20;i++)
    {
		drone->attitude_control(0x40,0,0,3,0);
		usleep(20000);
    }


    ros::Subscriber sub = nh.subscribe("location",1000,&LocationMessageCallback);

	uint8_t  testdata = 97;
	uint8_t len = sizeof(testdata);
    rosAdapter->init("/dev/ttyUSB0",230400);
    rosAdapter->sendToMobile(&testdata,len);

//    sleep(1);
//	ROS_INFO("move_dji_test");
//    DJI::UserData testuserdata = NULL;
//	ROS_INFO("move_dji_test2");
//	sleep(3);
//    ROS_INFO("move_dji_test3");
//    sleep(3);
//    rosAdapter->init("/dev/ttyTHS1",230400);
//    ROS_INFO("move_dji_test4");
//    sleep(3);
//    ROS_INFO("move_dji_test5");
//    sleep(3);
//    ROS_INFO("move_dji_test6");
//    Trial->sendToMobile(&testdata,len);

    ros::Subscriber sub2 = nh.subscribe("/dji_sdk/global_position",1000,&global_position_subscriber_callback);
    ros::Subscriber sub3 = nh.subscribe("/dji_sdk/TransparentTransmissionData",1,&Get_mobile_callback);
//    ros::Rate ratems(0.3);
    while(ros::ok())
    {
	    rosAdapter->sendToMobile(&testdata,len);

      	if(alti_flag||delta_pos)
    	{
    		drone->landing();
    		drone->release_sdk_permission_control();
    	}
//	    ratems.sleep();
        ros::spinOnce();
    }
}


void LocationMessageCallback(const move_dji::location& new_location)
{
	if (new_location.flag)
	{
		int utime = 1000000;	//1s
		float Velocity_X = new_location.delta_X;
		float Velocity_Y = new_location.delta_Y;
		drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
		usleep(utime);
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "position = (" << new_location.delta_X << "," << new_location.delta_Y << ")");

	}
	if((new_location.delta_X < 0.2) && (new_location.delta_Y < 0.2)&&(Height < 1.5))
		delta_pos = 1;

//
//	float Velociy_X_max = new_location.delta_X * 2;
//	float Velociy_Y_max = new_location.delta_Y * 2;
//	float deltaV = 0.2;	//手册最小精度
//	int Nx = Velociy_X_max / deltaV * 2;	//量化
//	int Ny = Velociy_Y_max / deltaV * 2;	//量化
//	int countN = 0;
//	int deltaTx = utime / Nx;		//步进时间
//	int deltaTy = utime / Ny;		//步进时间
//	float Velocity_X = 0;
//	float Velocity_Y = 0;
//
//	int N;
//	if(Nx > Ny)N=Nx;else N=Ny;
//	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
//			<< "position = (" << new_location.delta_X << "," << new_location.delta_Y << ")");
//
//	for(countN = 0 ; countN < N; countN++)
//	{
//		if(countN <= (Nx/2))
//			Velocity_X += deltaVx;
//		else
//			Velocity_X -= deltaVx;
//		if(countN <= (Ny/2))
//			Velocity_Y += deltaVy;
//		else
//			Velocity_Y -= deltaVy;
//		drone->attitude_control(0x40,Velocity_X,0,0,0);//水平速度
//		usleep(deltaTx);
//		ROS_INFO_STREAM("X is done!");
//		drone->attitude_control(0x40,0,Velocity_Y,0,0);//水平速度
//		usleep(deltaTy);
//		ROS_INFO_STREAM("Y is done!");
//	}


}

void global_position_subscriber_callback( dji_sdk::GlobalPosition global_position)
{
	long int during = 500000;
//
//
	long int & countNN = count;
//
	countNN++;
	Height = global_position.altitude;
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "altitude = "  << global_position.altitude);
	if(countNN > 200)
	{
		if(global_position.altitude > 0.8)
		{
			alti_flag = 0;
			drone->attitude_control(0x40,0,0,-1,0);//水平速度
			usleep(during);
			countNN = 0;
			ROS_INFO_STREAM("I changed the altitude!");
		}
		else alti_flag = 1;

	}
}

void Get_mobile_callback(const dji_sdk::TransparentTransmissionData &transparent_transmission_data)
{
	int len = sizeof(transparent_transmission_data.data);
	char i;
	for(i = 0;i < len;i++)
	{
		ROS_INFO("%c",transparent_transmission_data.data[i]);
	}
	ROS_INFO("\n");
	ROS_INFO_STREAM("I heard it!!");
}
//
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
