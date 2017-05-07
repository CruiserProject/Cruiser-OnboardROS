#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_dji/location.h>


void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void LocationMessageCallback(const move_dji::location& new_location);

int main(int argc,char **argv)
{
//	int i = 0;
	using namespace DJI::onboardSDK;

	ros::init(argc,argv,"move_dji");
	ros::NodeHandle nh;
	ROS_INFO("move_dji_test");
    ros::Subscriber sub = nh.subscribe("location",1000,&LocationMessageCallback);

	DJIDrone* drone = new DJIDrone(nh);

    drone->request_sdk_permission_control();
    sleep(5);


    drone->takeoff();
    sleep(6);



//    for (i = 0;i < 1000;i++)
//    {
//    	if(i < 500)
//    	    drone->attitude_control(0x40,2,0,0,0);//水平距离
//    	else
//    	    drone->attitude_control(0x40,0,2,0,0);//水平距离
//      usleep(2000);
//    }
//
//    drone->landing();
//    sleep(5);
//
//    ROS_INFO("move_dji_End");
//    drone->release_sdk_permission_control();
//    sleep(1);

//    drone->global_position_subscriber = nh.subscribe("",1000,&local_position_subscriber_callback);
    while(ros::ok())
    {

        ros::spinOnce();
    }
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

void LocationMessageCallback(const move_dji::location& new_location)
{
//	double time = 0.5 * ((double)1/RATE);
	int utime = 1000000;

	using namespace move_dji;

	float Velocity_X = new_location.delta_X * 1;
	float Velocity_Y = new_location.delta_Y * 1;

	using namespace DJI::onboardSDK;

	ros::NodeHandle nh;
	DJIDrone* drone = new DJIDrone(nh);

	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "position = (" << new_location.delta_X << "," << new_location.delta_Y << ")");

	drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
	usleep(utime);
}
