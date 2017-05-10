/*
 * mobile_msg.cpp
 *
 *  Created on: May 10, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <dji_sdk/TransparentTransmissionData.h>
#include <cruiser/Flag.h>
#include <cruiser/DeltaPosition.h>
#include <cruiser/TrackingPosition.h>

void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData);
void Visual_Landing_Cmd(char* mobile_msg,ros::Publisher pub_landing_flag);
void Object_Tracking_Cmd(char* mobile_msg,ros::Publisher pub_tracking_flag,ros::Publisher pub_tracking_position);

char mobile_msg[10] = {0};

int main(int argc,char **argv)
{
	ros::init(argc,argv,"mobile_msg");
	ros::NodeHandle nh;

	ros::Subscriber MobileMsg = nh.subscribe("/dji_sdk/data_received_from_remote_device",1,&GetMobileMsgCallback);
	ros::Publisher pub_landing_flag = nh.advertise<cruiser::Flag>("cruiser/landing_flag",1);
	ros::Publisher pub_tracking_flag = nh.advertise<cruiser::Flag>("cruiser/tracking_flag",1);
	ros::Publisher pub_tracking_position = nh.advertise<cruiser::TrackingPosition>("cruiser/tracking_position",1);
	while(ros::ok())
	{
		switch (mobile_msg[0])
		{
		case 0x01:
			Visual_Landing_Cmd(mobile_msg,pub_landing_flag);
			break;
		case 0x02:
			Object_Tracking_Cmd(mobile_msg,pub_tracking_flag,pub_tracking_position);
			break;
		default:break;
		}
		memset(mobile_msg, 0, sizeof(mobile_msg));
		ros::spinOnce();
	}

}

void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData)
{
	int i=0;
	for(i = 0;i < 10;i++)
	{
		mobile_msg[i] = mobileData.data[i];
	}
}

void Visual_Landing_Cmd(char* mobile_msg,ros::Publisher pub_landing_flag)
{
	switch(mobile_msg[1])
	{
	case 0x01:
	{
		cruiser::Flag landing_flag;
		landing_flag.flag = true;
		pub_landing_flag.publish(landing_flag);
		ROS_INFO("0101");
	}
	break;
	case 0x03:
	{
		cruiser::Flag landing_flag;
		landing_flag.flag = false;
		pub_landing_flag.publish(landing_flag);
		ROS_INFO("0103");
	}
	break;
	default:break;
	}
}

void Object_Tracking_Cmd(char* mobile_msg,ros::Publisher pub_tracking_flag,ros::Publisher pub_tracking_position)
{
	switch(mobile_msg[1])
	{
	case 0x01:
	{
		cruiser::Flag tracking_flag;
		tracking_flag.flag = true;
		pub_tracking_flag.publish(tracking_flag);
		ROS_INFO("0201");
	}
	break;
	case 0x03:
	{
		cruiser::Flag tracking_flag;
		tracking_flag.flag = false;
		pub_tracking_flag.publish(tracking_flag);
		ROS_INFO("0203");
	}
	break;
	case 0x11:
	{
		cruiser::TrackingPosition TrackingPosition;
		TrackingPosition.a_width_percent = float(mobile_msg[2])/100;
		TrackingPosition.a_height_percent = float(mobile_msg[3])/100;
		TrackingPosition.b_width_percent = float(mobile_msg[4])/100;
		TrackingPosition.b_height_percent = float(mobile_msg[5])/100;
		pub_tracking_position.publish(TrackingPosition);
		ROS_INFO_STREAM("a:( "<<TrackingPosition.a_width_percent<<" , "<<TrackingPosition.a_height_percent
				<<" ) b:( "<<TrackingPosition.b_width_percent<<" , "<<TrackingPosition.b_height_percent);
	}
	break;
	default:break;
	}
}
