/*
 * mobile_msg.cpp
 *
 *  Created on: May 10, 2017
 *      Author: cj
 */
#include <ros/ros.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_sdk.h>
#include <cstring>



void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData);
void Visual_Landing_Cmd(unsigned char* mobile_msg,ros::Publisher pub_landing_flag,CruiserDrone* cruiser);
void Object_Tracking_Cmd(unsigned char* mobile_msg,ros::Publisher pub_tracking_flag,ros::Publisher pub_tracking_position,CruiserDrone* cruiser);

unsigned char mobile_msg[10] = {0};

int main(int argc,char **argv)
{
	ros::init(argc,argv,"mobile_msg");
	ros::NodeHandle nh;

    CruiserDrone* cruiser = new CruiserDrone(nh);

	ros::Subscriber MobileMsg = nh.subscribe("/dji_sdk/data_received_from_remote_device",1,&GetMobileMsgCallback);

	ros::Publisher pub_landing_flag = nh.advertise<cruiser::Flag>("cruiser/landing_flag",1);
	ros::Publisher pub_tracking_flag = nh.advertise<cruiser::Flag>("cruiser/tracking_flag",1);
	ros::Publisher pub_tracking_position = nh.advertise<cruiser::TrackingPosition>("cruiser/tracking_position",1);

	ros::Rate communicate_rate(10);

	while(ros::ok())
	{
		switch (mobile_msg[0])
		{
		case 0x01:
			Visual_Landing_Cmd(mobile_msg,pub_landing_flag,cruiser);

			break;
		case 0x02:
			Object_Tracking_Cmd(mobile_msg,pub_tracking_flag,pub_tracking_position,cruiser);
			break;
		default:
			break;
		}
		memset(mobile_msg, 0, sizeof(mobile_msg));
		ros::spinOnce();
		communicate_rate.sleep();
	}
}

void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData)
{
	for(int i=0;i<10;i++)
	{
		mobile_msg[i] = mobileData.data[i];
	}
}


void Visual_Landing_Cmd(unsigned char* mobile_msg,ros::Publisher pub_landing_flag,CruiserDrone* cruiser)
{
	unsigned char data_to_mobile[10] = {0};
	//CDT SET 0x01
	switch(mobile_msg[1])
	{
	case 0x01:
	{
		//publish to topic
		cruiser::Flag landing_flag;
		landing_flag.flag = true;
		pub_landing_flag.publish(landing_flag);
		ROS_INFO_STREAM("0101");

		//ACK to mobile
		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x02;
		cruiser->SendMyDataToMobile(data_to_mobile);
		break;
	}
	case 0x03:
	{
		//publish to topic
		cruiser::Flag landing_flag;
		landing_flag.flag = false;
		pub_landing_flag.publish(landing_flag);
		ROS_INFO_STREAM("0103");

		//ACK to mobile
		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x04;
		cruiser->SendMyDataToMobile(data_to_mobile);
		break;
	}
	default:
		break;
	}
}

void Object_Tracking_Cmd(unsigned char* mobile_msg,ros::Publisher pub_tracking_flag,ros::Publisher pub_tracking_position,CruiserDrone* cruiser)
{
	unsigned char data_to_mobile[10] = {0};
	//CDT SET 0x02
	switch(mobile_msg[1])
	{
	case 0x01:
	{
		//publish to topic
		cruiser::Flag tracking_flag;
		tracking_flag.flag = true;
		pub_tracking_flag.publish(tracking_flag);
		ROS_INFO_STREAM("0201");

		//ACK to mobile
		data_to_mobile[0] = 0x02;
		data_to_mobile[1] = 0x02;
		cruiser->SendMyDataToMobile(data_to_mobile);
		break;
	}
	case 0x03:
	{
		//publish to topic
		cruiser::Flag tracking_flag;
		tracking_flag.flag = false;
		pub_tracking_flag.publish(tracking_flag);
		ROS_INFO_STREAM("0203");

		//ACK to mobile
		data_to_mobile[0] = 0x02;
		data_to_mobile[1] = 0x04;
		cruiser->SendMyDataToMobile(data_to_mobile);
		break;
	}
	case 0x11:
	{
		//publish to topic
		cruiser::TrackingPosition TrackingPosition;
		TrackingPosition.a_width_percent = float(mobile_msg[2])/100;
		TrackingPosition.a_height_percent = float(mobile_msg[3])/100;
		TrackingPosition.b_width_percent = float(mobile_msg[4])/100;
		TrackingPosition.b_height_percent = float(mobile_msg[5])/100;
		pub_tracking_position.publish(TrackingPosition);
		ROS_INFO_STREAM("0211 a:("<<TrackingPosition.a_width_percent<<","<<TrackingPosition.a_height_percent
				<<") b:("<<TrackingPosition.b_width_percent<<","<<TrackingPosition.b_height_percent<<")");

		//ACK to mobile
		data_to_mobile[0] = 0x02;
		data_to_mobile[1] = 0x12;
		cruiser->SendMyDataToMobile(data_to_mobile);
		break;
	}
	default:
		break;
	}

}


