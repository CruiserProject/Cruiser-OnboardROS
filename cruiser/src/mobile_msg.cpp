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
#include <dji_sdk/SendDataToRemoteDevice.h>
#include <cruiser/DeltaPosition.h>
#include <cstring>

void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData);
void Visual_Landing_Cmd(char* mobile_msg,ros::Publisher pub_landing_flag,ros::ServiceClient send_to_mobile_client);
void Object_Tracking_Cmd(char* mobile_msg,ros::Publisher pub_tracking_flag,ros::Publisher pub_tracking_position,ros::ServiceClient send_to_mobile_client);
void DeltaYCallback(const cruiser::DeltaPosition& new_location);
void float2char(float num, unsigned char& high, unsigned char& low);

float delta_x;
float delta_y;
char mobile_msg[10] = {0};
unsigned char data_to_mobile[10] = {0};
//unsigned char x_high,x_low,y_high,y_low;
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mobile_msg");
	ros::NodeHandle nh;

	ros::Subscriber MobileMsg = nh.subscribe("/dji_sdk/data_received_from_remote_device",1,&GetMobileMsgCallback);
	ros::Publisher pub_landing_flag = nh.advertise<cruiser::Flag>("cruiser/landing_flag",1);
	ros::Publisher pub_tracking_flag = nh.advertise<cruiser::Flag>("cruiser/tracking_flag",1);
	ros::Publisher pub_tracking_position = nh.advertise<cruiser::TrackingPosition>("cruiser/tracking_position",1);
	ros::ServiceClient send_to_mobile_client = nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");
	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&DeltaYCallback);

	while(ros::ok())
	{
		switch (mobile_msg[0])
		{
		case 0x01:
			Visual_Landing_Cmd(mobile_msg,pub_landing_flag,send_to_mobile_client);
			break;
		case 0x02:
			Object_Tracking_Cmd(mobile_msg,pub_tracking_flag,pub_tracking_position,send_to_mobile_client);
			break;
		default:break;
		}

		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x42;

		unsigned char  delta_x_high =  data_to_mobile[2];
		unsigned char  delta_x_low =  data_to_mobile[3];
		unsigned char  delta_y_high =  data_to_mobile[4];
		unsigned char  delta_y_low =  data_to_mobile[5];
		float2char(delta_x,delta_x_high,delta_x_low);
		float2char(delta_y,delta_y_high,delta_y_low);

		dji_sdk::SendDataToRemoteDevice::Request delta_pos_req;
		delta_pos_req.data.resize(10);
		memcpy(&delta_pos_req.data[0],data_to_mobile,10);
		dji_sdk::SendDataToRemoteDevice::Response delta_pos_resp;
		bool land_success = send_to_mobile_client.call(delta_pos_req,delta_pos_resp);
		memset(data_to_mobile, 0, sizeof(data_to_mobile));

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

void Visual_Landing_Cmd(char* mobile_msg,ros::Publisher pub_landing_flag,ros::ServiceClient send_to_mobile_client)
{
	switch(mobile_msg[1])
	{
	case 1:
	{
		cruiser::Flag landing_flag;
		landing_flag.flag = true;
		pub_landing_flag.publish(landing_flag);
		ROS_INFO("0101");
		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x02;
		dji_sdk::SendDataToRemoteDevice::Request req;
		req.data.resize(10);
		memcpy(&req.data[0],data_to_mobile,10);
		dji_sdk::SendDataToRemoteDevice::Response resp;
		bool success = send_to_mobile_client.call(req,resp);
		if(success)ROS_INFO("0102");
		memset(data_to_mobile, 0, sizeof(data_to_mobile));
		break;
	}
	case 0x03:
	{
		cruiser::Flag landing_flag;
		landing_flag.flag = false;
		pub_landing_flag.publish(landing_flag);
		ROS_INFO("0103");
		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x04;
		dji_sdk::SendDataToRemoteDevice::Request req;
		req.data.resize(10);
		memcpy(&req.data[0],data_to_mobile,10);
		dji_sdk::SendDataToRemoteDevice::Response resp;
		bool success = send_to_mobile_client.call(req,resp);
		if(success)ROS_INFO("0104");
		memset(data_to_mobile, 0, sizeof(data_to_mobile));
		break;
	}
	default:
		break;
	}
}

void Object_Tracking_Cmd(char* mobile_msg,ros::Publisher pub_tracking_flag,ros::Publisher pub_tracking_position,ros::ServiceClient send_to_mobile_client)
{
	switch(mobile_msg[1])
	{
	case 0x01:
	{
		cruiser::Flag tracking_flag;
		tracking_flag.flag = true;
		pub_tracking_flag.publish(tracking_flag);
		ROS_INFO("0201");
		data_to_mobile[0] = 0x02;
		data_to_mobile[1] = 0x02;
		dji_sdk::SendDataToRemoteDevice::Request req;
		req.data.resize(10);
		memcpy(&req.data[0],data_to_mobile,10);
		dji_sdk::SendDataToRemoteDevice::Response resp;
		bool success = send_to_mobile_client.call(req,resp);
		if(success)ROS_INFO("0202");
		memset(data_to_mobile, 0, sizeof(data_to_mobile));
		break;
	}
	case 0x03:
	{
		cruiser::Flag tracking_flag;
		tracking_flag.flag = false;
		pub_tracking_flag.publish(tracking_flag);
		ROS_INFO("0203");
		data_to_mobile[0] = 0x02;
		data_to_mobile[1] = 0x04;
		dji_sdk::SendDataToRemoteDevice::Request req;
		req.data.resize(10);
		memcpy(&req.data[0],data_to_mobile,10);
		dji_sdk::SendDataToRemoteDevice::Response resp;
		bool success = send_to_mobile_client.call(req,resp);
		if(success)ROS_INFO("0204");
		memset(data_to_mobile, 0, sizeof(data_to_mobile));
		break;
	}
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
		break;
	}
	default:
		break;
	}
}


void DeltaYCallback(const cruiser::DeltaPosition& new_location)
{
	if (new_location.state)
	{
		delta_x = new_location.delta_X_meter;
		delta_y = new_location.delta_Y_meter;
	}
}



void float2char(float num, unsigned char& high, unsigned char& low)
{
	high = (unsigned char)num;
	low = (unsigned char)(num*100 - high*100);
}

