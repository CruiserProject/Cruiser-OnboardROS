/*
 * CruiserDrone.h
 *
 *  Created on: May 12, 2017
 *      Author: cj
 */

#ifndef CRUISER_ONBOARDROS_CRUISER_INCLUDE_CRUISERDRONE_H_
#define CRUISER_ONBOARDROS_CRUISER_INCLUDE_CRUISERDRONE_H_

#include <ros/ros.h>
#include <cruiser/CruiserHeader.h>

class CruiserDrone
{
private:
	ros::Subscriber DeltaMsgLanding;
	ros::Subscriber DeltaMsgTracking;
	ros::Subscriber PositionNow;

	ros::ServiceClient send_to_mobile_client;

public:
	CruiserDrone(ros::NodeHandle& nh)
	{
		DeltaMsgLanding = nh.subscribe("cruiser/landing_move",1,&CruiserDrone::DeltaXYLandingCallback, this);
		DeltaMsgTracking = nh.subscribe("cruiser/tracking_move",1,&CruiserDrone::DeltaXYTrackingCallback, this);


		PositionNow = nh.subscribe("cruiser/tracking_position_now",1,&CruiserDrone::PositionNowCallback, this);
		send_to_mobile_client = nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");
	}

	void DeltaXYLandingCallback(cruiser::DeltaPosition Delta)
	{
		if (Delta.state)
		{
			unsigned char data_to_mobile[10] = {0};
			data_to_mobile[0] = 0x01;
			data_to_mobile[1] = 0x42;
			this->float2char(Delta.delta_X_meter,data_to_mobile[2],data_to_mobile[3]);
			this->float2char(Delta.delta_Y_meter,data_to_mobile[4],data_to_mobile[5]);
			SendMyDataToMobile(data_to_mobile);
		}
	}

	void DeltaXYTrackingCallback(cruiser::DeltaPosition Delta)
	{
		if (Delta.state)
		{
			unsigned char data_to_mobile[10] = {0};
			data_to_mobile[0] = 0x01;
			data_to_mobile[1] = 0x42;
			this->float2char(Delta.delta_X_meter,data_to_mobile[2],data_to_mobile[3]);
			this->float2char(Delta.delta_Y_meter,data_to_mobile[4],data_to_mobile[5]);
			SendMyDataToMobile(data_to_mobile);
		}
	}

	void PositionNowCallback(cruiser::TrackingPosition position)
	{
		unsigned char data_to_mobile[10] = {0};
		data_to_mobile[0] = 0x02;
		data_to_mobile[1] = 0x44;
		data_to_mobile[2] = (unsigned char)(position.a_width_percent*100);
		data_to_mobile[3] = (unsigned char)(position.a_height_percent*100);
		data_to_mobile[4] = (unsigned char)(position.b_width_percent*100);
		data_to_mobile[5] = (unsigned char)(position.b_height_percent*100);
		SendMyDataToMobile(data_to_mobile);
	}

	void SendVtlLandingMsg()
	{
		unsigned char data_to_mobile[10] = {0};
		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x06;
		SendMyDataToMobile(data_to_mobile);
	}

	void SendSucLandingMsg()
	{
		unsigned char data_to_mobile[10] = {0};
		data_to_mobile[0] = 0x01;
		data_to_mobile[1] = 0x08;
		SendMyDataToMobile(data_to_mobile);
	}

	void SendMyDataToMobile(unsigned char* data_to_mobile)
	{
		dji_sdk::SendDataToRemoteDevice::Request req;
		req.data.resize(10);
		memcpy(&req.data[0],data_to_mobile,10);
		dji_sdk::SendDataToRemoteDevice::Response resp;
		if(send_to_mobile_client.call(req,resp))
			ROS_INFO_STREAM("send "<<std::hex<<(int)data_to_mobile[0]<<(int)data_to_mobile[1]);
	}

	void float2char(float num, unsigned char& high, unsigned char& low)
	{
		high = (unsigned char)num;
		low = (unsigned char)(num*100 - high*100);
	}
};


#endif /* CRUISER_ONBOARDROS_CRUISER_INCLUDE_CRUISERDRONE_H_ */
