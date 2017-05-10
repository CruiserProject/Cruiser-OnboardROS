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

void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData);


int main(int argc,char **argv)
{
	ros::init(argc,argv,"mobile_msg");
	ros::NodeHandle nh;

	ros::Subscriber MobileMsg = nh.subscribe("/dji_sdk/data_received_from_remote_device",1,&GetMobileMsgCallback);


}

void GetMobileMsgCallback(const dji_sdk::TransparentTransmissionData& mobileData)
{
	int i=0;
	while(mobileData.data[i]!= 0 )
	{
		ROS_INFO_STREAM(mobileData.data[i++]);
	}

}
