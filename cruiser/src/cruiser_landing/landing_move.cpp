#include <ros/ros.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_drone.h>


DJIDrone *drone;
CruiserDrone* cruiserdrone;

bool alti_flag = false;
bool delta_pos = false;
float Height;
float Height_Last;


void DeltaMsgCallback(const cruiser::DeltaPosition& new_location);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"landing_move_node");
	ros::NodeHandle nh;

	cruiserdrone = new CruiserDrone(nh);
	drone = new DJIDrone(nh);

	ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&DeltaMsgCallback);

	ros::Rate rate(0.5);

	if(drone->request_sdk_permission_control())
		ROS_INFO_STREAM("landing_move_node : initialization and get control.");

    while(ros::ok())
    {
      	if(alti_flag||delta_pos)
    	{
      		cruiserdrone->SendVtlLandingMsg();
    		drone->landing();
    		cruiserdrone->SendSucLandingMsg();
    		alti_flag = false;
    		delta_pos = false;
    		if(drone->gimbal_angle_control(0, 0, 0, 10))
    			ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
    		usleep(100000);
    	}
        rate.sleep();
        ros::spinOnce();
    }
}

void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
{
	float Height = 0;
	if(drone->gimbal_angle_control(0, -900, 0, 10))
		ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
	usleep(100000);
	if (new_location.state)
	{
		drone->attitude_control(0x81,new_location.delta_X_meter,new_location.delta_Y_meter,0,0);//location
		usleep(20000);
		Height = cruiserdrone->GetHeightNow();
		if(Height > 1)
		{
			drone->attitude_control(0x81,0,0,0.2,0);//location
			usleep(20000);
		}
		else
		{
			if(abs(Height_Last-Height) < 1)	alti_flag = true;
		}

		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "landing_move_node : move position = (" << new_location.delta_X_meter << ","
				<< new_location.delta_Y_meter << ")");

		if((new_location.delta_X_meter < 0.2) && (new_location.delta_Y_meter < 0.2)&&(Height < 1.9))
			delta_pos = true;
	}
//	float Height = 0;
//	if(drone->gimbal_angle_control(0, -900, 0, 10))
//		ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
//	usleep(100000);
//	if (new_location.state)
//	{
//		float Velocity_X = new_location.delta_X_meter;
//		float Velocity_Y = new_location.delta_Y_meter;
//
//		for(int i = 0;i < 50; i++)
//		{
//			drone->attitude_control(0x40,Velocity_X,Velocity_Y,0,0);//水平速度
//			usleep(20000);
//		}
//		Height = cruiserdrone->GetHeightNow();
//		if(Height > 1)
//		{
//			for(int i = 0;i < 20; i++)
//			{
//				drone->attitude_control(0x40,0,0,-1,0);//水平速度
//				usleep(20000);
//			}
//		}
//		else
//		{
//			if(abs(Height_Last-Height) < 1)	alti_flag = true;
//		}
//
//		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
//				<< "landing_move_node : move position = (" << new_location.delta_X_meter << ","
//				<< new_location.delta_Y_meter << ")");
//
//		if((new_location.delta_X_meter < 0.2) && (new_location.delta_Y_meter < 0.2)&&(Height < 1.9))
//			delta_pos = true;
//	}

}


