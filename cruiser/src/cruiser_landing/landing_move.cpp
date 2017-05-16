#include <ros/ros.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_drone.h>


DJIDrone *drone;
CruiserDrone* cruiserdrone;

class LandingMove
{
private:
	ros::Subscriber DeltaMsg;
	ros::Subscriber Height;
	bool height_change_flag = false;
	float local_height = 0.0;
	float height_last = 0.0;

public:
	LandingMove(ros::NodeHandle& nh)
	{
		ros::Subscriber DeltaMsg = nh.subscribe("cruiser/landing_move",1,&LandingMove::DeltaMsgCallback,this);
		ros::Subscriber Height = nh.subscribe("/dji_sdk/local_position",1,&LandingMove::AutoHeightChanged,this);
	}

	~LandingMove()
	{
	//	cv::destroyWindow(OPENCV_WINDOW);
	}
	bool alti_flag = false;
	bool delta_pos = false;

	void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
	{
		this->height_change_flag = new_location.state;
		if(drone->gimbal_angle_control(0, -900, 0, 10))
		{
			ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
			usleep(100000);
		}

		if (new_location.state)
		{
			drone->attitude_control(0x81,new_location.delta_X_meter,new_location.delta_Y_meter,0,0);//location
			usleep(20000);
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

	void AutoHeightChanged(const dji_sdk::LocalPosition& new_height)
	{
		this->height_last = this->local_height;
		this->local_height = cruiserdrone->GetHeightNow();
		if(this->local_height > 1)
		{
			drone->attitude_control(0x81,0,0,0.2,0);//location
			usleep(30000);
		}
		else
		{
			if(abs(this->height_last - this->local_height) < 1)	this->alti_flag = true;
		}
	}

	void SetAltiFlag(bool flag)
	{
		this->alti_flag = flag;
	}
	void SetDeltaPos(bool flag)
	{
		this->delta_pos = flag;
	}

};






int main(int argc,char **argv)
{
	ros::init(argc,argv,"landing_move_node");
	ros::NodeHandle nh;
	if(drone->request_sdk_permission_control())
		ROS_INFO_STREAM("landing_move_node : initialization and get control.");

	ros::Rate rate(0.5);
	LandingMove* landing_move_node = new LandingMove(nh);
	drone = new DJIDrone(nh);
	cruiserdrone = new CruiserDrone(nh);
    while(ros::ok())
    {
      	if(landing_move_node->alti_flag||landing_move_node->delta_pos)
    	{
      		cruiserdrone->SendVtlLandingMsg();
    		drone->landing();
    		cruiserdrone->SendSucLandingMsg();
    		landing_move_node->SetAltiFlag(false);
    		landing_move_node->SetDeltaPos(false);
    		if(drone->gimbal_angle_control(0, 0, 0, 10))
    			ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
    		usleep(100000);
    	}
        rate.sleep();
        ros::spinOnce();
    }
}
