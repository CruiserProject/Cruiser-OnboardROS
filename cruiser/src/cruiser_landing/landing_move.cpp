#include <ros/ros.h>
#include <cruiser/CruiserHeader.h>
#include <cruiser/CruiserDrone.h>
#include <dji_sdk/dji_drone.h>


class LandingMove
{

private:
	ros::Subscriber DeltaMsg;
	ros::Subscriber ControlFlag;
	float local_height = 0.0;
	float height_last = 0.0;

public:

	bool alti_flag = false;
	bool delta_pos = false;//changed the meaning
	float delta_x_pos = 0;
	float delta_y_pos = 0;
	DJIDrone *drone;
	CruiserDrone *cruiserdrone;

	LandingMove(ros::NodeHandle& nh)
	{
		ControlFlag = nh.subscribe("cruiser/landing_flag",1,&LandingMove::GetDroneControl,this);
		DeltaMsg = nh.subscribe("cruiser/landing_move",1,&LandingMove::DeltaMsgCallback,this);
		drone = new DJIDrone(nh);
		cruiserdrone = new CruiserDrone(nh);
	}

	~LandingMove()
	{
		delete drone;
		delete cruiserdrone;
	}

	void GetDroneControl(const cruiser::Flag &landing_flag)
	{
		if(landing_flag.flag)
			if(this->drone->request_sdk_permission_control())
				ROS_INFO_STREAM("landing_move_node : initialization and get control.");
	}

	void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
	{
		if(this->drone->gimbal_angle_control(0, -900, 0, 10))
		{
			ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
			usleep(10000);
		}
		delta_pos = new_location.state;

		if (new_location.state)
		{
			this->delta_y_pos = new_location.delta_X_meter;//Note:north east down ,so x,y exchanged.
			this->delta_x_pos = new_location.delta_Y_meter;
		}
		
		this->height_last = this->local_height;
		this->local_height = this->cruiserdrone->GetHeightNow();
		if((this->local_height < 1)&&(fabs(this->height_last - this->local_height) < 1))
			this->alti_flag = true;
//		{
//			this->drone->attitude_control(0x82,0,0,-0.2,0);//location
//			usleep(500000);
//		}
//		else
//		{
//			if(fabs(this->height_last - this->local_height) < 1)	this->alti_flag = true;
//		}
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
	
	LandingMove *landing_move_node = new LandingMove(nh);

	ros::Rate rate(10);
    while(ros::ok())
    {
    	if(landing_move_node->delta_pos)
    	{
    		landing_move_node->drone->attitude_control(0x80,landing_move_node->delta_x_pos,landing_move_node->delta_y_pos,0,0);//location
    		ROS_INFO_STREAM("drone moved.");
    	
    		landing_move_node->drone->attitude_control(0x10,0,0,-0.2,0);//location
			ROS_INFO_STREAM("altitude changed.");
		}

      	if(landing_move_node->alti_flag)
    	{
      		landing_move_node->cruiserdrone->SendVtlLandingMsg();
    		landing_move_node->drone->landing();
    		landing_move_node->cruiserdrone->SendSucLandingMsg();
    		landing_move_node->SetAltiFlag(false);
    		landing_move_node->SetDeltaPos(false);
    	//	if(landing_move_node->drone->gimbal_angle_control(0, 0, 0, 10))
    //			ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");
  //  		usleep(10000);
    		if(landing_move_node->drone->release_sdk_permission_control())
    			ROS_INFO_STREAM("landing_move_node : release and end control.");
    	}
        ros::spinOnce();
        rate.sleep();
    }
}
