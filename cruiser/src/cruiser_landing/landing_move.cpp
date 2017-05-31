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
	bool alti_flag = false;
	bool delta_pos = false;

public:
	float delta_x_pos = 0.0;
	float delta_y_pos = 0.0;
	DJIDrone *drone;
	CruiserDrone *cruiserdrone;

	LandingMove(ros::NodeHandle& nh)
	{
		ControlFlag = nh.subscribe("cruiser/landing_flag",1,&LandingMove::GetTaskControl,this);
		DeltaMsg = nh.subscribe("cruiser/landing_move",1,&LandingMove::DeltaMsgCallback,this);
		drone = new DJIDrone(nh);
		cruiserdrone = new CruiserDrone(nh);
	}

	~LandingMove()
	{
		delete drone;
		delete cruiserdrone;
	}

	void GetTaskControl(const cruiser::Flag &landing_flag)
	{
		if(landing_flag.flag)
		{
			if(this->drone->request_sdk_permission_control())
				ROS_INFO_STREAM("landing_move_node : Initialization and getting permission control.");
		}
		else
		{
			this->SetAltiFlag(false);
    		this->SetDeltaPos(false);
    		if(this->drone->release_sdk_permission_control())
				ROS_INFO_STREAM("landing_move_node : Releasing control and ending task.");
		}				
	}

	void DeltaMsgCallback(const cruiser::DeltaPosition& new_location)
	{
		if(this->drone->gimbal_angle_control(0, -900, 0, 10))
		{
			ROS_INFO_STREAM("landing_move_node : Gimbal angle changed.");
			usleep(10000);
		}
		delta_pos = new_location.state;

		if (new_location.state)
		{
			delta_y_pos = new_location.delta_X_meter;//Note:north east down ,so x,y exchanged.
			delta_x_pos = new_location.delta_Y_meter;
		}
		
		height_last = local_height;
		local_height = this->cruiserdrone->GetHeightNow();
		if((local_height < 0.8)&&(fabs(height_last - local_height) < 0.8))
			alti_flag = true;
	}
	
	void SetAltiFlag(bool flag)
	{
		alti_flag = flag;
	}
	void SetDeltaPos(bool flag)
	{
		delta_pos = flag;
	}
	bool GetAltiFlag()
	{
		return alti_flag;
	}
	bool GetDeltaPos()
	{
		return delta_pos;
	}
	float GetHight()
	{
		return local_height;
	}
};

int main(int argc,char **argv)
{
	ros::init(argc,argv,"landing_move_node");
	ros::NodeHandle nh;
	
	LandingMove *landing_move_node = new LandingMove(nh);
	int Kp = 1.5;

	ros::Rate rate(2);
    while(ros::ok())
    {
    	if(landing_move_node->GetDeltaPos())
    	{
    		landing_move_node->drone->attitude_control(0x8A,Kp * landing_move_node->delta_x_pos,Kp * landing_move_node->delta_y_pos,-0.5,0);
    		ROS_INFO_STREAM("landing_move_node : Drone moved.");
			//landing_move_node->drone->attitude_control(0x8A,0,0,-0.5,0);
			//ROS_INFO_STREAM("landing_move_node : altitude changed.");
		}
		//else
		//{
		//	landing_move_node->drone->attitude_control(0x8A,0,0,-0.5,0);
		//	ROS_INFO_STREAM("landing_move_node : Circles not found.");
		//}

      	if(landing_move_node->GetAltiFlag())
    	{
    	    landing_move_node->cruiserdrone->SendVtlLandingMsg();
			landing_move_node->drone->landing();
			landing_move_node->SetAltiFlag(false);
    		landing_move_node->SetDeltaPos(false);
    		if(landing_move_node->drone->release_sdk_permission_control())
				ROS_INFO_STREAM("landing_move_node : Releasing control and ending task.");
  			//landing_move_node->cruiserdrone->SendSucLandingMsg();
			//if(landing_move_node->drone->gimbal_angle_control(0, 0, 0, 10))
				//ROS_INFO_STREAM("landing_move_node : gimbal angle changed.");

    		
				//usleep(10000);
    	}
        ros::spinOnce();
        rate.sleep();
    }
}
