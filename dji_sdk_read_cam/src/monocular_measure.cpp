#include <ros/ros.h>
#include "test1/Location.h"
#include "math.h"

//以下参数必须保留
//机传感器尺寸参数
const float sensor_weight=6.17;
const float sensor_height=4.55;

//相机等效焦距
const float focal_length=20;

//相机视角
const float optic_angle=94;

//相机与地面的夹角　
const float degree=0;

//可以根据代码组织需求更改该函数的组织形式，当前函数输入参数以消息的形式组织，消息中的内容为：目标的像素坐标（x,y）,飞机当前飞行高度height
//目标追踪坐标转换函数
void track_coord_cal(const test1::Location msg)
{
	float real_x=0;
	float real_y=0;

	//图像坐标原点转换为像素坐标
	float v0=sensor_weight/2;
	float u0=sensor_height/2;

	//计算fx.fy
	float fx=focal_length/(sensor_weight/1080);
	float fy=focal_length/(sensor_height/720);

	//依据单目测距数学模型进行坐标转换
	real_x=(msg.x-u0)*msg.height/(sqrt(fx*fx+(msg.y-v0)*sin(degree-atan((msg.y-v0)/fy))));
	real_y=msg.height/tan(degree-atan((msg.y-v0)/fy));

	ROS_INFO("the real x is: [%f], the real y is:[%f]",real_x,real_y);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"monocular_measure");
	ros::NodeHandle nh;

	ros::Subscriber sub=nh.subscribe("Land_Coordinate",1000,track_coord_cal);

	ros::spin();

	return 0;
}
