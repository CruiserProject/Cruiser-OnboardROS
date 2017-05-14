#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <unistd.h>
#include </usr/include/opencv/cv.h>   //changed path
#include </home/ubuntu/SAGACIOUS_EAGLE/src/Cruiser-OnboardROS/dji_sdk_read_cam/include/djicam.h>   //changed path

#include <cruiser/TrackingPosition.h>
#include <cruiser/Flag.h>
#include <cruiser/DeltaPosition.h>
#include <dji_sdk/LocalPosition.h>

#include "kcftracker.hpp"

using namespace std;
using namespace cv;

const float sensor_width=6.17; //相机传感器尺寸参数
const float sensor_height=4.55;
const float focal_length=20; //相机等效焦距
const float optic_angle=94; //相机视角
const float degree=45; //相机与地面的夹角

bool drawing_box = false;
bool gotBB = false;
float height;
bool state=true;
bool positionFlag=false;
bool startFlag=false;
bool initFlag=false;
float x_lt=0,y_lt=0,x_rb=0,y_rb=0;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = false;
bool LAB = false;

KCFTracker tracker;
Rect result;
Rect box;
Mat capture;
static const std::string OPENCV_WINDOW = "tracking";

void trackingCoordCal(float x,float y,float& delta_x,float& delta_y)
{
	ROS_INFO_STREAM("tracking_alg_node : coordinate transformed.");
	//图像坐标原点转换为像素坐标
	float v0=sensor_width/2;
	float u0=sensor_height/2;

	//计算fx.fy
	float fx=focal_length/(sensor_width/640.0);
	float fy=focal_length/(sensor_height/360.0);

	//依据单目测距数学模型进行坐标转换
	delta_x=(x-u0)*height/(sqrt(fx*fx+(y-v0)*sin(degree-atan((y-v0)/fy))));
	delta_y=height/tan(degree-atan((y-v0)/fy));
}
/*void drawBox(Mat& image, CvRect box, Scalar color, int thick)
{
	rectangle(image, cvPoint(box.x, box.y), cvPoint(box.x + box.width, box.y + box.height), color, thick);
}
*/
/*
void mouseHandler(int event, int x, int y, int flags, void *param)
{
	switch (event)
	{
		case CV_EVENT_MOUSEMOVE:
			if(drawing_box)
			{
				box.width = x - box.x;
				box.height = y - box.y;
			}
			break;
		case CV_EVENT_LBUTTONDOWN:
			drawing_box = true;
			box = Rect(x, y, 0, 0);
			break;
		case CV_EVENT_LBUTTONUP:
			drawing_box = false;


			if(box.width < 0)
			{cruiser::DeltaPosition
				box.x += box.width;
				box.width *= -1;
			}

			if(box.height < 0)
			{cruiser::DeltaPosition
				box.y += box.height;
				box.height *= -1;
			}false
			gotBB = true;
			break;
	}
}
*/
//get drone's global height,height in this function is a global variable
void localPositionCallback(const dji_sdk::LocalPosition &h)
{
	height=h.z;
	ROS_INFO_STREAM("tracking_alg_node : height changed.");
}

void getFlagCallback(const cruiser::Flag &msg)
{
	if(msg.flag)
		startFlag=true;
	else
	{
		positionFlag=false;
		startFlag=false;
		initFlag=false;
	}
	ROS_INFO_STREAM("tracking_alg_node : flag changed.");
}

void getPositionCallback(const cruiser::TrackingPosition &msg)
{
	if(msg.a_width_percent<msg.b_width_percent&&msg.a_height_percent<msg.b_height_percent)
	{
		x_lt=msg.a_width_percent;
		y_lt=msg.a_height_percent;
		x_rb=msg.b_width_percent;
		y_rb=msg.b_height_percent;
	}
	else if(msg.a_width_percent>msg.b_width_percent&&msg.a_height_percent>msg.b_height_percent)
	{
		x_lt=msg.b_width_percent;
		y_lt=msg.b_height_percent;
		x_rb=msg.a_width_percent;
		y_rb=msg.a_height_percent;
	}
	else if(msg.a_width_percent<msg.b_width_percent&&msg.a_height_percent>msg.b_height_percent)
	{
		x_lt=msg.a_width_percent;
		y_lt=msg.b_height_percent;
		x_rb=msg.b_width_percent;
		y_rb=msg.a_height_percent;
	}
	else if(msg.a_width_percent>msg.b_width_percent&&msg.a_height_percent<msg.b_height_percent)
	{
		x_lt=msg.b_width_percent;
		y_lt=msg.a_height_percent;
		x_rb=msg.a_width_percent;
		y_rb=msg.b_height_percent;
	}
	positionFlag=true;
	ROS_INFO_STREAM("tracking_alg_node : get tracking position.");
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Subscriber tracking_flag;
  ros::Subscriber rect_sub;
  ros::Subscriber Height;
  ros::Publisher pub;
  ros::Publisher pubs;
  cruiser::DeltaPosition deltaPosition;
  cruiser::TrackingPosition myPosition;

	public:
  	ImageConverter():it_(nh_)
  	{
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/dji_sdk/image_raw", 1,&ImageConverter::imageCallback, this);
      
      rect_sub = nh_.subscribe("cruiser/tracking_position", 1,&getPositionCallback);
   	  tracking_flag=nh_.subscribe("cruiser/tracking_flag",1,&getFlagCallback);
      Height=nh_.subscribe("/dji_sdk/local_position",1,&localPositionCallback);
      pub=nh_.advertise<cruiser::DeltaPosition>("cruiser/tracking_move",1);
      pubs=nh_.advertise<cruiser::TrackingPosition>("cruiser/tracking_position_now",1);
      cv::namedWindow(OPENCV_WINDOW);
  	}

  	~ImageConverter()
  	{
    	cv::destroyWindow(OPENCV_WINDOW);
  	}

  	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  	{
      if(startFlag&&positionFlag)
      {
    	ROS_INFO_STREAM("tracking_alg_node : start tracking.");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return ;
        }
        capture=cv_ptr->image.clone();   //read camera
        cv::resize(capture,capture,Size(640,360));
        

  /*		if(!gotBB)
			{
				drawBox(capture, box,(0,0,255),2);
				if (!capture.empty())
				{
	 				imshow(OPENCV_WINDOW, capture);
				}
				if(cvWaitKey(33) == 'q')
					return ;
			}
*/
        if(!initFlag)
        {
          //cvSetMouseCallback("tracking",NULL, NULL);
          box.x=x_lt*640;
          box.y=y_lt*360;
          box.width=(x_rb-x_lt)*640;
          box.height=(y_rb-x_lt)*360;
          tracker.init(box, capture);
          initFlag=true;
        }

        if(initFlag)
        {
          result = tracker.update(capture);
          deltaPosition.state=state;

          //caluate the deltaPosition in the ground coordinate system
          trackingCoordCal(result.x+result.width/2,result.y+result.height/2,deltaPosition.delta_X_meter,deltaPosition.delta_Y_meter);
          //cout << "target is located at: (" <<result.x<<","<<result.y<<")"<< endl;
          rectangle(capture, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 0, 255), 1, 8);
          //msg.a_width_percent msg.b_width_percent msg.a_height_percent msg.b_height_percent
          myPosition.a_width_percent=result.x/640.0;
          myPosition.a_height_percent=result.y/360.0;
          myPosition.b_width_percent=(result.x+result.width)/640.0;
          myPosition.b_height_percent=(result.y+result.height)/360.0;
          if(!SILENT)
          {
            if (!capture.empty())
            {
              imshow(OPENCV_WINDOW, capture);
              waitKey(10);
            }
          }
          pub.publish(deltaPosition);
          ROS_INFO_STREAM("tracking_alg_node : publish delta position "
  				<< deltaPosition.delta_X_meter << " " << deltaPosition.delta_Y_meter);
          pubs.publish(myPosition);
          ROS_INFO_STREAM("tracking_alg_node : publish delta position "
  				<< myPosition.a_width_percent << " " << myPosition.a_height_percent
  				<< myPosition.b_width_percent << " " << myPosition.b_height_percent);
        }
      }
    }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracking_alg_node");
	ImageConverter ic;
	//cvSetMouseCallback("tracking", mouseHandler, NULL);
	tracker=KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	ROS_INFO_STREAM("tracking_alg_node : initialization.");
	ros::spin();
	return 0;
}
