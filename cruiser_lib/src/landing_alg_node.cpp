#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
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

#include <dji_sdk/LocalPosition.h>
#include <cruiser/Flag.h>
#include <cruiser/DeltaPosition.h>
//#include <dji_sdk/dji_drone.h>

using namespace std;
using namespace cv;

Mat srcImage;
cruiser::DeltaPosition deltaPosition;

const float sensor_weight=6.17;//相机传感器尺寸数据
const float sensor_height=4.55;
const float focal_length=20;//相机等效焦距
const float optic_angle=94;//相机视角

float height;//定义全局变量，获取当前飞行高度
static const std::string OPENCV_WINDOW = "landpoint";
bool flag=false;

//get the local height of drone
void localPositionCallback(const dji_sdk::LocalPosition& h)
{
	height=h.z;
}

//get the flag which depends whether to execute the houghcircle programme
void landingFlagCallback(const cruiser::Flag& msg)
{
	flag=msg.flag;
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	ros::Subscriber Height;//get local hight
	ros::Subscriber Landing_flag;//get flag
	ros::Publisher delta_location;//publish delta_location message

	public:
		ImageConverter() : it_(nh_)
		{
			// Subscrive to input video feed and publish output video feed
			image_sub_ = it_.subscribe("/dji_sdk/image_raw", 1,&ImageConverter::imageCb, this);
			Height=nh_.subscribe("/dji_sdk/local_position",1,&localPositionCallback);
			Landing_flag=nh_.subscribe("cruiser/landing_flag",1,&landingFlagCallback);
			delta_location=nh_.advertise<cruiser::DeltaPosition>("cruiser/landing_move",1);

			//cv::namedWindow(OPENCV_WINDOW);
		}

		~ImageConverter()
		{
			cv::destroyWindow(OPENCV_WINDOW);
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg)
		{
			cv_bridge::CvImagePtr cv_ptr;
			sensor_msgs::Image im;

			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
//			ROS_ERROR("cv_bridge exception: %s", e.what());
//			ROS_ERROR("cv_bridge exception: %s", e.what());

				return;
			}

			if(flag)
			{
				srcImage=cv_ptr->image.clone();
				cv::resize(srcImage,srcImage,Size(640,480));
				Mat midImage;//临时变量和目标图的定义
				cvtColor(srcImage,midImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
				GaussianBlur( midImage, midImage, Size(9, 9), 2, 2 );

				vector<Vec3f> circles;
				HoughCircles( midImage, circles, CV_HOUGH_GRADIENT,1.5, 10, 200, 100, 0, 0 );

				float  add_pointx = 0, add_pointy = 0; int size, add_radius = 0;
				for( size_t i = 0; i < circles.size(); i++ )
				{
			  	Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					int radius = cvRound(circles[i][2]);
					add_radius += radius;
					add_pointx += cvRound(circles[i][0]);
					add_pointy += cvRound(circles[i][1]);

					// circle( srcImage, center, 3, Scalar(0,255,255), -1, 8, 0 );

					// circle( srcImage, center, radius, Scalar(155,50,255), 3, 8, 0 );
					size = i;
				}

				int radius = add_radius / (size + 1);
				float x = add_pointx / (size + 1);
				float y = add_pointy / (size + 1);

				//calculate the parameters in deltaposition
				if(x==0&&y==0)
				{
					ROS_INFO_STREAM("do not detect circle!.");
					deltaPosition.state=false;//there is no circle detected,so the state in deltaposition should be false
					deltaPosition.delta_X_meter=0;
					deltaPosition.delta_Y_meter=0;
				}
				else
				{
					deltaPosition.state=true;//if any circle is detected,the state should be true
					ROS_INFO_STREAM("Detect circle.");
					Point center(x, y);
					cout << "center=" << endl << center <<endl<< "radius=" << endl << radius<<endl;
					circle(srcImage, center, 3, Scalar(0, 59, 255), -1, 8, 0);
					circle(srcImage, center, radius, Scalar(155, 50, 20), 3, 8, 0);

					//计算实际坐标（相对位移）
					int X=srcImage.cols;
					int Y=srcImage.rows;

					//corordinate transform
					x=x/X;
					y=y/Y;
					
					x=(sensor_weight*0.001/2-x*sensor_weight*0.001)/(focal_length*0.001/height);
					y=(sensor_height*0.001/2-y*sensor_height*0.001)/(focal_length*0.001/height);
					ROS_INFO_STREAM("delta_X_meter = "<< x << " delta_Y_meter = " << y);

					deltaPosition.delta_X_meter=x;
					deltaPosition.delta_Y_meter=y;
				}
				//cv::imshow(OPENCV_WINDOW, srcImage);
				//cv::waitKey(3);
				// Output modified video stream
				//cv_ptr->image=srcImage;
				//image_pub_.publish(srcImage.toImageMsg());
   			    //ROS_INFO_STREAM("delta_X = "<< new_delta.delta_X << " delta_Y = " << new_delta.delta_Y <<" delta_flag ="<< new_delta.flag);
				delta_location.publish(deltaPosition);//发布坐标消息
			}
		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_alg_node");
  ImageConverter ic;
  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
