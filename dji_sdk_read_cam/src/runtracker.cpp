#include<ros/ros.h>
#include "kcftracker.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
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
#include "cv.h"
#include "highgui.h"
#include "djicam.h"
using namespace std;
using namespace cv;
bool drawing_box = false;
bool gotBB = false;
//my own data
bool flag=true; 
bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = false;
bool LAB = false;   
KCFTracker tracker;
Rect result;
//above are all globel 
Rect box;
Mat capture;
static const std::string OPENCV_WINDOW = "tracking";
void drawBox(Mat& image, CvRect box, Scalar color, int thick){
	rectangle(image, cvPoint(box.x, box.y), cvPoint(box.x + box.width, box.y + box.height), color, thick);
}

void mouseHandler(int event, int x, int y, int flags, void *param){
	switch (event){
	case CV_EVENT_MOUSEMOVE:
		if (drawing_box){
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
		if (box.width < 0){
			box.x += box.width;
			box.width *= -1;
		}
		if (box.height < 0){
			box.y += box.height;
			box.height *= -1;
		}
		gotBB = true;   
		break;
	}
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter():it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/dji_sdk/image_raw", 1,&ImageConverter::imageCb, this);
   // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
    
     
///Initialization 
        if(!gotBB){ 
	      
	drawBox(capture, box,(0,0,255),2);   
	if (!capture.empty())
	{
	 imshow(OPENCV_WINDOW, capture);
	}
	if (cvWaitKey(33) == 'q')
		return ;
	}
	if(flag&&gotBB)		
	{
	cvSetMouseCallback("tracking",NULL, NULL);   
	tracker.init(box, capture);
	flag=false;
	}
	
       if(gotBB)
	{
 	 result = tracker.update(capture);
	 cout << "target is located at: (" <<result.x<<","<<result.y<<")"<< endl;
	 rectangle(capture, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 0, 255), 1, 8);
	 if (!SILENT){
	  if (!capture.empty())
	    {
	     imshow(OPENCV_WINDOW, capture);
	     waitKey(10);
	    }
	  }

 	}





   
   // imshow(OPENCV_WINDOW, capture);
   // cout<<"refresh"<<endl;
   // waitKey(10);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
 	 ros::init(argc, argv, "trackers");
         ImageConverter ic;
         cvSetMouseCallback("tracking", mouseHandler, NULL);   
	
	// Create KCFTracker object
	tracker=KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	// Tracker results

	ros::spin();
       return 0;
}








	

	



