#include<ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include "ros/ros.h"
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
Mat srcImage;
static const std::string OPENCV_WINDOW = "landpoint";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_(nh_)
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  
        
       srcImage=cv_ptr->image.clone();
        Mat midImage;//临时变量和目标图的定义
        //转为灰度图，进行图像平滑
        cvtColor(srcImage,midImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
        GaussianBlur( midImage, midImage, Size(9, 9), 2, 2 );

        //进行霍夫圆变换
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

          //  circle( srcImage, center, 3, Scalar(0,255,255), -1, 8, 0 );

           // circle( srcImage, center, radius, Scalar(155,50,255), 3, 8, 0 );
            size = i;
        }
            int radius = add_radius / (size + 1);
            float x = add_pointx / (size + 1);
            float y = add_pointy / (size + 1);
            if(x==0&&y==0)
                cout<<"do not detect circle!"<<endl;
            else
            {
                Point center(x, y);
            cout << "center=" << endl << center <<endl<< "radius=" << endl << radius<<endl;
            circle(srcImage, center, 3, Scalar(0, 59, 255), -1, 8, 0);
            circle(srcImage, center, radius, Scalar(155, 50, 20), 3, 8, 0);
            }
            //【6】显示效果图
           // imshow("camera", srcImage);
       // waitKey(30);
   
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW, srcImage);
    cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}



