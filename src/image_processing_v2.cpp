#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Int16.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

int average_x = 0;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 
  ros::Publisher x_pub;
 
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nav_kinect/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);


    x_pub = nh_.advertise<std_msgs::Int16>("x", 1000);
   
   
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
   
   
    //declare output image
    cv::Mat outImg;
    cv::Mat original_image = cv_ptr->image;
   
    // Example 1: Draw an example circle on the video stream
    //getting the pink
   

    //Example 2: make a grayscale of the image
    //cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY);

    //Example 2.1: thresholding
    /*
    cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY);
    int T = 25;
    for (unsigned int i = 0; i < outImg.rows; i ++){
        for (unsigned int j = 0; j < outImg.cols; j ++){
            if ( outImg.at<uchar>(i,j) < T)
                outImg.at<uchar>(i,j) = 0;
            else
                outImg.at<uchar>(i,j) = 255;
        }
    }
    */
 
    //Example 2.2: thresholding using openCV
    //reference: http://docs.opencv.org/2.4/doc/tutorials/imgproc/threshold/threshold.html
    //cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY);
    //cv::threshold(outImg, outImg, 90, 255, 3);
    //Example 3: remove either the B, the G, or the R
    //note that the order of color channels is BGR by default, not RGB
   
    outImg = cv_ptr->image.clone();
   
    blur(cv_ptr->image, cv_ptr->image, Size(4, 4) , Point(-1, -1), BORDER_DEFAULT);
   
    for (unsigned int i = 0; i < outImg.rows; i ++){
        for (unsigned int j = 0; j < outImg.cols; j ++){
            //outImg.at<cv::Vec3b>(i,j)[2] = 0;  //set blue to 0
       
       
            //example of how to look up pixel values
            int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
            int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
            int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
           
            if(b_ij > 80 && b_ij < 180 && g_ij > 0 && g_ij < 90 && r_ij > 130 & r_ij <230)
            {
                outImg.at<cv::Vec3b>(i,j)[0] = 255;
                outImg.at<cv::Vec3b>(i,j)[1] = 255;
                outImg.at<cv::Vec3b>(i,j)[2] = 255;
            }
            else
            {
                outImg.at<cv::Vec3b>(i,j)[0] = 0;
                outImg.at<cv::Vec3b>(i,j)[1] = 0;
                outImg.at<cv::Vec3b>(i,j)[2] = 0;
            }
           
        }
    }
   
    //erosion and dilation
    erode(outImg, outImg, Mat());
    dilate(outImg, outImg, Mat());

    //finding the center of the circle
   
    int x_sum = 0;
    int y_sum = 0;
    int count = 0;
   
    for (int i = 0; i < outImg.rows; i ++){
        for (int j = 0; j < outImg.cols; j ++){
            if((int)outImg.at<cv::Vec3b>(i,j)[0] == 255)
            {
                x_sum += j;
                y_sum += i;
                count++;
            }
        }
    }
   
   
    average_x = 0;
    int average_y = 0;
   
    if(count > 0)
    {
        average_x = x_sum/count;
        average_y = y_sum/count;
        cv::circle(outImg, cv::Point(average_x,average_y), 20, CV_RGB(255,0,0), 1, 8, 0);
    }

   
   
    //Example 4: smoothing and image difference
    /*
    outImg = cv_ptr->image.clone();
    cv::GaussianBlur( outImg, outImg, cv::Size( 55, 55 ), 0, 0 );
   
    for (unsigned int i = 0; i < outImg.rows; i ++){
        for (unsigned int j = 0; j < outImg.cols; j ++){
            //example of how to look up pixel values
            int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
            int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
            int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
           
            //example of how to look up pixel values
            int b_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[0];
            int g_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[1];
            int r_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[2];
           
            outImg.at<cv::Vec3b>(i,j)[0] = b_ij2 - b_ij; 
            outImg.at<cv::Vec3b>(i,j)[1] = g_ij2 - g_ij; 
            outImg.at<cv::Vec3b>(i,j)[2] = r_ij2 - r_ij;
        }
    }
   
   
    //Example 5: edge detection
    /*cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY);
    int edge_threshold = 50; //between 0 and 100
    int edge_canny_ratio = 3;
    int edge_kernel_size = 3;
    cv::GaussianBlur(outImg, outImg, cv::Size( 7, 7 ), 0, 0 );
   
    cv::Canny(outImg, outImg, edge_threshold, edge_threshold*edge_canny_ratio, edge_kernel_size );     */
   

    //show input
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
   
    //show output
    cv::imshow(OUT_WINDOW, outImg);
   
    //pause for 3 ms
    cv::waitKey(3);
   
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
   
    //
    std_msgs::Int16 msg1;
    msg1.data = average_x;

    ROS_INFO("%d", msg1.data);
    x_pub.publish(msg1);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
