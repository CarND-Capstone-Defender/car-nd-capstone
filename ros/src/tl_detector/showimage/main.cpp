#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    cv::Mat rgb_image;
    cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
    cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    cv::imshow("view", rgb_image);

    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  ros::Subscriber sub = nh.subscribe("/tl_classifier/image_raw/compressed", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}