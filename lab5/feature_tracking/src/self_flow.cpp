/*
 * @file self_flow.cpp
 * Feel free to edit any part of this file!
 */
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

/** imageCallback This function is called when a new image is published. */
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    // Convert ROS msg type to OpenCV image type.
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  DELIVERABLE 8 | Optical Flow
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // LK tracker estimates the optical flow for sparse points in the image.
    // Alternatively, dense approaches try to estimate the optical flow for the whole image.
    // Try to calculate your own optical flow using Farneback’s algorithm (see OpenCV documentation).
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
    //
    // ~~~~ end solution

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/images_topic", 100, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
