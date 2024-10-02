/*
 * @file self_flow.cpp
 * @brief Track features from frame to frame.
 */

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <stdio.h>

#include <fstream>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>

#include "brisk_feature_tracker.h"
#include "feature_tracker.h"
#include "lk_feature_tracker.h"
#include "orb_feature_tracker.h"
#include "sift_feature_tracker.h"

class SelfFlow : public rclcpp::Node {
 public:
  SelfFlow() : Node("self_flow") {}

  void showFlow(cv::Mat const& frame, cv::Mat const& flow, int spacing = 20) {
    cv::Mat annotatedFrame = frame.clone();
    cv::cvtColor(frame, annotatedFrame, CV_GRAY2BGR);

    // TODO: Display optical flow
  }

  /** imageCallback This function is called when a new image is published.
   * For the first part of the assignment (working with a pair of images), you
   * can ignore this function.
   * @brief Tracks features from frame to frame using images received via a ROS
   *  topic.
   */
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      // Convert ROS msg type to OpenCV image type.
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  DELIVERABLE 8 | Optical Flow
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      // ~ ~
      //
      // LK tracker estimates the optical flow for sparse points in the image.
      // Alternatively, dense approaches try to estimate the optical flow for
      // the whole image. Try to calculate your own optical flow using
      // Farneback’s algorithm (see OpenCV documentation).
      //
      // ~~~~ begin solution
      //
      //     **** TODO: FILL IN HERE ***

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "Could not convert from '%s' to 'bgr8'.",
                   msg->encoding.c_str());
    }
    //
    // ~~~~ end solution
  }

  void run() {
    cv::namedWindow("view", cv::WINDOW_NORMAL);
    image_transport::ImageTransport it(shared_from_this());
    std::map<std::string, std::string> stats;
    image_transport::Subscriber sub =
        it.subscribe("/images_topic",
                     100,
                     std::bind(&SelfFlow::imageCallback, this, std::placeholders::_1));
  }
};

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfFlow>();
  node->run();
  rclcpp::spin(node);

  return EXIT_SUCCESS;
}
