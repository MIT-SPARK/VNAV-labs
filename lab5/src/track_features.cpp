/*
 * @file track_features.cpp
 * @brief Track features from frame to frame.
 */

#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <stdio.h>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "akaze_feature_tracker.h"
#include "brisk_feature_tracker.h"
#include "feature_tracker.h"
#include "lk_feature_tracker.h"
#include "orb_feature_tracker.h"
#include "sift_feature_tracker.h"

class TrackerNode : public rclcpp::Node {
public:
  int mode_;
  std::string descriptor_;
  bool save_images_;
  bool show_images_;

  image_transport::Subscriber sub_;

  TrackerNode() : Node("tracker_node") {

    declare_parameter<int>("mode");
    declare_parameter<std::string>("descriptor");
    declare_parameter<std::string>("img1_path");
    declare_parameter<std::string>("img2_path");

    declare_parameter<bool>("save_images");
    declare_parameter<bool>("show_images");

    if (!(get_parameter("mode", mode_) &&
          get_parameter("descriptor", descriptor_))) {
      RCLCPP_ERROR(get_logger(),
                   "Much be launched with desired `mode` and `descriptor`");
    }
    LOG(INFO) << "Descriptor: " << descriptor_;
    LOG(INFO) << "Mode: " << mode_;

    get_parameter("save_images", save_images_);
    get_parameter("show_images", show_images_);
  }

  void run() {
    std::shared_ptr<FeatureTracker> feature_tracker;
    std::shared_ptr<LKFeatureTracker> lk_tracker;

    if (descriptor_ == "SIFT") { // default: SIFT
      LOG(INFO) << "Using SIFT";
      feature_tracker.reset(new SiftFeatureTracker());
      // (TODO) Implement the functions in SiftFeatureTracker
    } else if (descriptor_ == "ORB") {
      LOG(INFO) << "Using ORB";
      feature_tracker.reset(new OrbFeatureTracker());
      // (TODO) Implement the functions in OrbFeatureTracker
    } else if (descriptor_ == "AKAZE") {
      LOG(INFO) << "Using AKAZE";
      feature_tracker.reset(new AkazeFeatureTracker());
      // (TODO) Implement the functions in AKAZEFeatureTracker
    } else if (descriptor_ == "BRISK") {
      LOG(INFO) << "Using BRISK";
      feature_tracker.reset(new BriskFeatureTracker());
      // (TODO) Implement the functions in BRISKFeatureTracker
    } else if (descriptor_ == "LK") {
      lk_tracker.reset(new LKFeatureTracker());
      // (TODO) Implement the functions in LKFeatureTracker
    } else {
      LOG(ERROR) << "Unknown descriptor type" << descriptor_;
      return;
    }

    if (mode_ == 0) {
      // IMAGE
      cv::Mat img_1, img_2;
      if (!retrieveImages(&img_1, &img_2)) {
        RCLCPP_FATAL(get_logger(), "Could not retrieveImages!");
        return;
      }
      RCLCPP_WARN(get_logger(), "Retrieved Images!");
      feature_tracker->trackFeatures(img_1, img_2, nullptr, save_images_,
                                     show_images_);
      cv::waitKey(0);
    } else if (mode_ == 1) {
      // VIDEO
      cv::namedWindow("view", cv::WINDOW_NORMAL);
      image_transport::ImageTransport it(shared_from_this());

      if (descriptor_ == "LK") {
        sub_ = it.subscribe("/images_topic", 1,
                            std::bind(&TrackerNode::lkImageCallback, this,
                                      std::placeholders::_1, lk_tracker));
      } else {
        sub_ = it.subscribe("/images_topic", 1,
                            std::bind(&TrackerNode::rgbImageCallback, this,
                                      std::placeholders::_1, feature_tracker));
      }
    } else {
      RCLCPP_FATAL_STREAM(get_logger(),
                          "mode must be set to 0 or 1, not " << mode_);
    }
  }

  /** imageCallback This function is called when a new image is published.
   * You will need to modify this function when working with video sequences.
   * For the first part of the assignment (working with a pair of images), you
   * can ignore this function.
   * @brief Tracks features from frame to frame using images received via a
   * ROS topic.
   */
  void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                        std::shared_ptr<FeatureTracker> feature_tracker) {
    try {
      // Convert ROS msg type to OpenCV image type.
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      // Visualize the image.
      cv::imshow("view", image);
      static cv::Mat prev_image = image;
      feature_tracker->trackFeatures(image, prev_image, nullptr, save_images_,
                                     show_images_);

      prev_image = image;
      cv::waitKey(10);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not convert from"
                                            << msg->encoding.c_str()
                                            << "to 'bgr8'.");
    }
  }

  /**
   * @brief Callback for LKtracker
   *
   */
  void lkImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                       std::shared_ptr<LKFeatureTracker> lk_tracker) {
    try {
      // Convert ROS msg type to OpenCV image type.
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      // Visualize the image.
      lk_tracker->trackFeatures(image);
      cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not convert from"
                                            << msg->encoding.c_str()
                                            << " to 'bgr8'.");
    }
  }

  /** You do not need to modify this function
   * @function retrieveImages
   * @brief Retrieves a pair of images using ROS params
   */
  bool retrieveImages(cv::Mat *img_1, cv::Mat *img_2) {
    CHECK_NOTNULL(img_1);
    CHECK_NOTNULL(img_2);
    std::string img1_path;
    if (get_parameter("img1_path", img1_path)) {
      RCLCPP_INFO(get_logger(), "Got param: %s", img1_path.c_str());
    } else {
      RCLCPP_FATAL(get_logger(), "Failed to get param 'img1_path'");
      return false;
    }

    std::string img2_path;
    if (get_parameter("img2_path", img2_path)) {
      RCLCPP_INFO(get_logger(), "Got param: %s", img2_path.c_str());
    } else {
      RCLCPP_FATAL(get_logger(), "Failed to get param 'img2_path'");
      return false;
    }

    *img_1 = imread(img1_path, cv::IMREAD_GRAYSCALE);
    *img_2 = imread(img2_path, cv::IMREAD_GRAYSCALE);
    if (!img_1->data || !img_2->data) {
      RCLCPP_FATAL(get_logger(), " --(!) Error reading images ");
      return false;
    }

    return true;
  }
};

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackerNode>();
  node->run();
  rclcpp::spin(node);
  return 0;
}
