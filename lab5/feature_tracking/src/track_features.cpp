/*
 * @file track_features.cpp
 * @brief Track features from frame to frame.
 */

#include <stdio.h>
#include <iostream>
#include <memory>

#include <glog/logging.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "feature_tracker.h"
#include "sift_feature_tracker.h"
#include "surf_feature_tracker.h"
#include "orb_feature_tracker.h"
#include "fast_feature_tracker.h"
#include "lk_feature_tracker.h"

using namespace std;
using namespace cv::xfeatures2d;

/** imageCallback This function is called when a new image is published.
 * For the first part of the assignment (working with a pair of images), you
 * can ignore this function.
 * @brief Tracks features from frame to frame using images received via a ROS
 *  topic.
 */
void imageCallback(const sensor_msgs::ImageConstPtr &msg,
                   std::shared_ptr <FeatureTracker> feature_tracker) {
  try {
    // Convert ROS msg type to OpenCV image type.
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // Visualize the image.
    cv::imshow("view", image);
    static cv::Mat prev_image = image;
    feature_tracker->trackFeatures(image, prev_image);
    cv::waitKey(1);
    prev_image = image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * @brief Callback for LKtracker
 *  You may find this function helpful as LKFeatureTracker does not inherit from
 *  the FeatureTracker base clase
 */
void lkImageCallback(const sensor_msgs::ImageConstPtr &msg,
                     std::shared_ptr<LKFeatureTracker> lk_tracker) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** You do not need to modify this function
 * @function retrieveImages
 * @brief Retrieves a pair of images using ROS params
 */
bool retrieveImages(const ros::NodeHandle &nh,
                    cv::Mat *img_1,
                    cv::Mat *img_2) {
  CHECK_NOTNULL(img_1);
  CHECK_NOTNULL(img_2);
  std::string img1_path;
  if (nh.getParam("img1_path", img1_path)) {
    ROS_INFO("Got param: %s", img1_path.c_str());
  } else {
    ROS_FATAL("Failed to get param 'img1_path'");
    return false;
  }

  std::string img2_path;
  if (nh.getParam("img2_path", img2_path)) {
    ROS_INFO("Got param: %s", img2_path.c_str());
  } else {
    ROS_FATAL("Failed to get param 'img2_path'");
    return false;
  }

  *img_1 = imread(img1_path, cv::IMREAD_GRAYSCALE);
  *img_2 = imread(img2_path, cv::IMREAD_GRAYSCALE);
  if (!img_1->data || !img_2->data) {
    ROS_FATAL(" --(!) Error reading images ");
    return false;
  }

  return true;
}

/**
 * @function main
 * @brief Entry point of the program
 */
int main(int argc,
         char **argv) {
  ros::init(argc, argv, "keypoint_trackers");
  ros::NodeHandle local_nh("~");

  int const mode = local_nh.param<int>("mode", 0);
  std::string const descriptor = local_nh.param<std::string>("descriptor", "");

  std::shared_ptr<FeatureTracker> feature_tracker;
  std::shared_ptr<LKFeatureTracker> lk_tracker;
  if (descriptor == "SIFT") {
    ///////////////////// SIFT ///////////////////////////////////////////////////
    feature_tracker.reset(new SiftFeatureTracker());
    // (TODO) Implement the functions in SiftFeatureTracker
  } else if (descriptor == "SURF") { // Try for different feature trackers!
    // /////////////////// SURF ///////////////////////////////////////////////////
    feature_tracker.reset(new SurfFeatureTracker());
    // (TODO) Implement the functions in SurfFeatureTracker
  } else if (descriptor == "ORB") {
    ///////////////////// ORB ////////////////////////////////////////////////////
    feature_tracker.reset(new OrbFeatureTracker());
    // (TODO) Implement the functions in OrbFeatureTracker
  } else if (descriptor == "FAST") {
    ///////////////////// FAST ///////////////////////////////////////////////////
    feature_tracker.reset(new FastFeatureTracker());
    // (TODO) Implement the functions in FastFeatureTracker
  } else if (descriptor == "LK") {
    ///////////////////// Lucas-Kanade ///////////////////////////////////////////////////
    lk_tracker.reset(new LKFeatureTracker());
    // (TODO) Implement the functions in LKFeatureTracker
  } else {
    ROS_ERROR("Unknown descriptor %s", descriptor.c_str());
    return EXIT_FAILURE;
  }

  if (mode == 0) { // Process images
    if (!feature_tracker) {
      ROS_ERROR("Cannot use descriptor \"%s\" for images", descriptor.c_str());
      return EXIT_FAILURE;
    }

    // Retrieve images using the path stored in ROS params.
    cv::Mat img_1, img_2;
    if (!retrieveImages(local_nh, &img_1, &img_2)) {
      ROS_ERROR("Failed to retrieve images");
      return EXIT_FAILURE;
    }
    feature_tracker->trackFeatures(img_1, img_2);
  } else if (mode == 1) {
    ///////////////////// TODO ///////////////////////////////////////////////////
    ///
    /// Feel free to modify as needed for LK method!
    ///
    /// Try to use a video sequence and track features from frame to frame.
    /// Compute at the same time an average of the statistics mentioned in the
    /// handout.

    cv::namedWindow("view", cv::WINDOW_NORMAL);
    image_transport::ImageTransport it(local_nh);
    auto callback = [feature_tracker, lk_tracker, descriptor](const sensor_msgs::ImageConstPtr &msg) {
      if (descriptor == "LK") {
        lkImageCallback(msg, lk_tracker);
      } else {
        imageCallback(msg, feature_tracker);
      }
    };
    image_transport::Subscriber sub = it.subscribe("/images_topic", 100, callback);
    ros::spin();
    cv::destroyWindow("view");
  } else {
    ROS_ERROR("Unrecognized mode argument");
    return EXIT_FAILURE;
  }

  feature_tracker->printStats();

  return EXIT_SUCCESS;
}
