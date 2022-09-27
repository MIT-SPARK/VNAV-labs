#include "csv_parser.h"

#include <glog/logging.h>

// ROS headers.
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/Int64MultiArray.h>

using namespace std;
using namespace cv;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 8 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to evaluate place recognition
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 3 | Evaluating BoW Place Recognition with RANSAC
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// Follow along with the code below and handout instructions to evaluate BoW
// Place Recognition with RANSAC
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// (TODO) Path to the folder containing the data.csv file, do not specify data.csv!
// I.e. path for "/home/tonirv/datasets/EuRoC/MH_01_easy/mav0/cam0/data.csv" is
//  "/home/tonirv/datasets/EuRoC/MH_01_easy/mav0/cam0/";
cv::String PATH_TO_DATASET_data_csv = "/home/tonirv/datasets/EuRoC/MH_01_easy/mav0/cam0/";

// (TODO) To deal with looping ROS bags, change the variable below to reflect
// the number of images in the dataset that you are using.
// (hint: use `rosbag info <bag name>` to check the total number of messages)
constexpr long unsigned int IMAGE_COUNT = 3682;

// Function that returns the filenames of the two frames with ids frame_1 and
// frame_2 in the Euroc dataset.
bool
getFilenameForImageIdx(const long unsigned int frame_1,
                       const long unsigned int frame_2,
                       std::pair<std::string, std::string>* loop_closure_frames) {
  CHECK_NOTNULL(loop_closure_frames);
  std::ifstream file(PATH_TO_DATASET_data_csv + "data.csv");

  long unsigned int i = 0;
  bool first_done = false, second_done = false;
  CSVIterator loop(file);
  ++loop; // Jump first definition line in data.csv.
  for(; loop != CSVIterator(); ++loop) {
    if (i == frame_1%IMAGE_COUNT) {
      loop_closure_frames->first = (*loop)[1];
      first_done = true;
    }
    if (i == frame_2%IMAGE_COUNT) {
      loop_closure_frames->second = (*loop)[1];
      second_done = true;
    }
    i++;
    if (first_done && second_done) {
      LOG(INFO) << "Parsed loop closure frames.";
      return true;
    }
  }
  return false;
}

// ROS callback to listen to loop closures calculated by ORB-SLAM.
void loopClosureCallback(const std_msgs::Int64MultiArray::ConstPtr& msg) {
  if (msg->data.size() > 0) {
    ROS_INFO("Heard a loop closure!");
  }
  long unsigned int frame_1_idx = msg->data.at(0);
  long unsigned int frame_2_idx = msg->data.at(1);

  // (TODO) Use the function getFilenameForImage to read the image filename
  // from the Euroc dataset folder.
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution

  std::pair<std::string, std::string> image_names;
  if (!getFilenameForImageIdx(frame_1_idx, frame_2_idx, &image_names)){
    ROS_ERROR("Could not find filename for images with loop closure.");
  }

  cv::String first_image_path = PATH_TO_DATASET_data_csv + "data/" + image_names.first;
  cv::String second_image_path = PATH_TO_DATASET_data_csv + "data/" + image_names.second;

  // (TODO) Use cv::imread to read the images from the dataset folder.
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  // *** fill in here ***

  // (TODO) Use the code in previous labs to compute the number of inlier
  // matches between frames to geometrically verify that the loop closure
  // makes sense.
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
  // *** fill in here ***

  // (TODO) Print and report the frames that had a loop closure, as well as an
  // image of the inliers with the number of inliers (in other words provide
  // proof that the loop closure is geometrically sound).
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution

}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // Init ROS node.
  ros::init(argc, argv, "deliverable_2");
  ros::NodeHandle local_nh("");

  // Subscribe to the topic informing about the loop closures from ORB_SLAM
  ros::Subscriber loop_closure_sub = local_nh.subscribe("/loop_closure", 1, loopClosureCallback);

  // ROS spin until killed.
  ros::spin();

  return 0;
}
