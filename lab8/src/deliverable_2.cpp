// ROS headers.
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include "helper_functions.hpp"

// GTSAM headers.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <glog/logging.h>

using namespace std;
using namespace gtsam;

// A projection factor well suited for tracking a point projected from a camera image
// (HINT) A gtsam::Cal3_S2 object represents camera intrinsics (see https://gtsam.org/doxygen/a02852.html#ae7de8f587615c7b0909c06df658e96e5)
using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2023  - Lab 8 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement object localization using GTSAM
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 2 | Object Localization
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// Follow along with the handout instructions to localize the teddy bear using
// YOLO and GTSAM
//
// NOTE: We do not provide template code for this section or the corresponding
// header file.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// TODO Implement me!
// ~~~~ begin solution
//
//     **** FILL IN HERE ***
//
// ~~~~ end solution
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // Init ROS node.
  ros::init(argc, argv, "deliverable_2");
  ros::NodeHandle local_nh("");

  // ROS publishers for ground-truth trajectory -- you'll have to publish the messages yourself
  // Feel free to move these elsewhere
  ros::Publisher gt_marker_pub =
      local_nh.advertise<visualization_msgs::Marker>("/gt_trajectory_lines", 10,
                                                     true);
  ros::Publisher gt_pose_array_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/gt_trajectory", 10,
                                                   true);

  // (TODO) You will need to setup a gtsam factor graph that grows every time there is a new image.
  // You may want to design a new class to help manage the callback(s) involved -- see earlier labs for examples.
  // But, feel free to do it however you want: add your own static variables, functions, classes, etc.

  // (HINT) See helper_functions.hpp -- the inputs, outputs, and documentation of the helper functions may provide guidance if you get stuck

  // (TODO) You will probably want to construct an image_transport::ImageTransport object somewhere,
  // and then construct a subscriber to "camera/rgb/image_color" with it (`imageTransport.subscribe(...)`) to get the color images

  // (TODO) You may want to subscribe to "camera/rgb/camera_info" too (with a normal ros::Subscriber).
  // Inside the sensor_msgs::CameraInfoConstPtr object you can find the K matrix...
  // he K matrix can be used for the gtsam::Cal3_S2 part of a gtsam::GenericProjectionFactor

  // (HINT) Remember that every factor should have a non-zero noise model. When using ground truth positioning, it should be small.
  // (HINT) A good estimate of the camera centroid noise is `gtsam::noiseModel::Isotropic::Sigma(2, 10.0)`

  // (HINT) You can send images for object detection to YOLO (darknet_ros_node) using a ROS Action 
  // client: 'actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>'.

  // ROS spin until killed.
  ros::spin();

  return 0;
}
