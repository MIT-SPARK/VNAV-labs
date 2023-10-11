/*
 * @file pose_estimation.cpp
 * @brief Estimates the pose from frame to frame.
 */


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// OpenGV
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/point_cloud/methods.hpp>

#include "lab6_utils.h"
#include "pose_estimation.h"
#include "tracker_shim.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 6 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to use the following methods for motion estimation:
//  - Nister's 5 point algorithm (2D-2D correspondences)
//  - Longuet-Higgins 8 point algorithm (2D-2D correspondences)
//  - OpenGV's 2 point algorithm (2D-2D correspondences, known rotation)
//  - Arun's 3 point algorithm (3D-3D correspondences)
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// These are Google Flags (also known as gflags).
// Have a look at the provided roslaunch to see how to modify them on the fly.
// Feel free to use ROS parameters, or command-line flags if you prefer.
DEFINE_bool(use_ransac, true, "Use Random Sample Consensus.");
DEFINE_bool(scale_translation, true, "Whether to scale estimated translation to match ground truth scale");
DEFINE_int32(pose_estimator, 0,
             "Pose estimation algorithm, valid values are:"
             "0 for OpengGV's 5-point algorithm."
             "1 for OpengGV's 8-point algorithm."
             "2 for OpengGV's 2-point algorithm."
             "3 for Arun's 3-point method.");

using namespace std;
namespace enc = sensor_msgs::image_encodings;

// Global definitions.
// Mono OpenGV RANSAC problems and adapters.
// See OpenGV examples on how to use:
// https://laurentkneip.github.io/opengv/page_how_to_use.html
using RansacProblem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Adapter = opengv::relative_pose::CentralRelativeAdapter;
using RansacProblemGivenRot = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using AdapterGivenRot = opengv::relative_pose::CentralRelativeAdapter;
using Adapter3D = opengv::point_cloud::PointCloudAdapter;
using RansacProblem3D = opengv::sac_problems::point_cloud::PointCloudSacProblem;

// Global variables. As a general coding style, global variables are followed
// by a trailing underscore (Google coding style).
std::unique_ptr<TrackerWrapper> feature_tracker_;
ros::Publisher pub_pose_estimation_, pub_pose_gt_;
ros::Subscriber sub_cinfo_;
geometry_msgs::PoseStamped curr_pose_;
geometry_msgs::PoseStamped prev_pose_;

// See definition of CameraParams in lab6_utils.h
// We declare these global variables, and we assign values to them later in the main function
CameraParams camera_params_;
cv::Mat R_camera_body, t_camera_body;
cv::Mat T_camera_body;
geometry_msgs::Pose pose_camera_body;
tf::Transform transform_camera_body;


void poseCallbackTesse(const nav_msgs::Odometry::ConstPtr& msg){
  // This function takes a message from the simulator of type nav_msgs::Odometry,
  // and use it to update the ground-truth pose of the drone's camera frame.
  // 
  // Note that the simulator provides the pose of the drone's body frame,
  // Therefore, we need to use the fixed transformation from the drone's camera
  // to the drone's camera to obtain the pose of the drone's camera frame.
  // 
  // In math, we want to get T_{camera}^W, but the msg provides T_{body}^W,
  // so we should do T_{camera}^W = T_{body}^W * T_{camera}^{body} to get the 
  // drone's camera pose.

  // This is T_{body}^W
  curr_pose_.pose = msg->pose.pose;

  // Convert pose message to tf::Transform
  tf::Transform current_pose;
  tf::poseMsgToTF(curr_pose_.pose, current_pose);

  // Perform the coordinate transform T_{camera}^W = T_{body}^W * T_{camera}^{body}
  // and convert to pose message
  tf::poseTFToMsg(current_pose * transform_camera_body, curr_pose_.pose);

  // publish the converted pose message so we can visualize in rViz
  curr_pose_.header.frame_id = "world";
  pub_pose_gt_.publish(curr_pose_);
}

/**
 * @brief      Compute 3D bearing vectors from pixel points
 *
 * @param[in]  pts1              Feature correspondences from camera 1
 * @param[in]  pts2              Feature correspondences from camera 2
 * @param      bearing_vector_1  Bearing vector to pts1 in camera 1
 * @param      bearing_vector_2  Bearing vector to pts2 in camera 2
 */
void calibrateKeypoints(const std::vector<cv::Point2f>& pts1,
                        const std::vector<cv::Point2f>& pts2,
                        opengv::bearingVectors_t& bearing_vector_1,
                        opengv::bearingVectors_t& bearing_vector_2) {
    //
    // For this part, we perform:
    //   1. Use the function cv::undistortPoints to rectify the keypoints.
    //   2. Return the bearing vectors for each keypoint.
    //

    std::vector<cv::Point2f> points1_rect, points2_rect;
    cv::undistortPoints(pts1, points1_rect, camera_params_.K, camera_params_.D);
    cv::undistortPoints(pts2, points2_rect, camera_params_.K, camera_params_.D);

    for (auto const& pt: points1_rect){
      opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
      bearing_vector_1.push_back(bearing_vector.normalized());
    }

    for (auto const& pt: points2_rect){
      opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
      bearing_vector_2.push_back(bearing_vector.normalized());
    }
}

/**
 * @brief      Update pose estimate using previous absolue pose and estimated relative pose
 *
 * @param[in]  prev_pose         ground-truth absolute pose of previous frame
 * @param[in]  relative_pose     estimated relative pose between current frame and previous frame
 * @param      output            estimated absolute pose of current frame
 */
void updatePoseEstimate(geometry_msgs::Pose const& prev_pose, geometry_msgs::Pose const& relative_pose, geometry_msgs::Pose& output) {
  tf::Transform prev, relative;
  tf::poseMsgToTF(prev_pose, prev);
  tf::poseMsgToTF(relative_pose, relative);
  tf::poseTFToMsg(prev*relative, output);
}

/**
 * @brief      Given an estimated translation up to scale, return an absolute translation with correct scale using ground truth
 *
 * @param[in]  prev_pose         ground-truth absolute pose of previous frame
 * @param[in]  curr_pose         ground-truth absolute pose of current frame
 * @param      translation       estimated translation between current frame and previous frame
 */
void scaleTranslation(geometry_msgs::Point& translation, geometry_msgs::PoseStamped const& prev_pose, geometry_msgs::PoseStamped const& curr_pose) {
  if (!FLAGS_scale_translation) return;
  tf::Transform prev, curr;
  tf::poseMsgToTF(prev_pose.pose, prev);
  tf::poseMsgToTF(curr_pose.pose, curr);
  tf::Transform const relative_pose = prev.inverseTimes(curr);
  double const translation_scale = relative_pose.getOrigin().length();
  if (isnan(translation_scale) || isinf(translation_scale)) {
    ROS_WARN("Failed to scale translation");
    return;
  }
  double const old_scale = sqrt(pow(translation.x, 2) + pow(translation.y, 2) + pow(translation.z, 2));
  translation.x *= translation_scale / old_scale;
  translation.y *= translation_scale / old_scale;
  translation.z *= translation_scale / old_scale;
}


/** @brief     (TODO) Compute Relative Pose Error (RPE) in translation and rotation.
    @param[in] gt_t_prev_frame ground-truth transform for previous frame.
    @param[in] gt_t_curr_frame ground-truth transform for current frame.
    @param[in] est_t_prev_frame estimated transform for previous frame.
    @param[in] est_t_curr_frame estimated transform for current frame.
*/
void evaluateRPE(const tf::Transform& gt_t_prev_frame,
                 const tf::Transform& gt_t_curr_frame,
                 const tf::Transform& est_t_prev_frame,
                 const tf::Transform& est_t_curr_frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4 (Part 2) | Evaluate Relative Pose Error (RPE)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //   1. Follow the equations given in the README to compute the relative pose
  //     error at each time frame
  //   2. Print the error in translation and rotation.
  //   3. For plotting the statics, you might want to log the errors in a .txt or .csv file
  //
  // Hint: You might find tf::vectorTFToEigen and tf::matrixTFToEigen helpful

  tf::Transform const est_relative_pose = est_t_prev_frame.inverseTimes(est_t_curr_frame);
  tf::Transform const gt_relative_pose = gt_t_prev_frame.inverseTimes(gt_t_curr_frame);

  // ***************************** begin solution *****************************
  

  // ***************************** end solution *****************************

}

/** @brief (TODO) This function is called when a new image is published. This is
 *   where all the magic happens for this lab
 *  @param[in]  rgb_msg    RGB Camera message
 *  @param[in]  depth_msg  Depth Camera message
 */
void cameraCallback(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4-5 | A Suite of Methods for Motion Estimation
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //   1. (Deliverable 4) 2D-2D Correspondences
  //     - 5 point Nister (with RANSAC)
  //     - 8 point Longuet-Higgins (with RANSAC)
  //     - 2 point OpenGV (with RANSAC)
  //   2. (Deliverable 5) 3D-3D Correspondences
  //     - 3 point Arun (with and without RANSAC)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  cv::Mat bgr, depth;

  try {
    // Convert ROS msg type to OpenCV image type.
    bgr = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
    depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert rgb or depth images.");
  }

  // for drawing
  cv::Mat view = bgr.clone();

  // cv::imshow("view", view);

  // The static keyword makes this variable to be created only once.
  // Therefore, this line of code will only be evaluated the first time
  // cameraCallback is called, but never again.
  // Nevertheless, the variable `prev_image` will continue to be available.
  // store images from previous time step
  static cv::Mat prev_bgr = bgr.clone();
  static cv::Mat prev_depth = depth.clone();

  // Track features returns the 2D-2D matches between images
  // in pixels (pixel coords in image 1 -> pixel coords in image 2).
  std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> matched_kp_1_kp_2;
  feature_tracker_->track(prev_bgr, bgr, &matched_kp_1_kp_2);

  int N = matched_kp_1_kp_2.first.size();
  std::cout << "Matched " << N << " keypoints" << std::endl;
  std::vector<cv::Point2f> pts1, pts2;
  
  cv::KeyPoint::convert(matched_kp_1_kp_2.first, pts1);
  cv::KeyPoint::convert(matched_kp_1_kp_2.second, pts2);

  // (TODO) undistort the image keypoints and convert them to bearing vectors 
  // using camera intrinsics
  // Use the calibrateKeypoints function defined
  // Store the results in bearing_vector_1 and bearing_vector_2 defined here.
  opengv::bearingVectors_t bearing_vector_1, bearing_vector_2;
  // ***************************** begin solution *****************************
        


  // ***************************** end solution *****************************

  // We create the central relative adapter, have a look at OpenGV's
  // documentation to understand how this class works.
  Adapter adapter_mono (bearing_vector_1, bearing_vector_2);

  // (TODO) Compute the pose estimate using the different techniques
  // here. You should fill the variable `pose_estimate` with your pose
  // estimation.
  geometry_msgs::PoseStamped pose_estimation;
  tf::poseTFToMsg(tf::Pose::getIdentity(), pose_estimation.pose);

  // initialize a relative pose estimate, you will update relative_pose_estimate using the following algorithms
  geometry_msgs::Pose relative_pose_estimate = pose_estimation.pose;

  switch(FLAGS_pose_estimator) {
  case 0: {
    // *********************************************************************************
    // ******************************** 5-point Method *********************************
    // *********************************************************************************

    // First, ensure we have the minimal number of correspondences to compute the relative pose.
    static constexpr size_t min_nr_of_correspondences = 5;
    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!FLAGS_use_ransac) {
        // Without RANSAC [OPTIONAL]
        // Consider using the "extractPose" function provided to obtain inliers, rotation, and translation

        // ***************************** begin solution *****************************
        

        // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // Consider using ``eigen2Pose`` to convert the solution of ransac to ``relative_pose_estimate``

        // ***************************** begin solution *****************************
        


        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to compute pose estimation using"
               " Nister's algorithm.");
    }
    break;
  }
  case 1: {
    // *********************************************************************************
    // ******************************** 8-point Method *********************************
    // *********************************************************************************

    static constexpr size_t min_nr_of_correspondences = 8;

    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!FLAGS_use_ransac) {
        // Without RANSAC [OPTIONAL]
        // Consider using the "extractPose" function provided to obtain inliers, rotation, and translation
        // Consider using "cv2Pose(Rmat, tmat)" to recover the rotation Rmat and translation tmat

        // ***************************** begin solution *****************************
        


       // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // Consider using ``eigen2Pose`` to convert the solution of ransac to ``relative_pose_estimate``


        // ***************************** begin solution *****************************
        

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to compute pose estimation using"
               " Longuet-Higgins' algorithm.");
    }
    break;
  }
  case 2: {
    // *********************************************************************************
    // ******************************** 2-point Method *********************************
    // *********************************************************************************

    static constexpr size_t min_nr_of_correspondences = 2;
    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      
      // Obtain the rotation part from ground-truth
      tf::Transform curr_frame, prev_frame;
      tf::poseMsgToTF(curr_pose_.pose, curr_frame);
      tf::poseMsgToTF(prev_pose_.pose, prev_frame);
      Eigen::Matrix3d rotation;
      tf::matrixTFToEigen(prev_frame.inverseTimes(curr_frame).getBasis(), rotation);
      adapter_mono.setR12(rotation);

      // We only estimate the translation part
      if (!FLAGS_use_ransac) {
        // Without RANSAC (OPTIONAL)
        // ***************************** begin solution *****************************
        


        // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // ***************************** begin solution *****************************
        


        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to estimate relative translation using 2pt algorithm.");
    }
    break;
  }
  case 3: {
    // *********************************************************************************
    // ******************************** 3-point Method *********************************
    // *********************************************************************************

    // Scale the bearing vectors to point clouds, by querying the depth values of each keypoint
    for (int i=0; i<N; i++) {

      // Use the pixel locations of the keypoints to query depth in the depth image  
      double d1 = double( prev_depth.at<float>(std::floor(pts1[i].y), std::floor(pts1[i].x)) ) ;
      double d2 = double( depth.at<float>(std::floor(pts2[i].y), std::floor(pts2[i].x)) ) ;

      // Normalize the bearing vectors such that the last entry is 1
      bearing_vector_1[i] /= bearing_vector_1[i](2,0);
      bearing_vector_2[i] /= bearing_vector_2[i](2,0);

      // Scale the bearing vectors so that the last entry is equal to depth
      bearing_vector_1[i] *= d1;
      bearing_vector_2[i] *= d2;
    }

    // OpenGV PointCloud 3D-3D Adapter
    opengv::points_t cloud_1, cloud_2;
    for (auto i = 0ul; i < bearing_vector_1.size(); i++) {
        cloud_1.push_back(bearing_vector_1[i]);
        cloud_2.push_back(bearing_vector_2[i]);
    }

    Adapter3D adapter_3d(cloud_1, cloud_2);

    static constexpr int min_nr_of_correspondences = 3; 
    if (adapter_3d.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!FLAGS_use_ransac){
        // Without RANSAC (Optional)
        // ***************************** begin solution *****************************
        


        // ***************************** end solution *****************************
      } else{
        // (TODO) With RANSAC
        // ***************************** begin solution *****************************
        



        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to estimate absolute transform using Arun's 3pt algorithm.");
    }
    break;
  }
  default: {
    ROS_ERROR("Wrong pose_estimator flag!");
  }
  }

  // scale the relative pose estimate (for 5-pt, 2-pt and 8-pt, the translation is up to scale)
  if (FLAGS_pose_estimator < 3) {
    scaleTranslation(relative_pose_estimate.position, prev_pose_, curr_pose_);
  }

  // pose estimate w.r.t. ground truth
  updatePoseEstimate(prev_pose_.pose, relative_pose_estimate, pose_estimation.pose);

  // Compute tf::Transform in order to do actual pose operations.
  tf::Transform gt_t_prev_frame, gt_t_curr_frame;
  tf::Transform est_t_prev_frame, est_t_curr_frame;
  tf::poseMsgToTF(pose_estimation.pose, est_t_curr_frame);
  tf::poseMsgToTF(curr_pose_.pose, gt_t_curr_frame);
  tf::poseMsgToTF(prev_pose_.pose, est_t_prev_frame);
  tf::poseMsgToTF(prev_pose_.pose, gt_t_prev_frame);

  // Evaluate pose errors
  evaluateRPE(gt_t_prev_frame, gt_t_curr_frame,est_t_prev_frame,est_t_curr_frame);

  // (TODO) Now, publish your estimated pose here. Since your pose estimate is
  // relative to the previous frame, use the ground-truth pose of the previous
  // frame to publish your pose_estimate in world frame. This will allow you to
  // see how far away is your estimate from the ground-truth one in Rviz.
  // Remember to subscirbe to the right topic in Rviz. Make this abosolute pose
  pose_estimation.header.frame_id = "world";
  pub_pose_estimation_.publish(pose_estimation);

  // Save for next iteration
  prev_bgr = bgr.clone();
  prev_depth = depth.clone();
  prev_pose_ = curr_pose_;
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "keypoint_trackers");
  ros::NodeHandle local_nh("~");

  // populate camera intrinsics and distortion
  camera_params_.K = cv::Mat::zeros(3, 3, CV_64F);
  camera_params_.K.at<double>(0,0) = 415.69219381653056;
  camera_params_.K.at<double>(1,1) = 415.69219381653056;
  camera_params_.K.at<double>(0,2) = 360.0;
  camera_params_.K.at<double>(1,2) = 240.0;
  camera_params_.D = cv::Mat::zeros(cv::Size(5,1),CV_64F);

  // Constant extrinsics of the camera wrt the body, T_{camera}^{body}
  T_camera_body = cv::Mat::zeros(cv::Size(4,4),CV_64F);
  T_camera_body.at<double>(0,2) = 1.0;
  T_camera_body.at<double>(1,0) = -1.0;
  T_camera_body.at<double>(1,3) = 0.05;
  T_camera_body.at<double>(2,1) = -1.0;
  T_camera_body.at<double>(3,3) = 1.0;
  // convert to geometry_msgs::Pose
  R_camera_body = T_camera_body(cv::Range(0,3),cv::Range(0,3));
  t_camera_body = T_camera_body(cv::Range(0,3),cv::Range(3,4));
  pose_camera_body = cv2Pose(R_camera_body,t_camera_body);
  // convert to tf transform
  tf::poseMsgToTF(pose_camera_body, transform_camera_body);

  feature_tracker_.reset(new TrackerWrapper());

  // Subscribe to drone pose estimation.
  auto pose_sub = local_nh.subscribe("/ground_truth_pose", 10, poseCallbackTesse);

  // Subscribe to rgb and depth images.
  image_transport::ImageTransport it(local_nh);
  image_transport::SubscriberFilter sf_rgb(it, "/rgb_images_topic", 1);
  image_transport::SubscriberFilter sf_depth(it, "/depth_images_topic", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sf_rgb, sf_depth);
  sync.registerCallback(cameraCallback);

  // Advertise drone pose.
  pub_pose_gt_ = local_nh.advertise<geometry_msgs::PoseStamped>("/gt_camera_pose", 1);
  pub_pose_estimation_ = local_nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1);

  while (ros::ok()) {
    ros::spinOnce();
    cv::waitKey(1);
  }
  cv::destroyAllWindows();

  return EXIT_SUCCESS;
}
