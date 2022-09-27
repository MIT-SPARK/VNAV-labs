/**
 * @file    SFMExample.cpp
 * @brief   Bundle adjustment.
 */
#include <vector>

// For loading the data, see the comments therein for scenario (camera rotates
// around cube)
#include "deliverable_4.h"
#include "example_data.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as
// Point2 (x, y).
#include <gtsam/geometry/Point2.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/Marginals.h>

#include <ros/ros.h>

using namespace std;
using namespace gtsam;

struct Keypoint {
  Keypoint(const size_t& lmk_idx, const Point2& kpt_coords)
      : lmk_idx_(lmk_idx), kpt_coords_(kpt_coords) {}

  // Index of the landmark which this keypoint represents.
  size_t lmk_idx_;
  // Pixel-wise coordinates of the keypoint.
  Point2 kpt_coords_;
};

void createMeasurements(
    const Cal3_S2::shared_ptr& K, const std::vector<Point3>& landmarks,
    const std::vector<Pose3>& poses,
    std::vector<std::pair<size_t, Keypoint>>* measurements) {
  assert(measurements != nullptr);
  measurements->clear();
  for (size_t i = 0; i < poses.size(); ++i) {
    SimpleCamera camera(poses[i], *K);
    for (size_t j = 0; j < landmarks.size(); ++j) {
      measurements->push_back(
          std::make_pair(i, Keypoint(j, camera.project(landmarks[j]))));
    }
  }
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Init ROS node.
  ros::init(argc, argv, "gtsam_solver");
  ros::NodeHandle local_nh("");

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  noiseModel::Isotropic::shared_ptr measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Create the set of ground-truth poses
  vector<Pose3> camera_poses = createPoses();

  // Create the set of ground-truth landmarks
  vector<Point3> landmarks = createPoints();

  // Create an empty factor graph
  NonlinearFactorGraph graph;

  // TODO: Add a prior on pose x0. This indirectly specifies where the origin is.
  // Insert code below:

  // Simulated measurements from each camera pose.
  // A vector of pairs, where for each pair in the vector, the first component
  // is the index of the camera which took the measurement, the second component
  // is the actual keypoint measurement in pixel coordinates together with the
  // idx of the corresponding landmark.
  std::vector<std::pair<size_t, Keypoint>> measurements;
  createMeasurements(K, landmarks, camera_poses, &measurements);

  // TODO: Add GenericProjectionFactor factors to the graph according to measurements
  // Use the character 'l' to symbolize landmarks, and 'x' to symbolize poses.
  // Insert code below:


  // TODO: Because the structure-from-motion problem has a scale ambiguity,
  // the problem is still under-constrained Here we add a prior on the
  // position of the first landmark. This fixes the scale by indicating the
  // distance between the first camera and the first landmark. All other
  // landmark positions are interpreted using this scale.
  // Insert code below:


  // Print the graph to debug.
  // graph.print("Graph");

  // Create the data structure to hold the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  for (size_t i = 0; i < camera_poses.size(); ++i)
    initialEstimate.insert(Symbol('x', i), camera_poses[i].compose(Pose3(
                                               Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                               Point3(0.05, -0.10, 0.20))));
  for (size_t j = 0; j < landmarks.size(); ++j)
    initialEstimate.insert<Point3>(Symbol('l', j),
                                   landmarks[j] + Point3(-0.25, 0.20, 0.15));
  initialEstimate.print("Initial Estimates:\n");

  // Optimize the graph.
  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-08);
  Values result =
      LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

  result.print("Final results:\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  // (Plot in Rviz the 3D positions of the landmarks before/after
  // optimization.
  ros::Publisher landmarks_gt_pub =
      local_nh.advertise<visualization_msgs::Marker>("/landmarks_gt", 10, true);
  ros::Publisher landmarks_init_pub =
      local_nh.advertise<visualization_msgs::Marker>("/landmarks_init", 10,
                                                     true);
  ros::Publisher landmarks_optimal_pub =
      local_nh.advertise<visualization_msgs::Marker>("/landmarks_optimal", 10,
                                                     true);
  ros::Publisher poses_gt_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/poses_gt", 10, true);
  ros::Publisher poses_init_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/poses_init", 10, true);
  ros::Publisher poses_optimal_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/poses_optimal", 10, true);

  // Plot in Rviz the frame of reference of the cameras before/after
  // optimization.
  // Initial
  std::vector<Point3> initial_landmarks;
  for (int i = 0; i < landmarks.size(); ++i) {
    initial_landmarks.push_back(initialEstimate.at<Point3>(Symbol('l', i)));
  }
  DrawPoint3(landmarks_init_pub, initial_landmarks, marker_color::color_init);
  std::vector<Pose3> initial_poses;
  for (int i = 0; i < camera_poses.size(); ++i) {
    initial_poses.push_back(initialEstimate.at<Pose3>(Symbol('x', i)));
  }
  DrawPoses3(poses_init_pub, initial_poses);

  // Optimal
  std::vector<Point3> optimal_landmarks;
  for (int i = 0; i < landmarks.size(); ++i) {
    optimal_landmarks.push_back(result.at<Point3>(Symbol('l', i)));
  }
  DrawPoint3(landmarks_optimal_pub, optimal_landmarks, marker_color::color_optimal);
  std::vector<Pose3> optimal_poses;
  for (int i = 0; i < camera_poses.size(); ++i) {
    optimal_poses.push_back(result.at<Pose3>(Symbol('x', i)));
  }
  DrawPoses3(poses_optimal_pub, optimal_poses);

  // Plot in Rviz the ground-truth of the above variables (camera poses
  // + landmarks).
  DrawPoint3(landmarks_gt_pub, landmarks, marker_color::color_gt);
  DrawPoses3(poses_gt_pub, camera_poses);


  Marginals marginals(graph, result);
  print(marginals.marginalCovariance(Symbol('l', 1)), "l1 covariance");
  print(marginals.marginalCovariance(Symbol('l', 2)), "l2 covariance");

  // Do not forget to spin ROS.
  ros::spin();

  return 0;
}
/* *************************************************************************
 */

/*
Marginals marginals(graph, result);
  print(marginals.marginalCovariance(x1), "x1 covariance");
  print(marginals.marginalCovariance(x2), "x2 covariance");
  print(marginals.marginalCovariance(x3), "x3 covariance");
  print(marginals.marginalCovariance(l1), "l1 covariance");
  print(marginals.marginalCovariance(l2), "l2 covariance");
  
  https://github.com/pal-robotics/rviz_plugin_covariance
  
  */