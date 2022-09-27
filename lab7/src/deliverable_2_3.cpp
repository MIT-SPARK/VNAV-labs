/**
 * @file MoCapPose3Example.cpp
 * @brief 3D example with custom MoCap factor.
 */

/**
 * A simple 3D pose-graph with Motion Capture (MoCap) measurement.
 * The robot moves from x1 to x3, with odometry information between each pair.
 * Each step has an associated MoCap measurement.
 * The graph strcuture is:
 *
 *  g1        g3
 *  |         |
 *  x1 - x2 - x3 - ...
 */

#include <random>

// ROS headers.
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

// Custom MoCap factor.
#include "deliverable_2_3.h"

#include "trajectory_color.h"

// GTSAM headers.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <glog/logging.h>

using namespace std;
using namespace gtsam;

// This is just to generate random numbers.
std::random_device rd_{};
std::mt19937 gen_{rd_()};

// Example colors to use for the ground-truth trajectory (gt), etc.
TrajectoryColor color_gt_trajectory(PointColor(0.0, 1.0, 0.0, 1.0),
                                    LineColor(0.0, 1.0, 0.0, 1.0));
TrajectoryColor color_noisy_trajectory(PointColor(1.0, 0.0, 0.0, 0.7),
                                       LineColor(1.0, 1.0, 1.0, 0.7));
TrajectoryColor color_initial_trajectory(PointColor(1.0, 0.0, 0.0, 1.0),
                                         LineColor(1.0, 0.0, 0.0, 1.0));
TrajectoryColor color_optimal_trajectory(PointColor(0.0, 1.0, 1.0, 1.0),
                                         LineColor(0.0, 1.0, 1.0, 1.0));

// Draws a trajectory in Rviz, by publishing a set of lines and poses using
// the publishers passed as argument and the actual trajectory to plot.
// You can also pass what color you want the trajectory in, and the frame_id
// were the Rviz Markers should be plotted (generally set to 'world).
void drawTrajectory(const ros::Publisher& marker_pub,
                    const ros::Publisher& pose_array_pub,
                    const std::vector<geometry_msgs::PoseStamped>& trajectory,
                    const TrajectoryColor& trajectory_color = TrajectoryColor(),
                    const std::string& frame_id = "world") {
  // Create the vertices for the points and lines
  CHECK_GE(trajectory.size(), 0);
  geometry_msgs::PoseStamped prev_i = trajectory.at(0);

  // Create visual markers.
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = frame_id;
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "points_and_lines";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.01;

  // Points are green
  points.color.r = trajectory_color.point_color_.r_;
  points.color.g = trajectory_color.point_color_.g_;
  points.color.b = trajectory_color.point_color_.b_;
  points.color.a = trajectory_color.point_color_.a_;

  // Line strip is blue
  line_strip.color.r = trajectory_color.line_color_.r_;
  line_strip.color.g = trajectory_color.line_color_.g_;
  line_strip.color.b = trajectory_color.line_color_.b_;
  line_strip.color.a = trajectory_color.line_color_.a_;

  // Contains set of poses.
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = frame_id;

  // Loop over the trajectory.
  for (const geometry_msgs::PoseStamped& i : trajectory) {
    if (i.header.stamp.toNSec() < prev_i.header.stamp.toNSec()) {
      LOG(WARNING)
          << "Stamp of poses in trajectory should be increasing in value.\n"
          << "Got timestamp: " << i.header.stamp.toNSec() << " for pose #"
          << i.header.seq << "\n"
          << "Got timestamp: " << prev_i.header.stamp.toNSec() << " for pose #"
          << prev_i.header.seq;
    }
    prev_i = i;
    geometry_msgs::Point p;
    p.x = i.pose.position.x;
    p.y = i.pose.position.y;
    p.z = i.pose.position.z;

    // Create points and lines.
    points.points.push_back(p);
    line_strip.points.push_back(p);

    // Store pose axis.
    pose_array.poses.push_back(i.pose);
  }

  // Publish poses.
  pose_array_pub.publish(pose_array);

  // Publish lines and points.
  marker_pub.publish(points);
  marker_pub.publish(line_strip);
}

// Generates odometry measurements by creating a trajectory, corrupting it with
// noise and returning relative poses between consecutive points.
// It also returns the noisy trajectory from which the measurements were taken.
// The ground-truth trajectory is also returned for plotting.
// The last two arguments specify the level of noise that corrupts the
// measurements.
void generateOdometryMeasurements(
    const int& N_measurements, std::vector<Pose3>* measurements,
    std::vector<Pose3>* noisy_trajectory = nullptr,
    std::vector<Pose3>* gt_trajectory = nullptr,
    const double& rot_std_dev = 1e-6, const double& pos_std_dev = 1e-4) {
  CHECK_NOTNULL(measurements);
  measurements->clear();
  static constexpr double R = 2;

  // Generate ground_truth trajectory.
  std::vector<Pose3> gt_traj;
  for (double i = 0; i < (double)N_measurements; i++) {
    gt_traj.push_back(
        Pose3(Rot3().Yaw(4.0 * M_PI / N_measurements * i),
              Point3(R * std::cos(4.0 * M_PI / N_measurements * i),
                     R * std::sin(4.0 * M_PI / N_measurements * i),
                     i / N_measurements)));
  }

  // Return the gt_trajectory if requested.
  if (gt_trajectory != nullptr) {
    *gt_trajectory = gt_traj;
  }

  // Generate measurements by adding noise to trajectory.
  static std::normal_distribution<> d_rotation{0, rot_std_dev};
  static std::normal_distribution<> d_position{0, pos_std_dev};
  Vector6 e;
  for (const Pose3& gt_pose : *gt_trajectory) {
    e << d_rotation(gen_), d_rotation(gen_), d_rotation(gen_), d_position(gen_),
        d_position(gen_), d_position(gen_);
    measurements->push_back(gt_pose.expmap(e));
  }

  // Return the noisy_trajectory if requested.
  if (noisy_trajectory != nullptr) {
    *noisy_trajectory = *measurements;
  }

  // Calculate relative displacement (actual odometry measurements).
  for (size_t i = N_measurements - 1; i > 0; i--) {
    measurements->at(i) =
        measurements->at(i - 1).inverse().compose(measurements->at(i));
  }
  // Erase the first element, since it does not contain any relative measurement
  // but just the first pose.
  CHECK_GT(measurements->size(), 0);
  measurements->erase(measurements->begin());
}

// mocap_measurements is a vector containing pairs, where the first element
// of the pair represents the index of the pose it is giving information about,
// while the second element provides the measured position of such pose.
void generateMoCapMeasurements(
    const int& N_measurements, const std::vector<Pose3>& gt_trajectory,
    std::vector<std::pair<int, Point3>>* mocap_measurements,
    const double& pos_std_dev = 1e-4) {
  CHECK_NOTNULL(mocap_measurements);
  CHECK_LE(N_measurements, gt_trajectory.size());
  // Generate measurements by adding noise to trajectory.
  static std::normal_distribution<> d_position{0, pos_std_dev};
  int step = std::floor(gt_trajectory.size() / N_measurements);
  for (int i = 0; i < gt_trajectory.size(); i = i + step) {
    mocap_measurements->push_back(std::make_pair(
        i + 1, Point3(gt_trajectory.at(i).x() + d_position(gen_),
                      gt_trajectory.at(i).y() + d_position(gen_),
                      gt_trajectory.at(i).z() + d_position(gen_))));
  }
}

// Convenience function to convert a gtsam Pose to a ROS Pose for visualization.
void gtsamPoseToRosPose(const gtsam::Pose3& gtsam_pose,
                        geometry_msgs::PoseStamped* ros_pose) {
  CHECK_NOTNULL(ros_pose);
  ros_pose->pose.position.x = gtsam_pose.x();
  ros_pose->pose.position.y = gtsam_pose.y();
  ros_pose->pose.position.z = gtsam_pose.z();
  ros_pose->pose.orientation.w = gtsam_pose.rotation().quaternion()(0);
  ros_pose->pose.orientation.x = gtsam_pose.rotation().quaternion()(1);
  ros_pose->pose.orientation.y = gtsam_pose.rotation().quaternion()(2);
  ros_pose->pose.orientation.z = gtsam_pose.rotation().quaternion()(3);
}

// Convenience function to convert a trajectory using gtsam Pose
// to a ROS trajectory using ROS Pose for visualization.
void convertTrajectoryFromGtsamToRos(
    const std::vector<gtsam::Pose3>& gtsam_traj,
    std::vector<geometry_msgs::PoseStamped>* ros_traj) {
  CHECK_NOTNULL(ros_traj);
  geometry_msgs::PoseStamped ros_pose;
  for (const gtsam::Pose3& gtsam_pose : gtsam_traj) {
    gtsamPoseToRosPose(gtsam_pose, &ros_pose);
    ros_traj->push_back(ros_pose);
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // Init ROS node.
  ros::init(argc, argv, "gtsam_example_1");
  ros::NodeHandle local_nh("");

  ros::Publisher pose_pub_ =
      local_nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);

  // Simulate noisy odometry measurements for a robot trajectory in 3D.
  // Store as well the noisy and ground-truth trajectory for later plotting.
  // Hint: to allow easy debugging, try first to generate little measurements
  // (5-10).
  std::vector<Pose3> measurements;
  std::vector<Pose3> noisy_trajectory;
  std::vector<Pose3> gt_trajectory;
  static constexpr int num_measurements = 500;
  static constexpr double rot_std_dev = 1e-2;
  static constexpr double pos_std_dev = 1e-1;
  generateOdometryMeasurements(num_measurements, &measurements,
                               &noisy_trajectory, &gt_trajectory, rot_std_dev,
                               pos_std_dev);

  static constexpr bool use_mocap = true;
  // Simulate motion capture measurements of the actual position of the drone.
  static constexpr int num_mocap_measurements = 10;
  static constexpr double mocap_std_dev = 1e-3;
  std::vector<std::pair<int, Point3>> mocap_measurements;
  if (use_mocap) {
    generateMoCapMeasurements(num_mocap_measurements, gt_trajectory,
                              &mocap_measurements, mocap_std_dev);
  }

  // Convert trajectory to ROS type.
  std::vector<geometry_msgs::PoseStamped> ros_noisy_trajectory;
  convertTrajectoryFromGtsamToRos(noisy_trajectory, &ros_noisy_trajectory);

  // Convert trajectory to ROS type.
  std::vector<geometry_msgs::PoseStamped> gt_ros_trajectory;
  convertTrajectoryFromGtsamToRos(gt_trajectory, &gt_ros_trajectory);

  // ROS publishers for ground-truth trajectory.
  ros::Publisher gt_marker_pub = local_nh.advertise<visualization_msgs::Marker>(
      "/gt_trajectory_lines", 10, true);
  ros::Publisher gt_pose_array_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/gt_trajectory", 10, true);

  // ROS publishers for noisy trajectory.
  ros::Publisher marker_pub = local_nh.advertise<visualization_msgs::Marker>(
      "/trajectory_lines", 10, true);
  ros::Publisher pose_array_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/trajectory", 10, true);

  // ROS publishers for initial trajectory.
  ros::Publisher initial_marker_pub =
      local_nh.advertise<visualization_msgs::Marker>(
          "/initial_trajectory_lines", 10, true);
  ros::Publisher initial_pose_array_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/initial_trajectory", 10,
                                                   true);

  // ROS publishers for optimized trajectory.
  ros::Publisher optimal_marker_pub =
      local_nh.advertise<visualization_msgs::Marker>(
          "/optimal_trajectory_lines", 10, true);
  ros::Publisher optimal_pose_array_pub =
      local_nh.advertise<geometry_msgs::PoseArray>("/optimal_trajectory", 10,
                                                   true);

  // Turn ROS at 50 Hz.
  ros::Rate rate(100);
  size_t i = 0;
  std::string world_frame = "world";
  // Draw noisy and ground-truth trajectories and visulize them in Rviz.
  // When running this node, open rviz either using the configuration file
  // in the rviz folder of this lab, or by adding the topics using Rviz GUI.
  drawTrajectory(gt_marker_pub, gt_pose_array_pub, gt_ros_trajectory,
                 color_gt_trajectory);
  drawTrajectory(marker_pub, pose_array_pub, ros_noisy_trajectory,
                 color_noisy_trajectory);

  // Publish the motion of the robot over the noisy trajectory to have a sense
  // of how corrupted are the measurements observed.
  while (ros::ok() && i < ros_noisy_trajectory.size()) {
    ros_noisy_trajectory.at(i).header.frame_id = world_frame;
    ros_noisy_trajectory.at(i).header.seq = i;
    ros_noisy_trajectory.at(i).header.stamp = ros::Time::now();
    pose_pub_.publish(ros_noisy_trajectory.at(i));
    i++;
    rate.sleep();
  }

  // Create a factor graph container.
  NonlinearFactorGraph graph;

  // Create an Odometry measurement noise model (covariance matrix).
  const auto rot_var = pow(rot_std_dev, 2);
  const auto tra_var = pow(pos_std_dev, 2);
  Vector6 sigmas;
  // sigmas << tra_var, tra_var, tra_var, rot_var, rot_var, rot_var;
  sigmas << rot_std_dev, rot_std_dev, rot_std_dev, pos_std_dev, pos_std_dev,
      pos_std_dev;
  const noiseModel::Diagonal::shared_ptr odometryNoise =
      noiseModel::Diagonal::Sigmas(sigmas);

  // 2a.
  // TODO: Add factors to the graph.
  // Create BetweenFactor(s) between consecutive poses using the odometry
  // measurements.
  //
  // Insert code below:

  // End 2a. 
 
  if (!use_mocap) {
    Matrix init_pose = Matrix::Identity(4, 4);
    init_pose(0, 3) = 2;
    gtsam::Pose3 initial_pose(init_pose);
    const noiseModel::Diagonal::shared_ptr initialNoise =
        noiseModel::Diagonal::Sigmas(sigmas);
    // 2b
    // TODO: Add a prior factor on the node with key 1 
    // to constrain it to initial_pose
    //
    // Insert code below:

    // End 2b.
  }

  if (use_mocap) {
    // 3b. Add motion capture measurements (mocap_measurements)
    // TODO: create the 3D MoCap measurement noise model (a diagonal noise
    // model should be sufficient). Think of how many dimensions it should have.
    // You are given mocap_std_dev (defined above)
    //
    //

    //  TODO: add the MoCap factors
    // Note that there is no prior factor needed at first pose, since MoCap
    // provides the global positions (and rotations given more than 1 MoCap
    // measurement).
    //
    // Insert code below:

    // End 3b. 
  }

  // You can print factor graph for debugging if you want.
//   graph.print("\nFactor Graph:\n");

  // Set the initial variable values for the optimization.
  // They should not be the ground truth ones, and they do not need to be
  // the noisy ones. You can add your own initial guess.
  // Here you have an example:
  Values initial;
  std::vector<Pose3> initial_trajectory;
  for (int i = 0; i <= measurements.size(); i++) {
    const auto p = Pose3(
        Rot3().Yaw(3.0 * M_PI / measurements.size() * (float)i),
        Point3(1.2 * std::cos(3.0 * M_PI / measurements.size() * (float)i),
               0.5 * std::sin(3.0 * M_PI / measurements.size() * (float)i),
               (float)i * 2.0 / measurements.size()));
    initial_trajectory.push_back(p);
    initial.insert(i + 1, p);
  }

  // Plot your initial trajectory in Rviz.
  // Use initials.at<Pose3>(Symbol('x', k)) to retrieve the pose of variable k
  // in initials if you need to. You can convert initial trajectory to ROS
  // using the convertTrajectoryFromGtsamToRos type and plot it in Rviz.
  std::vector<geometry_msgs::PoseStamped> ros_initial_trajectory;
  convertTrajectoryFromGtsamToRos(initial_trajectory, &ros_initial_trajectory);
  drawTrajectory(initial_marker_pub, initial_pose_array_pub,
                 ros_initial_trajectory, color_initial_trajectory);

  // Print initial values for debugging if you want.
  // initials.print("\nInitial Values:\n");

  // Use Gauss-Newton method to optimize the initial values
  GaussNewtonParams parameters;

  // Modify the max number of iterations to see the overall improvement.
  parameters.setMaxIterations(1);
  parameters.setAbsoluteErrorTol(1e-06);

  // Print per iteration.
  parameters.setVerbosity("ERROR");

  // Create the optimizer GaussNewtonOptimizer given the parameters
  // above , the initial values and the graph.
  GaussNewtonOptimizer optimizer(graph, initial, parameters);

  // Finally, Optimize!
  Values results = optimizer.optimize();

  // Print final values for debugging if you want.
  // results.print("Final Result:\n");

  std::cout << "\e[1mOptimization finished.\e[0m" << std::endl;

  // Convert optimal trajectory to ROS type and plot optimized
  // trajectory in Rviz. To access the results use:
  // results.at<Pose3>(Symbol('x', key)), where key is the id of the pose for
  // which you want the result.
  std::vector<Pose3> optimal_trajectory;
  std::vector<geometry_msgs::PoseStamped> ros_optimal_trajectory;
  for (int i = 1; i < results.size(); ++i) {
    optimal_trajectory.push_back(results.at<Pose3>(i));
  }
  convertTrajectoryFromGtsamToRos(optimal_trajectory, &ros_optimal_trajectory);
  drawTrajectory(optimal_marker_pub, optimal_pose_array_pub,
                 ros_optimal_trajectory, color_optimal_trajectory);

  // ROS spin until killed.
  ros::spin();

  return 0;
}
