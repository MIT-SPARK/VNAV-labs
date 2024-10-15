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

#include <geometry_msgs/msg/detail/pose_array__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <random>

// ROS headers.
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Custom MoCap factor.
#include "deliverable_2_3.h"
#include "trajectory_color.h"

// GTSAM headers.
#include <glog/logging.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

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

// Convenience function to convert a gtsam Pose to a ROS Pose for visualization.
void gtsamPoseToRosPose(const gtsam::Pose3& gtsam_pose,
                        geometry_msgs::msg::PoseStamped* ros_pose) {
  CHECK_NOTNULL(ros_pose);
  ros_pose->pose.position.x = gtsam_pose.x();
  ros_pose->pose.position.y = gtsam_pose.y();
  ros_pose->pose.position.z = gtsam_pose.z();
  ros_pose->pose.orientation.w = gtsam_pose.rotation().toQuaternion().w();
  ros_pose->pose.orientation.x = gtsam_pose.rotation().toQuaternion().x();
  ros_pose->pose.orientation.y = gtsam_pose.rotation().toQuaternion().y();
  ros_pose->pose.orientation.z = gtsam_pose.rotation().toQuaternion().z();
}

// Convenience function to convert a trajectory using gtsam Pose
// to a ROS trajectory using ROS Pose for visualization.
void convertTrajectoryFromGtsamToRos(
    const std::vector<gtsam::Pose3>& gtsam_traj,
    std::vector<geometry_msgs::msg::PoseStamped>* ros_traj) {
  CHECK_NOTNULL(ros_traj);
  geometry_msgs::msg::PoseStamped ros_pose;
  for (const gtsam::Pose3& gtsam_pose : gtsam_traj) {
    gtsamPoseToRosPose(gtsam_pose, &ros_pose);
    ros_traj->push_back(ros_pose);
  }
}

// Generates odometry measurements by creating a trajectory, corrupting it with
// noise and returning relative poses between consecutive points.
// It also returns the noisy trajectory from which the measurements were taken.
// The ground-truth trajectory is also returned for plotting.
// The last two arguments specify the level of noise that corrupts the
// measurements.
void generateOdometryMeasurements(const int& N_measurements,
                                  std::vector<Pose3>* measurements,
                                  std::vector<Pose3>* noisy_trajectory = nullptr,
                                  std::vector<Pose3>* gt_trajectory = nullptr,
                                  const double& rot_std_dev = 1e-6,
                                  const double& pos_std_dev = 1e-4) {
  CHECK_NOTNULL(measurements);
  measurements->clear();
  static constexpr double R = 2;

  // Generate ground_truth trajectory.
  std::vector<Pose3> gt_traj;
  for (double i = 0; i < (double)N_measurements; i++) {
    gt_traj.push_back(Pose3(Rot3().Yaw(4.0 * M_PI / N_measurements * i),
                            Point3(R * std::cos(4.0 * M_PI / N_measurements * i),
                                   R * std::sin(4.0 * M_PI / N_measurements * i),
                                   i / N_measurements)));
  }

  // Return the gt_trajectory if requested.
  if (gt_trajectory != nullptr) {
    *gt_trajectory = gt_traj;
  }

  // Generate measurements by adding noise to trajectory.
  Pose3 pose = gt_trajectory->at(0);
  Vector6 e;
  // for (const Pose3 &gt_pose : *gt_trajectory) {
  std::vector<Pose3> noisy_traj;
  noisy_traj.push_back(pose);

  for (size_t i = 0; i < gt_trajectory->size() - 1; ++i) {
    Pose3 delta = gt_trajectory->at(i).inverse().compose(gt_trajectory->at(i + 1));
    std::normal_distribution<> d_rotation_yaw{delta.rotation().yaw(), rot_std_dev};
    std::normal_distribution<> d_rotation_pitch{delta.rotation().pitch(), rot_std_dev};
    std::normal_distribution<> d_rotation_roll{delta.rotation().roll(), rot_std_dev};
    std::normal_distribution<> d_position_x{delta.translation().x(), pos_std_dev};
    std::normal_distribution<> d_position_y{delta.translation().y(), pos_std_dev};
    std::normal_distribution<> d_position_z{delta.translation().z(), pos_std_dev};
    e << d_rotation_roll(gen_), d_rotation_pitch(gen_), d_rotation_yaw(gen_),
        d_position_x(gen_), d_position_y(gen_), d_position_z(gen_);
    measurements->push_back(Pose3::Expmap(e));
    noisy_traj.push_back(noisy_traj.back().expmap(e));
  }

  // Return the noisy_trajectory if requested.
  if (noisy_trajectory != nullptr) {
    *noisy_trajectory = noisy_traj;
  }
}

// mocap_measurements is a vector containing pairs, where the first element
// of the pair represents the index of the pose it is giving information about,
// while the second element provides the measured position of such pose.
void generateMoCapMeasurements(const int& N_measurements,
                               const std::vector<Pose3>& gt_trajectory,
                               std::vector<std::pair<int, Point3>>* mocap_measurements,
                               const double& pos_std_dev = 1e-4) {
  CHECK_NOTNULL(mocap_measurements);
  CHECK_LE(N_measurements, gt_trajectory.size());
  // Generate measurements by adding noise to trajectory.
  static std::normal_distribution<> d_position{0, pos_std_dev};
  int step = std::floor(gt_trajectory.size() / N_measurements);
  for (int i = 0; i < gt_trajectory.size(); i = i + step) {
    mocap_measurements->push_back(
        std::make_pair(i + 1,
                       Point3(gt_trajectory.at(i).x() + d_position(gen_),
                              gt_trajectory.at(i).y() + d_position(gen_),
                              gt_trajectory.at(i).z() + d_position(gen_))));
  }
}

class MocapPose3Node : public rclcpp::Node {
 public:
  size_t max_solver_iterations_;
  bool use_mocap_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gt_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr gt_pose_array_pub_;

  // ROS publishers for noisy trajectory.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  // ROS publishers for initial trajectory.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr initial_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr initial_pose_array_pub_;

  // ROS publishers for optimized trajectory.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optimal_pose_array_pub_;

  MocapPose3Node() : Node("mocap_pose3_node") {
    rclcpp::QoS latch = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", latch);

    // ROS publishers for ground-truth trajectory.

    gt_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/gt_trajectory_lines", latch);

    gt_pose_array_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>("/gt_trajectory", latch);

    // ROS publishers for noisy trajectory.
    marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("/trajectory_lines", latch);

    pose_array_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>("/trajectory", latch);

    // ROS publishers for initial trajectory.
    initial_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/initial_trajectory_lines", latch);

    initial_pose_array_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>("/initial_trajectory", latch);

    // ROS publishers for optimized trajectory.
    optimal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/optimal_trajectory_lines", latch);
    optimal_pose_array_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>("/optimal_trajectory", latch);
  }

  void run() {
    declare_parameter<long>("max_solver_iterations");
    declare_parameter<bool>("use_mocap");
    get_parameter("max_solver_iterations", max_solver_iterations_);
    get_parameter("use_mocap", use_mocap_);

    // Simulate noisy odometry measurements for a robot trajectory in 3D.
    // Store as well the noisy and ground-truth trajectory for later plotting.
    // Hint: to allow easy debugging, try first to generate little measurements
    // (5-10).
    std::vector<Pose3> measurements;
    std::vector<Pose3> noisy_trajectory;
    std::vector<Pose3> gt_trajectory;
    static constexpr int num_measurements = 500;
    static constexpr double rot_std_dev = 1e-2;
    static constexpr double pos_std_dev = 1e-2;
    generateOdometryMeasurements(num_measurements,
                                 &measurements,
                                 &noisy_trajectory,
                                 &gt_trajectory,
                                 rot_std_dev,
                                 pos_std_dev);

    // Simulate motion capture measurements of the actual position of the drone.
    static constexpr int num_mocap_measurements = 10;
    static constexpr double mocap_std_dev = 1e-10;
    std::vector<std::pair<int, Point3>> mocap_measurements;
    if (use_mocap_) {
      generateMoCapMeasurements(
          num_mocap_measurements, gt_trajectory, &mocap_measurements, mocap_std_dev);
    }

    // Convert trajectory to ROS type.
    std::vector<geometry_msgs::msg::PoseStamped> ros_noisy_trajectory;
    convertTrajectoryFromGtsamToRos(noisy_trajectory, &ros_noisy_trajectory);

    // Convert trajectory to ROS type.
    std::vector<geometry_msgs::msg::PoseStamped> gt_ros_trajectory;
    convertTrajectoryFromGtsamToRos(gt_trajectory, &gt_ros_trajectory);

    // Turn ROS at 100 Hz.
    rclcpp::Rate rate(100);
    std::string world_frame = "world";
    // Draw noisy and ground-truth trajectories and visulize them in Rviz.
    // When running this node, open rviz either using the configuration file
    // in the rviz folder of this lab, or by adding the topics using Rviz GUI.
    drawTrajectory(
        gt_marker_pub_, gt_pose_array_pub_, gt_ros_trajectory, color_gt_trajectory);
    drawTrajectory(
        marker_pub_, pose_array_pub_, ros_noisy_trajectory, color_noisy_trajectory);

    // Publish the motion of the robot over the noisy trajectory to have a sense
    // of how corrupted are the measurements observed.
    for (size_t i = 0; i < ros_noisy_trajectory.size(); ++i) {
      ros_noisy_trajectory.at(i).header.frame_id = world_frame;
      ros_noisy_trajectory.at(i).header.stamp = now();
      pose_pub_->publish(ros_noisy_trajectory.at(i));
      i++;
      rate.sleep();
    }

    // Create a factor graph container.
    NonlinearFactorGraph graph;

    // Create an Odometry measurement noise model (covariance matrix).
    Vector6 sigmas;
    sigmas << rot_std_dev, rot_std_dev, rot_std_dev, pos_std_dev, pos_std_dev,
        pos_std_dev;
    const noiseModel::Diagonal::shared_ptr odometryNoise =
        noiseModel::Diagonal::Sigmas(sigmas);

    // 2a.
    // TODO: Add factors to the graph.
    // Create BetweenFactor(s) between consecutive poses using the odometry
    // measurements.
    // Start of 2a

    // End 2a.

    // 2b.
    if (!use_mocap_) {
      Matrix init_pose = Matrix::Identity(4, 4);
      init_pose(0, 3) = 2;
      gtsam::Pose3 initial_pose(init_pose);
      const noiseModel::Diagonal::shared_ptr initialNoise =
          noiseModel::Diagonal::Sigmas(sigmas);
      // TODO: Add a prior factor on the node with key 1
      // to constrain it to initial_pose
      // Start of 2b.

      // End 2b.
    }

    if (use_mocap_) {
      // 3b. Add motion capture measurements (mocap_measurements)
      // TODO: create the 3D MoCap measurement noise model (a diagonal noise
      // model should be sufficient). Think of how many dimensions it should
      // have. You are given mocap_std_dev (defined above)
      // Start of 3b.

      //  TODO: add the MoCap factors
      // Note that there is no prior factor needed at first pose, since MoCap
      // provides the global positions (and rotations given more than 1 MoCap
      // measurement).

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
      const auto p =
          Pose3(Rot3().Yaw(3.0 * M_PI / measurements.size() * (float)i),
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
    std::vector<geometry_msgs::msg::PoseStamped> ros_initial_trajectory;
    convertTrajectoryFromGtsamToRos(initial_trajectory, &ros_initial_trajectory);
    drawTrajectory(initial_marker_pub_,
                   initial_pose_array_pub_,
                   ros_initial_trajectory,
                   color_initial_trajectory);

    // Print initial values for debugging if you want.
    // initials.print("\nInitial Values:\n");

    // Use Gauss-Newton method to optimize the initial values
    GaussNewtonParams parameters;

    // Modify the max number of iterations to see the overall improvement.
    // You can set max_solver_iterations_ as a parameter in the ros2 launch
    // command
    parameters.setMaxIterations(max_solver_iterations_);
    parameters.setAbsoluteErrorTol(1e-06);

    // Print per iteration.
    parameters.setVerbosity("ERROR");

    // Create the optimizer GaussNewtonOptimizer given the parameters
    // above , the initial values and the graph.
    GaussNewtonOptimizer optimizer(graph, initial, parameters);

    // Finally, Optimize!
    Values results = optimizer.optimize();

    // Print final values for debugging if you want.
    //  results.print("Final Result:\n");

    std::cout << "\e[1mOptimization finished.\e[0m" << std::endl;

    // Convert optimal trajectory to ROS type and plot optimized
    // trajectory in Rviz. To access the results use:
    // results.at<Pose3>(Symbol('x', key)), where key is the id of the pose for
    // which you want the result.
    std::vector<Pose3> optimal_trajectory;
    std::vector<geometry_msgs::msg::PoseStamped> ros_optimal_trajectory;
    for (int i = 1; i < results.size(); ++i) {
      optimal_trajectory.push_back(results.at<Pose3>(i));
    }
    convertTrajectoryFromGtsamToRos(optimal_trajectory, &ros_optimal_trajectory);
    drawTrajectory(optimal_marker_pub_,
                   optimal_pose_array_pub_,
                   ros_optimal_trajectory,
                   color_optimal_trajectory);
  }

  // Draws a trajectory in Rviz, by publishing a set of lines and poses using
  // the publishers passed as argument and the actual trajectory to plot.
  // You can also pass what color you want the trajectory in, and the frame_id
  // were the Rviz Markers should be plotted (generally set to 'world).
  void drawTrajectory(
      const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& marker_pub,
      const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr& pose_array_pub,
      const std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
      const TrajectoryColor& trajectory_color = TrajectoryColor(),
      const std::string& frame_id = "world") {
    // Create the vertices for the points and lines
    CHECK_GE(trajectory.size(), 0);
    geometry_msgs::msg::PoseStamped prev_i = trajectory.at(0);

    // Create visual markers.
    visualization_msgs::msg::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = frame_id;
    points.header.stamp = line_strip.header.stamp = now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::msg::Marker::POINTS;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

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
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = now();
    pose_array.header.frame_id = frame_id;

    // Loop over the trajectory.
    for (const geometry_msgs::msg::PoseStamped& i : trajectory) {
      double t = rclcpp::Time(i.header.stamp).seconds();
      double t_last = rclcpp::Time(prev_i.header.stamp).seconds();
      if (t < t_last) {
        LOG(WARNING) << "Stamp of poses in trajectory should be increasing in value.\n"
                     << "Got timestamp #1: " << t << "\n"
                     << "Got timestamp #2: " << t_last;
      }
      prev_i = i;
      geometry_msgs::msg::Point p;
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
    pose_array_pub->publish(pose_array);

    // Publish lines and points.
    marker_pub->publish(points);
    marker_pub->publish(line_strip);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<MocapPose3Node>();
  node->run();
  rclcpp::spin(node);

  return 0;
}
