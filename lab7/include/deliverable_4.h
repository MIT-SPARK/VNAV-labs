#pragma once

// GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

// ROS headers.
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "trajectory_color.h"

namespace marker_color {
TrajectoryColor color_gt(PointColor(0.0, 1.0, 0.0, 1.0),
                         LineColor(0.0, 1.0, 0.0, 1.0));
TrajectoryColor color_optimal(PointColor(0.0, 0.0, 1.0, 1.0),
                              LineColor(0.0, 0.0, 1.0, 1.0));
TrajectoryColor color_init(PointColor(1.0, 0.0, 0.0, 1.0),
                           LineColor(1.0, 0.0, 0.0, 1.0));
} // namespace marker_color

void DrawPoint3(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        &marker_pub,
    const rclcpp::Time time, const std::vector<gtsam::Point3> &landmarks,
    const TrajectoryColor &color = TrajectoryColor()) {
  static constexpr char kFrameId[] = "world";

  visualization_msgs::msg::Marker landmark_markers;
  landmark_markers.header.frame_id = kFrameId;
  landmark_markers.header.stamp = time;
  landmark_markers.ns = "landmark";
  landmark_markers.action = visualization_msgs::msg::Marker::ADD;
  landmark_markers.pose.orientation.w = 1.0;
  landmark_markers.id = 0;
  landmark_markers.type = visualization_msgs::msg::Marker::POINTS;

  landmark_markers.scale.x = 0.1;
  landmark_markers.scale.y = 0.1;

  // landmark_markers
  landmark_markers.color.r = color.point_color_.r_;
  landmark_markers.color.g = color.point_color_.g_;
  landmark_markers.color.b = color.point_color_.b_;
  landmark_markers.color.a = color.point_color_.a_;

  // Loop over the trajectory.
  for (const auto &landmark : landmarks) {
    geometry_msgs::msg::Point pt;
    pt.x = landmark.x();
    pt.y = landmark.y();
    pt.z = landmark.z();
    landmark_markers.points.push_back(pt);
  }

  // Publish
  marker_pub->publish(landmark_markers);
}

void DrawPoses3(
    const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        &pose_array_pub,
    const rclcpp::Time time, const std::vector<gtsam::Pose3> &poses) {
  static constexpr char kFrameId[] = "world";
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = time;
  pose_array.header.frame_id = kFrameId;

  for (const auto &pose : poses) {
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.pose.position.x = pose.x();
    ros_pose.pose.position.y = pose.y();
    ros_pose.pose.position.z = pose.z();
    ros_pose.pose.orientation.w = pose.rotation().toQuaternion().w();
    ros_pose.pose.orientation.x = pose.rotation().toQuaternion().x();
    ros_pose.pose.orientation.y = pose.rotation().toQuaternion().y();
    ros_pose.pose.orientation.z = pose.rotation().toQuaternion().z();

    pose_array.poses.push_back(ros_pose.pose);
  }

  // Publish poses.
  pose_array_pub->publish(pose_array);
}
