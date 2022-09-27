#pragma once

// GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

// ROS headers.
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include "trajectory_color.h"

namespace marker_color {
TrajectoryColor color_gt(PointColor(0.0, 1.0, 0.0, 1.0),
                         LineColor(0.0, 1.0, 0.0, 1.0));
TrajectoryColor color_optimal(PointColor(0.0, 0.0, 1.0, 1.0),
                              LineColor(0.0, 0.0, 1.0, 1.0));
TrajectoryColor color_init(PointColor(1.0, 0.0, 0.0, 1.0),
                           LineColor(1.0, 0.0, 0.0, 1.0));
}  // namespace marker_color

void DrawPoint3(const ros::Publisher& marker_pub,
                std::vector<gtsam::Point3>& landmarks,
                const TrajectoryColor& color = TrajectoryColor()) {
  static constexpr char kFrameId[] = "world";

  visualization_msgs::Marker landmark_markers;
  landmark_markers.header.frame_id = kFrameId;
  landmark_markers.header.stamp = ros::Time::now();
  landmark_markers.ns = "landmark";
  landmark_markers.action = visualization_msgs::Marker::ADD;
  landmark_markers.pose.orientation.w = 1.0;
  landmark_markers.id = 0;
  landmark_markers.type = visualization_msgs::Marker::POINTS;

  landmark_markers.scale.x = 0.1;
  landmark_markers.scale.y = 0.1;

  // landmark_markers
  landmark_markers.color.r = color.point_color_.r_;
  landmark_markers.color.g = color.point_color_.g_;
  landmark_markers.color.b = color.point_color_.b_;
  landmark_markers.color.a = color.point_color_.a_;

  // Loop over the trajectory.
  for (const auto& landmark : landmarks) {
    geometry_msgs::Point pt;
    pt.x = landmark.x();
    pt.y = landmark.y();
    pt.z = landmark.z();
    landmark_markers.points.push_back(pt);
  }

  // Publish
  marker_pub.publish(landmark_markers);
}

void DrawPoses3(const ros::Publisher& pose_array_pub,
                std::vector<gtsam::Pose3>& poses) {
  static constexpr char kFrameId[] = "world";
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = kFrameId;

  for (const auto& pose : poses) {
    geometry_msgs::PoseStamped ros_pose;
    ros_pose.pose.position.x = pose.x();
    ros_pose.pose.position.y = pose.y();
    ros_pose.pose.position.z = pose.z();
    ros_pose.pose.orientation.w = pose.rotation().quaternion()(0);
    ros_pose.pose.orientation.x = pose.rotation().quaternion()(1);
    ros_pose.pose.orientation.y = pose.rotation().quaternion()(2);
    ros_pose.pose.orientation.z = pose.rotation().quaternion()(3);

    pose_array.poses.push_back(ros_pose.pose);
  }

  // Publish poses.
  pose_array_pub.publish(pose_array);
}