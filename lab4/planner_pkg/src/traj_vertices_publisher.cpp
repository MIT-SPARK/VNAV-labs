#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <sstream>

#include <csv.h>
#include <math.h>

/**
 * Helper function to transform a geometry_msgs/Transform message from Unity
 * frame to ROS frame.
 *
 * Note: This has to be consistent with the transformation convention used in
 * tesse_ros_bridge
 */
geometry_msgs::Pose unity2RosPose(const geometry_msgs::Transform& unity_T) {
  geometry_msgs::Pose ros_T;

  // position: flip y & z
  ros_T.position.x = unity_T.translation.x;
  ros_T.position.y = unity_T.translation.z;
  ros_T.position.z = unity_T.translation.y;

  // rotation: flip y & z, take negative of vector
  ros_T.orientation.x = -unity_T.rotation.x;
  ros_T.orientation.y = -unity_T.rotation.z;
  ros_T.orientation.z = -unity_T.rotation.y;
  ros_T.orientation.w = unity_T.rotation.w;

  return ros_T;
}

/**
 * Helper function to load desired trajectory from simulator data
 */
geometry_msgs::PoseArray load_pose_array_from_simulator(
    std::string simulator_data_directory) {
  // filepath to the static tf csv file
  std::string static_tf_csv_file =
      simulator_data_directory +
      "/StreamingAssets/vnav-2020-lab4_static_tfs.csv";

  // create the appropriate CSV reader
  io::CSVReader<8> in(static_tf_csv_file);
  in.set_header("object_name", "x", "y", "z", "qx", "qy", "qz", "qw");

  // variable to save object name
  std::string object_name;
  double x, y, z, qx, qy, qz, qw;

  // traj vertex point with index
  // needs to be sorted before insertion into the trajectory message
  std::vector<std::pair<int, geometry_msgs::Pose>> traj_vertices_with_index;

  std::string door_name = "red_square_drone_door";

  while (in.read_row(object_name, x, y, z, qx, qy, qz, qw)) {
    // the drone racing door are of names in the format:
    // red_sqaure_drone_door (#) where # is the index
    // of the door on the sequence.

    // split name by empty space
    std::istringstream iss(object_name);
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss),
              std::istream_iterator<std::string>(),
              std::back_inserter(tokens));

    if (tokens[0].compare(door_name) == 0) {
      if (tokens.size() == 1) {
        tokens.push_back("0");
      } else {
        // remove the parentheses
        tokens[1].erase(0, 1);
        tokens[1].pop_back();
      }

      // load the xyz and quaternion to msg
      geometry_msgs::Transform unity_T;
      unity_T.translation.x = x;
      unity_T.translation.y = y;
      unity_T.translation.z = z;
      unity_T.rotation.x = qx;
      unity_T.rotation.y = qy;
      unity_T.rotation.z = qz;
      unity_T.rotation.w = qw;

      // transform to ROS frame
      geometry_msgs::Pose ros_T = unity2RosPose(unity_T);

      // load the vertex with point into vector for sorting
      // create the pair to hold the current trajectory point
      std::pair<int, geometry_msgs::Pose> traj_vertex_with_index;
      traj_vertex_with_index.first = std::stoi(tokens[1]);
      traj_vertex_with_index.second = ros_T;
      traj_vertices_with_index.push_back(traj_vertex_with_index);
    }
  }

  // sort the vertices and insert them into the final trajectory
  std::sort(
      traj_vertices_with_index.begin(),
      traj_vertices_with_index.end(),
      [](std::pair<int, geometry_msgs::Pose> a,
         std::pair<int, geometry_msgs::Pose> b) { return a.first < b.first; });

  geometry_msgs::PoseArray traj_msg;
  for (const auto& vertex_with_index : traj_vertices_with_index) {
    traj_msg.poses.push_back(vertex_with_index.second);
  }

  ROS_INFO_STREAM("Finished loading trajectory csv: " << traj_msg.poses.size());
  return traj_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_vertices_publisher");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(5);

  // trajectory vertices publisher
  ros::Publisher traj_vertices_pub =
      n.advertise<geometry_msgs::PoseArray>("/desired_traj_vertices", 1);

  // load simulator data directory
  std::string simulator_data_directory;
  if (n.getParam("simulator_data_directory", simulator_data_directory)) {
    ROS_INFO_STREAM(
        "Simulator data directory loaded at:" << simulator_data_directory);
  } else {
    ROS_ERROR("Cannot find simulator_data_directory param.");
    return -1;
  }

  geometry_msgs::PoseArray msg =
      load_pose_array_from_simulator(simulator_data_directory);

  if (msg.poses.size() == 0) {
    ROS_ERROR_THROTTLE(5, "Empty trajectory msg.");
    ros::shutdown();
  }
  while (ros::ok()) {
    traj_vertices_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
