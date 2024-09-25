#include <csv.h> // Ensure you have a CSV library
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <iostream>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

class TrajVerticesPublisher : public rclcpp::Node {
public:
  TrajVerticesPublisher() : Node("traj_vertices_publisher") {
    // Declare parameters
    declare_parameter<std::string>("simulator_data_directory", "");

    // Create the publisher
    traj_vertices_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/desired_traj_vertices", 1);

    // Load simulator data directory
    get_parameter("simulator_data_directory", simulator_data_directory_);

    if (simulator_data_directory_.empty()) {
      RCLCPP_ERROR(get_logger(), "Cannot find simulator_data_directory param.");
      rclcpp::shutdown();
      return;
    }

    // Load the pose array from the simulator
    geometry_msgs::msg::PoseArray msg =
        load_pose_array_from_simulator(simulator_data_directory_);

    if (msg.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty trajectory msg.");
      rclcpp::shutdown();
      return;
    }

    // Publish the message in a loop
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200), [this, msg]() {
          traj_vertices_pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(),
                      "Published trajectory with %zu poses.", msg.poses.size());
        });
  }

private:
  std::string simulator_data_directory_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      traj_vertices_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose
  unity2RosPose(const geometry_msgs::msg::Transform &unity_T) {
    geometry_msgs::msg::Pose ros_T;
    ros_T.position.x = unity_T.translation.x;
    ros_T.position.y = unity_T.translation.z; // Flip y & z
    ros_T.position.z = unity_T.translation.y;

    ros_T.orientation.x = -unity_T.rotation.x; // Flip y & z
    ros_T.orientation.y = -unity_T.rotation.z;
    ros_T.orientation.z = -unity_T.rotation.y;
    ros_T.orientation.w = unity_T.rotation.w;

    return ros_T;
  }

  geometry_msgs::msg::PoseArray
  load_pose_array_from_simulator(const std::string &simulator_data_directory) {
    std::string static_tf_csv_file =
        simulator_data_directory +
        "/StreamingAssets/vnav-2020-lab4_static_tfs.csv";

    io::CSVReader<8> in(static_tf_csv_file);
    in.set_header("object_name", "x", "y", "z", "qx", "qy", "qz", "qw");

    std::string object_name;
    double x, y, z, qx, qy, qz, qw;
    std::vector<std::pair<int, geometry_msgs::msg::Pose>>
        traj_vertices_with_index;
    std::string door_name = "red_square_drone_door";

    while (in.read_row(object_name, x, y, z, qx, qy, qz, qw)) {
      std::istringstream iss(object_name);
      std::vector<std::string> tokens;
      std::copy(std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>(),
                std::back_inserter(tokens));

      if (tokens[0].compare(door_name) == 0) {
        if (tokens.size() == 1) {
          tokens.push_back("0");
        } else {
          tokens[1].erase(0, 1);
          tokens[1].pop_back();
        }

        geometry_msgs::msg::Transform unity_T;
        unity_T.translation.x = x;
        unity_T.translation.y = y;
        unity_T.translation.z = z;
        unity_T.rotation.x = qx;
        unity_T.rotation.y = qy;
        unity_T.rotation.z = qz;
        unity_T.rotation.w = qw;

        geometry_msgs::msg::Pose ros_T = unity2RosPose(unity_T);
        traj_vertices_with_index.emplace_back(std::stoi(tokens[1]), ros_T);
      }
    }

    // Sort the vertices
    std::sort(traj_vertices_with_index.begin(), traj_vertices_with_index.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });

    geometry_msgs::msg::PoseArray traj_msg;
    for (const auto &vertex_with_index : traj_vertices_with_index) {
      traj_msg.poses.push_back(vertex_with_index.second);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Finished loading trajectory csv: "
                                               << traj_msg.poses.size());
    return traj_msg;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajVerticesPublisher>());
  rclcpp::shutdown();
  return 0;
}
