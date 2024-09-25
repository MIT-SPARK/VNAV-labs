#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class SpiralPublisherNode : public rclcpp::Node {

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub;

public:
  SpiralPublisherNode() : Node("spiral_publisher_node") {

    pose_pub = create_publisher<geometry_msgs::msg::PoseArray>(
        "desired_traj_vertices", 1);

    uint32_t const frequency = 4;
    uint32_t const n_loops = 4;
    double const start_height = 2;
    double const loop_radius = 3;
    double const loop_height = 2;
    uint32_t const n_poses = 1 + (frequency * n_loops);
    double const max_height = n_loops * loop_height;

    geometry_msgs::msg::PoseArray poseArray;

    geometry_msgs::msg::Pose start;
    start.position.x = loop_radius;
    start.position.z = start_height;
    poseArray.poses.push_back(start);

    for (uint32_t loop = 0; loop < n_loops; ++loop) {
      for (uint32_t angle_idx = 1; angle_idx <= frequency; ++angle_idx) {
        double const angle = 2 * M_PI * angle_idx / frequency;

        geometry_msgs::msg::Pose next_pose;
        next_pose.position.x = std::cos(angle) * loop_radius;
        next_pose.position.y = std::sin(angle) * loop_radius;
        next_pose.position.z =
            start_height + static_cast<double>(1 + poseArray.poses.size()) /
                               n_poses * max_height;
        poseArray.poses.push_back(next_pose);
      }
    }

    pose_pub->publish(poseArray);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpiralPublisherNode>());
  return 0;
}
