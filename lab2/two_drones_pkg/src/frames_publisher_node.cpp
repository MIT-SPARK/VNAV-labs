#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

class FramesPublisherNode : public rclcpp::Node {
  rclcpp::Time startup_time;

  rclcpp::TimerBase::SharedPtr heartbeat;

  // TODO: Declare a std::unique_ptr<tf2_ros::TransformBroadcaster>
  // ...

 public:
  FramesPublisherNode() : Node("frames_publisher_node") {
    // NOTE: This method is run once, when the node is launched.

    // TODO: Instantiate the Transform Broadcaster
    // ....

    startup_time = now();
    heartbeat = create_timer(this,
                             get_clock(),
                             rclcpp::Duration::from_seconds(0.02),
                             std::bind(&FramesPublisherNode::onPublish, this));
    heartbeat->reset();
  }

  void onPublish() {
    // NOTE: This method is called at 50Hz, due to the timer created on line 25.

    //
    // 1. TODO: Compute time elapsed in seconds since the node has been started
    //    i.e. the time elapsed since startup_time (defined on line 13)
    //   HINTS:
    //   - Get the current time with this->now()
    //   - use the - operator between the current time and startup_time
    //   - convert the resulting Duration to seconds, store result into a double

    // ...

    // Here we declare two geometry_msgs::msg::TransformStamped objects, which
    // need to be populated
    geometry_msgs::msg::TransformStamped world_T_av1;
    geometry_msgs::msg::TransformStamped world_T_av2;

    // 2. TODO: Populate the two transforms for the AVs, using the variable "time"
    //    computed above. Specifically:
    //     - world_T_av1 should have origin in [cos(time), sin(time), 0.0] and
    //       rotation such that:
    //        i) its y axis stays tangent to the trajectory and
    //       ii) the z vector stays parallel to that of the world frame
    //
    //     - world_T_av2 shoud have origin in [sin(time), 0.0, cos(2*time)], the
    //       rotation is irrelevant to our purpose.
    //    NOTE: world_T_av1's orientation is crucial for the rest fo the
    //    assignment,
    //          make sure you get it right
    //
    //    HINTS:
    //    - check out the ROS tf2 Tutorials:
    //    https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
    //    - consider the setRPY method on a tf2::Quaternion for world_T_av1

    // ...

    // 3. TODO: Publish the transforms, namely:
    //     - world_T_av1 with frame_id "world", child_frame_id "av1"
    //     - world_T_av2 with frame_id "world", child_frame_id "av2"
    //    HINTS:
    //         1. you need to define a
    //         std::unique_ptr<tf2_ros::TransformBroadcaster> as a member of the
    //            node class (line 18) and use its sendTransform method below
    //         2. the frame names are crucial for the rest of the assignment,
    //            make sure they are as specified, "av1", "av2" and "world"

    // ...
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramesPublisherNode>());
  return 0;
}
