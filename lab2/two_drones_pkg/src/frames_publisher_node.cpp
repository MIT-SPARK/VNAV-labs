#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

class FramesPublisherNode {
 private:
  ros::NodeHandle nh;
  ros::Time startup_time;

  ros::Timer heartbeat;
  // *** FILL IN *** instantiate a transform broadcaster...

 public:
  FramesPublisherNode() {
    // NOTE: This method is run once, when the node is launched.
    startup_time = ros::Time::now();
    heartbeat =
        nh.createTimer(ros::Duration(0.02), &FramesPublisherNode::onPublish, this);
    heartbeat.start();
  }

  void onPublish(const ros::TimerEvent&) {
    // NOTE: This method is called at 50Hz, due to the timer created on line 16.

    // 1. Compute time elapsed in seconds since the node has been started
    //    i.e. the time elapsed since startup_time (defined on line 8)
    //   HINTS:
    //   - check out the ros::Time API at
    //     http://wiki.ros.org/roscpp/Overview/Time#Time_and_Duration
    //   - use the - (subtraction) operator between ros::Time::now() and startup_time
    //   - convert the resulting Duration to seconds, store result into a double

    // double time = *** FILL IN ***;

    // Here we declare two geometry_msgs::TransformStamped objects, which need to be
    // populated
    geometry_msgs::TransformStamped AV1World;
    geometry_msgs::TransformStamped AV2World;
    // NOTE: fields in a ros message default to zero, so we set an identity transform by
    //       setting just the w component of the rotation
    AV1World.transform.rotation.w = 1.0;
    AV2World.transform.rotation.w = 1.0;

    // 2. Populate the two transforms for the AVs, using the variable "time"
    //    computed above. Specifically:
    //     - AV1World should have origin in [cos(time), sin(time), 0.0] and
    //       rotation such that:
    //        i) its y axis stays tangent to the trajectory and
    //       ii) the z vector stays parallel to that of the world frame
    //     - AV1World should have frame_id "world" and child_frame_id "av1"
    //
    //     - AV2World shoud have origin in [sin(time), 0.0, cos(2*time)], the
    //       rotation is irrelevant to our purpose.
    //     - AV2World should have frame_id "world" and child_frame_id "av2"
    //    NOTE: AV1World's orientation is crucial for the rest fo the assignment,
    //          make sure you get it right
    //
    //    HINTS:
    //    1. check out the ROS tf2 Tutorials: http://wiki.ros.org/tf2/Tutorials,
    //      https://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29#The_Code
    //    2. consider the setRPY method on a tf2::Quaternion for AV1
    //    3. the frame names are crucial for the rest of the assignment,
    //       make sure they are as specified, "av1", "av2" and "world"

    // *** FILL IN ***

    // 3. Publish the transforms using a tf2_ros::TransformBroadcaster
    //    HINTS:
    //         1. you need to define a tf2_ros::TransformBroadcaster as a member of the
    //            node class (line 11) and use its sendTrasform method below

    // *** FILL IN ***
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "frames_publisher_node");
  FramesPublisherNode node;
  ros::spin();
  return 0;
}
