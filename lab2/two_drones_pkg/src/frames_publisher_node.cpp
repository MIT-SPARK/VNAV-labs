#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

class FramesPublisherNode{
 ros::NodeHandle nh;
 ros::Time startup_time;

 ros::Timer heartbeat;
 // *** FILL IN *** instantiate a transform broadcaster...

 public:
  FramesPublisherNode(){
    // NOTE: This method is run once, when the node is launched.
    startup_time = ros::Time::now();
    heartbeat = nh.createTimer(ros::Duration(0.02), &FramesPublisherNode::onPublish, this);
    heartbeat.start();
  }

  void onPublish(const ros::TimerEvent&){
    // NOTE: This method is called at 50Hz, due to the timer created on line 16.

    //
    // 1. Compute time elapsed in seconds since the node has been started
    //    i.e. the time elapsed since startup_time (defined on line 8)
    //   HINTS:
    //   - check out the ros::Time API at
    //     http://wiki.ros.org/roscpp/Overview/Time#Time_and_Duration
    //   - use the - operator between ros::Time::now() and startup_time
    //   - convert the resulting Duration to seconds, store result into a double

    // double time = *** FILL IN ***;

    // Here we declare two tf::Transform objects, which need to be populated
    tf::Transform AV1World(tf::Transform::getIdentity());
    tf::Transform AV2World(tf::Transform::getIdentity());

    // 2. Populate the two transforms for the AVs, using the variable "time"
    //    computed above. Specifically:
    //     - AV1World should have origin in [cos(time), sin(time), 0.0] and
    //       rotation such that:
    //        i) its y axis stays tangent to the trajectory and
    //       ii) the z vector stays parallel to that of the world frame
    //
    //     - AV2World shoud have origin in [sin(time), 0.0, cos(2*time)], the
    //       rotation is irrelevant to our purpose.
    //    NOTE: AV1World's orientation is crucial for the rest fo the assignment,
    //          make sure you get it right
    //
    //    HINTS:
    //    - check out the ROS tf Tutorials: http://wiki.ros.org/tf/Tutorials,
    //      http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29#The_Code
    //    - consider the setRPY method on a tf::quaternion for AV1

    // *** FILL IN *** 

    // 3. Publish the transforms, namely:
    //     - AV1World with frame_id "world", child_frame_id "av1"
    //     - AV2World with frame_id "world", child_frame_id "av2"
    //    HINTS:
    //         1. you need to define a tf::TransformBroadcaster as a member of the
    //            node class (line 11) and use its sendTrasform method below
    //         2. the frame names are crucial for the rest of the assignment,
    //            make sure they are as specified, "av1", "av2" and "world"
    
    // *** FILL IN ***

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "frames_publisher_node");
  FramesPublisherNode node;
  ros::spin();
  return 0;
}
