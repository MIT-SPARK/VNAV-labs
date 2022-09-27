#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <sstream>

#include <math.h>

ros::Subscriber desire_traj_vertices_sub;
ros::Publisher desired_state_pub;
tf::TransformBroadcaster* br;

/**
 * Callback function for listening to the desired vertices msgs
 */
void trajCB(const geometry_msgs::PoseArray& traj_msg) {
  // sanity check for traj_msg size
  if (traj_msg.poses.size() == 0) {
    ROS_ERROR_THROTTLE(1, "Empty trajectory vertices msg.");
    return;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 0 |  16.485 - Fall 2020  - Lab 4 coding assignment  (10 pts)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  //  As a simple warm up exercise before we get to the actual 'real deal',
  //  let's just make our quadcopter fly to the first gate in the course.
  //  In this section:
  //   1. Extract the first vertex of the trajectory
  //   2. Set the acceleration and velocities to zero
  //   3. Publish the desired MultiDOFJointTrajectoryPoint
  //   4. Create and publish TF transform of the desired pose
  // ~~~~ begin solution

  //
  //
  //     **** FILL IN HERE ***
  //
  //

  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 0
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);
  ros::Time start(ros::Time::now());

  // deired traj vertices subscriber
  desire_traj_vertices_sub = n.subscribe("desired_traj_vertices", 10, trajCB);

  // publisher for desired states for controller
  desired_state_pub =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);

  br = new tf::TransformBroadcaster();

  ros::spin();
}
