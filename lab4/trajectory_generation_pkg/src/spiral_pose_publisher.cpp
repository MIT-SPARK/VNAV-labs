//
// Created by stewart on 9/20/20.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "spiral_pose_publisher");
    ros::NodeHandle nh;

    ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseArray>("/desired_traj_vertices", 1);

    uint32_t const frequency = 4;
    uint32_t const n_loops = 4;
    double const start_height = 2;
    double const loop_radius = 3;
    double const loop_height = 2;

    uint32_t const n_poses = 1 + (frequency * n_loops);
    double const max_height = n_loops * loop_height;

    geometry_msgs::PoseArray poseArray;

    geometry_msgs::Pose start;
    start.position.x = loop_radius;
    start.position.z = start_height;
    poseArray.poses.push_back(start);

    for (uint32_t loop = 0; loop < n_loops; ++loop) {
        for (uint32_t angle_idx = 1; angle_idx <= frequency; ++angle_idx) {
            double const angle = 2 * M_PI * angle_idx / frequency;

            geometry_msgs::Pose next_pose;
            next_pose.position.x = std::cos(angle) * loop_radius;
            next_pose.position.y = std::sin(angle) * loop_radius;
            next_pose.position.z = start_height + static_cast<double>(1 + poseArray.poses.size()) / n_poses * max_height;
            poseArray.poses.push_back(next_pose);
        }
    }
    while (posePublisher.getNumSubscribers() == 0) {
        ros::spinOnce();
    }
    ROS_INFO("Publishing");
    posePublisher.publish(poseArray);
    return 0;
}
