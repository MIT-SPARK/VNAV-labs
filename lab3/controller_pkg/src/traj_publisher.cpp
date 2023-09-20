#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define STATIC_POSE 0

#define PI M_PI

#define TFOUTPUT 1
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Rate loop_rate(100);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf2_ros::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {
        tf2::Vector3 origin(0,0,0);

        double t = (ros::Time::now()-start).toSec();

        // Quantities to fill in
        geometry_msgs::TransformStamped desired_pose;
        desired_pose.header.stamp = ros::Time::now();
        desired_pose.header.frame_id = "world";
        desired_pose.child_frame_id = "av-desired";

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

#if STATIC_POSE
        // Static Pose
        tf2::Vector3 displacement(0,0,2);
        tf2::Vector3 trans = origin + displacement;

        desired_pose.transform.translation.x = trans.x();
        desired_pose.transform.translation.y = trans.y();
        desired_pose.transform.translation.z = trans.z();

        tf2::Quaternion q;
        q.setRPY(0,0,PI/4);
        count++;
        std::cout<<"Desired Orientation" << count << std::endl;

        desired_pose.transform.rotation.x = q.x();
        desired_pose.transform.rotation.y = q.y();
        desired_pose.transform.rotation.z = q.z();
        desired_pose.transform.rotation.w = q.w();
#else
        // Circle
        double R = 5.0;
        double timeScale = 2.0;
        tf2::Vector3 trans = origin + tf2::Vector3(R*sin(t/timeScale), R*cos(t/timeScale), 2);
        desired_pose.transform.translation.x = trans.x();
        desired_pose.transform.translation.y = trans.y();
        desired_pose.transform.translation.z = trans.z();

        tf2::Quaternion q;
        q.setRPY(0,0,-t/timeScale);
        desired_pose.transform.rotation.x = q.x();
        desired_pose.transform.rotation.y = q.y();
        desired_pose.transform.rotation.z = q.z();
        desired_pose.transform.rotation.w = q.w();
        velocity.linear.x = R*cos(t/timeScale)/timeScale;
        velocity.linear.y = -R*sin(t/timeScale)/timeScale;
        velocity.linear.z = 0;

        velocity.angular.x = 0.0;
        velocity.angular.y = 0.0;
        velocity.angular.z = -1.0/timeScale;

        acceleration.linear.x = -R*sin(t/timeScale)/timeScale/timeScale;
        acceleration.linear.y = -R*cos(t/timeScale)/timeScale/timeScale;
        acceleration.linear.z = 0;
#endif

        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0] = desired_pose.transform;
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.transform.translation.x
           << " y:" << desired_pose.transform.translation.y
           << " z:" << desired_pose.transform.translation.z;
        ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(desired_pose);
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
