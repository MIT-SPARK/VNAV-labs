#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {
        tf::Vector3 origin(0,0,0);

        double t = (ros::Time::now()-start).toSec();

        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

#if STATIC_POSE
        // Static Pose
        tf::Vector3 displacement(0,0,2);
        desired_pose.setOrigin(origin+displacement);
        tf::Quaternion q;
        q.setRPY(0,0,PI/4);
        count++;
        std::cout<<"Desired Orientation" << count << std::endl;
        desired_pose.setRotation(q);

#else
        // Circle
        double R = 5.0;
        double timeScale = 2.0;
        desired_pose.setOrigin(
                        origin + tf::Vector3(R*sin(t/timeScale), R*cos(t/timeScale), 2)
                     );
        tf::Quaternion q;
        q.setRPY(0,0,-t/timeScale);
        desired_pose.setRotation(q);
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
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z();
        ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
