#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <sstream>
#include <iostream>

#include <math.h>

#define STATIC_POSE 0

#define PI M_PI

#define TFOUTPUT 1

class TrajPublisherNode : public rclcpp::Node
{
    public:
    TrajPublisherNode() : Node("traj_publisher_node") {
        rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_pub =
            this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("desired_state", 1);
        rclcpp::Rate loop_rate(100);
        auto clock = this->get_clock();
        rclcpp::Time start = clock->now();
        
#if TFOUTPUT
        tf2_ros::TransformBroadcaster br(this);
#endif

        int count = 0;
        while (rclcpp::ok()) {
            tf2::Vector3 origin(0,0,0);

            double t = (clock->now() - start).seconds();
            

            // Quantities to fill in
            geometry_msgs::msg::TransformStamped desired_pose;
            //desired_pose.header.stamp = ros::Time::now();
            desired_pose.header.frame_id = "world";
            desired_pose.child_frame_id = "av-desired";

            geometry_msgs::msg::Twist velocity;
            velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
            velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
            geometry_msgs::msg::Twist acceleration;
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
            //std::cout<<"Desired Orientation" << count << std::endl;

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
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg;
            msg.transforms.resize(1);
            msg.transforms[0] = desired_pose.transform;
            msg.velocities.resize(1);
            msg.velocities[0] = velocity;
            msg.accelerations.resize(1);
            msg.accelerations[0] = acceleration;
            desired_state_pub->publish(msg);

            std::stringstream ss;
            ss << "Trajectory Position"
            << " x:" << desired_pose.transform.translation.x
            << " y:" << desired_pose.transform.translation.y
            << " z:" << desired_pose.transform.translation.z;
            //RCLCPP_INFO(this->get_logger(),"%s", ss.str().c_str());

#if TFOUTPUT
            br.sendTransform(desired_pose);
#endif

            loop_rate.sleep();
            ++count;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                               // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<TrajPublisherNode>());    // Spin the node so it processes callbacks
    rclcpp::shutdown();                                     // Shutdown the ROS 2 system when done
    return 0;
}