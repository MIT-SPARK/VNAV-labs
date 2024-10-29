#include "rclcpp/rclcpp.hpp"
#include <ultralytics_ros/msg/yolo_result.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

#include "deliverable_1.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode(): Node("my_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
            "/yolo_result", 1, std::bind(&MyNode::topic_callback, this, std::placeholders::_1));
    }

private:
    // ros2 interface show ultralytics_ros/msg/YoloResult
    void topic_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "====================");
        RCLCPP_INFO(this->get_logger(), "I received a message");
        RCLCPP_INFO(this->get_logger(), "====================");

        for (auto det :  msg->detections.detections)
        {
            for (auto res :  det.results)
            {
                auto class_id = res.hypothesis.class_id;
                auto score = res.hypothesis.score;
                RCLCPP_INFO(this->get_logger(), "class_id: '%s'", class_id.c_str());
                RCLCPP_INFO(this->get_logger(), "score: %.02f%%", score*100);
            }
            auto x = det.bbox.center.position.x;
            auto y = det.bbox.center.position.y;
            RCLCPP_INFO(this->get_logger(), "center: (%.02f, %.02f)", x, y);
            RCLCPP_INFO(this->get_logger(), "--------------------");

            std::string source_frame = "world";
            std::string target_frame = "openni_rgb_frame";
            try {
                geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                    source_frame, target_frame, tf2::TimePointZero
                );
                RCLCPP_INFO(this->get_logger(), "Camera Position    ::  x: %.02f,  y: %.02f,  z: %.02f",
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z
                );
                RCLCPP_INFO(this->get_logger(), "Camera Orientation :: qx: %.02f, qy: %.02f, qz: %.02f, qw: %.02f",
                    transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w
                );
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
            }
            RCLCPP_INFO(this->get_logger(), "--------------------");
        }
    }

    // subscribe to yolo results
    rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
