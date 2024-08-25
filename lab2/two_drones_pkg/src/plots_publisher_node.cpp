#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <list>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PlotsPublisherNode : public rclcpp::Node {
  rclcpp::Time startup_time;
  rclcpp::TimerBase::SharedPtr heartbeat;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  int num_trails;

  class TrajTrail {
    PlotsPublisherNode* parent;
    static int id;
    std::list<geometry_msgs::msg::Point> poses;
    std::string ref_frame, dest_frame;
    size_t buffer_size;

    std::string ns;

    visualization_msgs::msg::Marker marker_out;

    void update() {
      geometry_msgs::msg::TransformStamped
          transform;  // NOTE: you need to populate this transform
      try {
        // ~~~~~~~~~~~~~~~~~~~~~~  BEGIN OF EDIT SECTION ~~~~~~~~~~~~~~~~~~~~~~~~~

        /* The transform object needs to be populated with the most recent
         * transform from ref_frame to dest_frame as provided by tf.

         * Relevant variables in this scope:
         *   - ref_frame, the frame of reference relative to which the
         trajectory
         *                needs to be plotted (given)
         *   - dest_frame, the frame of reference of the object whose trajectory
         *                 needs to be plotted (given)
         *   - parent->tf_listener, a tf2_ros::TransformListener object (given)
         *   - transform, the transform object that needs to be populated
         *
         * HINT: use "lookupTransform", see
         *
         https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
         *
         * NOTE: only using "lookupTransform", you might see a lot of errors in
         your terminal!
         *       Unlike other topic types - which are typically received on a
         callback basis -
         *       \tf can be polled at any time and therefore "lookupTransform"
         might fail
         *       due to timing issues. To fix it, consider the method
         "canTransform" with
         *       a small wait time.
         */

        // TODO: fill in here

        // ~~~~~~~~~~~~~~~~~~~~~~~~  END OF EDIT SECTION ~~~~~~~~~~~~~~~~~~~~~~~~~
        while (poses.size() >= buffer_size) poses.pop_front();

        geometry_msgs::msg::Point tmp;
        tmp.x = transform.transform.translation.x;
        tmp.y = transform.transform.translation.y;
        tmp.z = transform.transform.translation.z;
        poses.push_back(tmp);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM(parent->get_logger(),
                            "transform lookup failed: " << ex.what());
      }
    }

   public:
    TrajTrail() : parent(NULL){};

    TrajTrail(PlotsPublisherNode* parent_,
              const std::string& ref_frame_,
              const std::string& dest_frame_,
              int buffer = 160)
        : parent(parent_),
          ref_frame(ref_frame_),
          dest_frame(dest_frame_),
          buffer_size(buffer) {
      marker_out.header.frame_id = ref_frame;
      marker_out.ns = "trails";
      marker_out.id = parent->num_trails++;
      marker_out.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker_out.action = visualization_msgs::msg::Marker::ADD;
      marker_out.color.a = 0.8;
      marker_out.scale.x = 0.02;
      marker_out.lifetime = rclcpp::Duration::from_seconds(1.0);
    }

    void setColor(float r_, float g_, float b_) {
      marker_out.color.r = r_;
      marker_out.color.g = g_;
      marker_out.color.b = b_;
    }

    void setNamespace(const std::string& ns_) { marker_out.ns = ns_; }

    void setDashed() { marker_out.type = visualization_msgs::msg::Marker::LINE_LIST; }

    visualization_msgs::msg::Marker getMarker() {
      update();
      marker_out.header.stamp = parent->now();
      marker_out.points.clear();
      for (auto& p : poses) marker_out.points.push_back(p);

      if (marker_out.type == visualization_msgs::msg::Marker::LINE_LIST &&
          poses.size() % 2)
        marker_out.points.resize(poses.size() - 1);

      return marker_out;
    }
  };

  TrajTrail av1trail, av2trail, av2trail_rel;

 public:
  PlotsPublisherNode() : Node("plots_publisher_node"), num_trails(0) {
    RCLCPP_INFO_STREAM(get_logger(), "Started plot node");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    startup_time = now();
    markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>("visuals", 0);

    RCLCPP_INFO_STREAM(get_logger(), "about to start heartbeat");
    heartbeat = create_timer(this,
                             get_clock(),
                             rclcpp::Duration::from_seconds(0.02),
                             std::bind(&PlotsPublisherNode::onPublish, this));
    heartbeat->reset();
    RCLCPP_INFO_STREAM(get_logger(), "started heartbeat");

    av1trail = TrajTrail(this, "world", "av1", 300);
    av1trail.setColor(.25, .52, 1);
    av1trail.setNamespace("Trail av1-world");
    av2trail = TrajTrail(this, "world", "av2", 300);
    av2trail.setColor(.8, .4, .26);
    av2trail.setNamespace("Trail av2-world");
    av2trail_rel = TrajTrail(this, "av1", "av2", 160);
    av2trail_rel.setDashed();
    av2trail_rel.setColor(.8, .4, .26);
    av2trail_rel.setNamespace("Trail av2-av1");
  }

  void onPublish() {
    visualization_msgs::msg::MarkerArray visuals;
    visuals.markers.resize(2);
    visualization_msgs::msg::Marker& av1(visuals.markers[0]);
    visualization_msgs::msg::Marker& av2(visuals.markers[1]);
    av1.header.frame_id = "av1";
    av1.ns = "AVs";
    av1.id = 0;
    av1.header.stamp = now();
    av1.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    av1.mesh_resource = "package://two_drones_pkg/mesh/quadrotor.dae";
    av1.action = visualization_msgs::msg::Marker::ADD;
    av1.pose.orientation.w = 1.0;
    av1.scale.x = av1.scale.y = av1.scale.z = av1.color.a = 1.0;
    av1.color.r = .25;
    av1.color.g = .52;
    av1.color.b = 1;
    av1.lifetime = rclcpp::Duration::from_seconds(1.0);
    av2 = av1;
    av2.header.frame_id = "av2";
    av2.ns = "AVs";
    av2.id = 1;
    av2.color.r = .8;
    av2.color.g = .4;
    av2.color.b = .26;

    // Trails
    visuals.markers.push_back(av1trail.getMarker());
    visuals.markers.push_back(av2trail.getMarker());
    visuals.markers.push_back(av2trail_rel.getMarker());
    markers_pub->publish(visuals);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlotsPublisherNode>());
  return 0;
}
