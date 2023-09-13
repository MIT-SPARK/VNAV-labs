#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <list>

class PlotsPublisherNode {
  ros::Time startup_time;
  ros::Timer heartbeat;
  ros::NodeHandle nh;
  ros::Publisher markers_pub;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::Buffer tf_buffer;
  int num_trails;

  class TrajTrail {
    PlotsPublisherNode* parent;
    static int id;
    std::list<geometry_msgs::Point> poses;
    std::string ref_frame, dest_frame;
    size_t buffer_size;

    std::string ns;
    float r, g, b, a;

    visualization_msgs::Marker marker_out;

    void update() {
      // NOTE: you need to populate this transform
      geometry_msgs::TransformStamped transform;
      try {
        // ~~~~~~~~~~~~~~~~~~~~~~  BEGIN OF EDIT SECTION  ~~~~~~~~~~~~~~~~~~~~~~~~~

        /* The transform object needs to be populated with the most recent
         * transform from ref_frame to dest_frame as provided by tf.

         * Relevant variables in this scope:
         *   - ref_frame, the frame of reference relative to which the trajectory
         *                needs to be plotted (given)
         *   - dest_frame, the frame of reference of the object whose trajectory
         *                 needs to be plotted (given)
         *   - parent->tf_buffer, a tf2_ros::Buffer object (given)
         *   - transform, the geometry_msgs::TransformStamped object that needs to be populated
         *
         * HINT: use "lookupTransform", see
         * https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29#TheCode
         */

        // *** FILL IN ***

        // ~~~~~~~~~~~~~~~~~~~~~~~~  END OF EDIT SECTION  ~~~~~~~~~~~~~~~~~~~~~~~~~
        while (poses.size() >= buffer_size) {
          poses.pop_front();
        }

        geometry_msgs::Point tmp;
        tmp.x = transform.transform.translation.x;
        tmp.y = transform.transform.translation.y;
        tmp.z = transform.transform.translation.z;
        poses.push_back(tmp);
      } catch (const tf2::TransformException& ex) {
        ROS_ERROR_STREAM("transform lookup failed: " << ex.what());
      }
    }

   public:
    TrajTrail() : parent(nullptr){};

    TrajTrail(PlotsPublisherNode* parent_,
              const std::string& ref_frame_,
              const std::string& dest_frame_,
              int buffer_size_ = 160)
        : parent(parent_),
          ref_frame(ref_frame_),
          dest_frame(dest_frame_),
          buffer_size(buffer_size_) {
      if (buffer_size <= 0) {
        ROS_ERROR_STREAM("invalid buffer size! defaulting to 10");
        buffer_size = 10;
      }

      marker_out.header.frame_id = ref_frame;
      marker_out.ns = "trails";
      marker_out.id = parent->num_trails++;
      marker_out.type = visualization_msgs::Marker::LINE_STRIP;
      marker_out.action = visualization_msgs::Marker::ADD;
      marker_out.color.a = 0.8;
      marker_out.scale.x = 0.02;
      marker_out.lifetime = ros::Duration(1.0);
    }

    void setColor(float r_, float g_, float b_) {
      marker_out.color.r = r_;
      marker_out.color.g = g_;
      marker_out.color.b = b_;
    }

    void setNamespace(const std::string& ns_) { marker_out.ns = ns_; }

    void setDashed() { marker_out.type = visualization_msgs::Marker::LINE_LIST; }

    visualization_msgs::Marker getMarker() {
      update();
      marker_out.header.stamp = ros::Time::now();
      marker_out.points.clear();
      for (auto& p : poses) marker_out.points.push_back(p);

      if (marker_out.type == visualization_msgs::Marker::LINE_LIST && poses.size() % 2)
        marker_out.points.resize(poses.size() - 1);

      return marker_out;
    }
  };

  TrajTrail av1trail;
  TrajTrail av2trail;
  TrajTrail av2trail_rel;

 public:
  PlotsPublisherNode() : tf_listener(tf_buffer), num_trails(0) {
    startup_time = ros::Time::now();
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("visuals", 0);
    heartbeat =
        nh.createTimer(ros::Duration(0.02), &PlotsPublisherNode::onPublish, this);
    heartbeat.start();
    av1trail = TrajTrail(this, "world", "av1", 300);
    av1trail.setColor(0.25, 0.52, 1.0);
    av1trail.setNamespace("Trail av1-world");
    av2trail = TrajTrail(this, "world", "av2", 300);
    av2trail.setColor(0.8, 0.4, 0.26);
    av2trail.setNamespace("Trail av2-world");
    av2trail_rel = TrajTrail(this, "av1", "av2", 160);
    av2trail_rel.setDashed();
    av2trail_rel.setColor(0.8, 0.4, 0.26);
    av2trail_rel.setNamespace("Trail av2-av1");

    ROS_INFO_STREAM("Waiting for av1 and av2 transforms to be broadcast...");
    while (ros::ok()) {
      const bool av1_present = tf_buffer.canTransform("av1", "world", ros::Time(0));
      const bool av2_present = tf_buffer.canTransform("av2", "world", ros::Time(0));
      if (av1_present && av2_present) {
        ROS_INFO_STREAM("Necessary frames are present, starting!");
        break;
      }
    }
  }

  void onPublish(const ros::TimerEvent&) {
    visualization_msgs::MarkerArray visuals;
    visuals.markers.resize(2);
    visualization_msgs::Marker& av1(visuals.markers[0]);
    visualization_msgs::Marker& av2(visuals.markers[1]);
    av1.header.frame_id = "av1";
    av1.ns = "AVs";
    av1.id = 0;
    av1.header.stamp = ros::Time();
    av1.type = visualization_msgs::Marker::MESH_RESOURCE;
    av1.mesh_resource = "package://two_drones_pkg/mesh/quadrotor.dae";
    av1.action = visualization_msgs::Marker::ADD;
    av1.pose.orientation.w = 1.0;
    av1.scale.x = av1.scale.y = av1.scale.z = av1.color.a = 1.0;
    av1.color.r = 0.25;
    av1.color.g = 0.52;
    av1.color.b = 1.0;
    av1.lifetime = ros::Duration(1.0);
    av2 = av1;
    av2.header.frame_id = "av2";
    av2.ns = "AVs";
    av2.id = 1;
    av2.color.r = 0.8;
    av2.color.g = 0.4;
    av2.color.b = 0.26;

    // Trails
    visuals.markers.push_back(av1trail.getMarker());
    visuals.markers.push_back(av2trail.getMarker());
    visuals.markers.push_back(av2trail_rel.getMarker());
    markers_pub.publish(visuals);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "plots_publisher_node");
  PlotsPublisherNode node;
  ros::spin();
  return 0;
}
