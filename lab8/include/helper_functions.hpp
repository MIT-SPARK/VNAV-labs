#pragma once

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <actionlib/action_definition.h>
#include <actionlib/client/simple_action_client.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

// (TODO) Feel free to use, modify, or delete functions in this file as you see fit

/**
 * Works for freiburg3_teddy dataset only. Converts a sensor_msgs::ImageConstPtr into a bgr OpenCV image, and returns the camera pose.
 * @param tf_listener A tf::TransformListener object that should stay alive from the beginning of your program to the end
 * @param msg The image to process
 * @return If success, OpenCV image and camera pose. Otherwise, an empty cv::Mat and pose. Check for success with `result.first.empty()`
 */
std::pair<cv::Mat, geometry_msgs::Pose> processImageAndPose(tf::TransformListener const& tf_listener, sensor_msgs::ImageConstPtr const& msg) {
    // Convert ROS msg type to OpenCV image type.
    cv::Mat img;
    try {
        img = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return std::make_pair<cv::Mat, geometry_msgs::Pose>({}, {});
    }

    // Get the latest coordinate transform from camera to world
    tf::StampedTransform T_WC;
    try {
        tf_listener.lookupTransform("world", "openni_rgb_optical_frame", ros::Time(0), T_WC);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return std::make_pair<cv::Mat, geometry_msgs::Pose>({}, {});
    }

    // convert from tf to geometry
    geometry_msgs::Pose camera_pose;
    tf::poseTFToMsg(tf::Transform(T_WC), camera_pose);
    return {std::move(img), camera_pose};
}

// A YOLOClient (defined below) makes it easier to get detections in an image from YOLO
// You can construct one with something like: `YOLOClient yolo_detector("darknet_ros/check_for_objects", true)`
using YOLOClient = actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>;

ACTION_DEFINITION(darknet_ros_msgs::CheckForObjectsAction) // defines a bunch of actionlib types that may be useful

/**
 * Computes the centroid of a bounding box which came from a darknet_ros_msgs::CheckForObjectsResult object.
 * A CheckForObjectsResult object can be obtained from the YOLOClient (see above) by calling `client.sendGoalAndWait(goal)`
 * and then `client.getResult()`, where goal is a darknet_ros_msgs::CheckForObjectsGoal object that can be constructed directly.
 *
 * Hint: You may want to confirm that bbox.Class == "teddy bear" before calling this.
 *
 * @param bbox A bounding box of an object detection. Note that bbox.Class is a string that gives the class of object detected
 * @param img Optional. If provided, will draw the bounding box, label, and confidence level on image.
 * @return The centroid of the bounding box
 */
cv::Point2f findCentroid(darknet_ros_msgs::BoundingBox const &bbox, cv::Mat const& img = cv::Mat()) {
    cv::Point2f centroid;

    // Calculate centroid of YOLO detection bounding box
    centroid.x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2.0;
    centroid.y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2.0;

    // save ROI for later
    // roi = cv::Rect(cv::Point2f(bbox.xmin, bbox.ymin), cv::Point2f(bbox.xmax, bbox.ymax));

    // let's see it
    ROS_INFO_STREAM(
        bbox.Class << " (" << bbox.probability << "): (" << bbox.xmin << "," << bbox.ymin << ") x (" << bbox.xmax << "," << bbox.ymax
                   << ")");
    if (!img.empty()) {
        cv::rectangle(img, cv::Point(bbox.xmin, bbox.ymin), cv::Point(bbox.xmax, bbox.ymax), cv::Scalar(0, 0, 255), 2, 8, 0);
        cv::circle(img, centroid, 4, cv::Scalar(0, 0, 255), -1);
    }

    return centroid;
}

/**
 * Converts a bgr8 cv::Mat into a sensor_msgs::Image to be published
 * @param header The header. Construct directly or get this from bbox.header, where bbox is a darknet_ros_msgs::BoundingBox object
 * @param img The cv::Mat bgr3 image
 * @return An image message that can be published using an image_transport::Publisher object
 */
sensor_msgs::ImageConstPtr matToImageMsg(std_msgs::Header const& header, cv::Mat const& img) {
    return cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
}
