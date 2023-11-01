#include <glog/logging.h>

#include <filesystem>
#include <iostream>

// ROS.
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// DBoW2
#include <DBoW2/DBoW2.h>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "csv_parser.h"
#include "euroc_dataset.h"
#include "tracker_wrapper.h"

// DBoW2 definitions.
using OrbVocabulary = DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB>;
using OrbDatabase = DBoW2::TemplatedDatabase<cv::Mat, DBoW2::FORB>;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2023  - Lab 8 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement and evaluate place recognition.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 3 | Evaluating BoW Place Recognition with RANSAC
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// Follow along with the code below and handout instructions to implement BoW
// Place Recognition and evaluate it with RANSAC.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

struct Config {
  float score_threshold = 0.0;
  float inlier_threshold = 0.0;
  int max_results = 5;
  int image_stride = 30;
  std::string dataset_path;
  std::string vocabulary_path;

  static Config load() {
    Config config;
    ros::NodeHandle nh("~");

    nh.getParam("score_threshold", config.score_threshold);
    nh.getParam("inlier_threshold", config.inlier_threshold);
    nh.getParam("max_results", config.max_results);
    nh.getParam("image_stride", config.image_stride);

    nh.getParam("dataset_path", config.dataset_path);
    if (!std::filesystem::exists(config.dataset_path)) {
      ROS_ERROR_STREAM("Could not find euroc dataset at '" << config.dataset_path
                                                           << "'.");
      config.dataset_path = "";
    }

    nh.getParam("vocabulary_path", config.vocabulary_path);
    if (!std::filesystem::exists(config.vocabulary_path)) {
      ROS_ERROR_STREAM("Could not find vocabulary at '" << config.vocabulary_path
                                                        << "'.");
      config.vocabulary_path = "";
    }

    return config;
  }

  bool valid() const { return !vocabulary_path.empty() && !dataset_path.empty(); }
};

// Struct to store our matches.
struct Match {
  size_t from;  // index of the from frame.
  size_t to;    // index of the to frame.
  float score;  // score of the match.
};

using Matches = std::vector<Match>;

// Convert a score in the range [0, 1] to a color in the range [0, 255].
std_msgs::ColorRGBA scoreToColor(const float score) {
  std_msgs::ColorRGBA color;
  color.b = 0u;
  if (score > 0.5f) {
    color.r = ((1.f - score) * 2.f);
    color.g = 1.f;
  } else {
    color.r = 1.f;
    color.g = (score * 2.f);
  }
  color.b = 0.0f;
  color.a = 1.0f;
  return color;
}

struct Visualizer {
  Visualizer(const EurocDataset& data,
             const Matches& matches,
             bool color_by_score = false) {
    traj_pub = nh.advertise<visualization_msgs::Marker>("trajectory", 1, true);
    lc_pub = nh.advertise<visualization_msgs::Marker>("loopclosures", 1, true);
    drawTrajectory(data);
    drawLoopClosures(data, matches, color_by_score);
  }

  void drawTrajectory(const EurocDataset& data) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for (const auto& pose : data.poses) {
      marker.points.push_back(pose.position);
    }

    traj_pub.publish(marker);
  }

  void drawLoopClosures(const EurocDataset& data,
                        const Matches& matches,
                        bool color_by_score = false) {
    if (matches.empty()) {
      ROS_WARN_STREAM("Trying to visualize empty matches! Disabling drawing matches");
      return;
    }

    // Visualize proposed loopclosures.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "loop_closures";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    float min_score, max_score;
    if (!color_by_score) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
    } else {
      min_score = std::numeric_limits<float>::max();
      max_score = std::numeric_limits<float>::min();
      for (const auto& match : matches) {
        min_score = std::min(min_score, match.score);
        max_score = std::max(max_score, match.score);
      }
      ROS_INFO_STREAM("Plotting scores: " << min_score << " (red) -> " << max_score
                                          << " (green)");
    }

    for (const Match& match : matches) {
      if (match.from >= data.poses.size() || match.to >= data.poses.size()) {
        ROS_ERROR_STREAM("Invalid match: " << match.from << " -> " << match.to
                                           << " for " << data.poses.size() << " poses");
        continue;
      }

      marker.points.push_back(data.poses[match.from].position);
      marker.points.push_back(data.poses[match.to].position);
      if (color_by_score) {
        const float normalized_score =
            (match.score - min_score) / (max_score - min_score);
        marker.colors.push_back(scoreToColor(normalized_score));
        marker.colors.push_back(scoreToColor(normalized_score));
      }
    }

    lc_pub.publish(marker);
  }

  void spin() { ros::spin(); }

  ros::NodeHandle nh;
  ros::Publisher traj_pub;
  ros::Publisher lc_pub;
};

std::vector<cv::Mat> descriptorMatToVector(const cv::Mat& descriptors) {
  std::vector<cv::Mat> result;
  // (TODO) Convert a (N, L) matrix of N descriptors of length L to a
  // vector of N descriptors of size (1, L)
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution

  return result;
}

Matches findLoopClosures(const Config& config,
                         const EurocDataset& data,
                         const OrbVocabulary& vocabulary) {
  Matches result;

  // Create the DBoW2 database.
  OrbDatabase database(vocabulary);

  ROS_INFO_STREAM("Finding matches for " << data.frames.size() << " frames...");
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(500, 1.2f, 1);

  for (size_t i = 0; i < data.frames.size(); ++i) {
    const cv::Mat& frame = data.frames[i];
    // Extract ORB features from the frame.
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector->detect(frame, keypoints);
    detector->compute(frame, keypoints, descriptors);

    std::vector<cv::Mat> descriptors_vec = descriptorMatToVector(descriptors);

    // (TODO) Compute the dbow descriptor of the frame and find any
    // previous matches. For the previous frame with the best score
    // (i.e., lowest distance to the current DBoW2 descriptor),
    // add a 'Match' struct to the result.
    // You may find the header files of DBoW2 useful, specifically:
    //   - TemplatedVocabulary (https://github.com/dorian3d/DBoW2/blob/master/include/DBoW2/TemplatedVocabulary.h)
    //   - TemplatedDatabase (https://github.com/dorian3d/DBoW2/blob/master/include/DBoW2/TemplatedDatabase.h)
    //
    // Hint: Pay attention to the signature for `transform` and `query`
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
    //
    // ~~~~ end solution
  }

  ROS_INFO_STREAM("Finished finding matches (found " << result.size()
                                                     << " candidates).");
  return result;
}

void pruneLoopClosures(Matches& matches, const float min_score) {
  // (TODO) Prune the matches that have a score less than min_score.
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution
}

void pruneSequentialLoopClosures(Matches& matches) {
  const size_t num_matches_before = matches.size();

  // (TODO) Prune the matches that correspond to sequential frames.
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  // ~~~~ end solution

  ROS_INFO_STREAM("Pruned " << num_matches_before - matches.size()
                            << " sequential matches");
}

void verifyLoopClosuresRansac(const Config& config,
                              const EurocDataset& data,
                              Matches& matches) {
  // The wrapper can be used like your tracker from lab 5.
  lab8::TrackerWrapper wrapper;

  for (Match& match : matches) {
    const auto& frame_from = data.frames.at(match.from);
    const auto& frame_to = data.frames.at(match.to);

    // (TODO) Use the code from lab 5 to compute the number of inlier
    // matches between frames to geometrically verify that the loop closure
    // makes sense. Set the score of each match to the number of inliers.
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
    //
    // ~~~~ end solution
  }
}

int main(int argc, char** argv) {
  // Initialize logging, flags, and ROS.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "deliverable_3");
  ros::NodeHandle nh;

  const auto config = Config::load();
  if (!config.valid()) {
    return 1;
  }

  // Load every n-th image from the dataset folder.
  ROS_INFO_STREAM("Loading frames ...");
  const auto data = EurocDataset::load(config.dataset_path, config.image_stride);
  if (!data) {
    ROS_ERROR_STREAM("Loaded data is not valid");
    return 1;
  }
  ROS_INFO_STREAM("Loaded " << data.frames.size() << " frames from dataset.");

  // Load the DBoW2 ORB vocabulary.
  OrbVocabulary vocabulary;
  ROS_INFO_STREAM("Loading vocabulary ...");
  vocabulary.load(config.vocabulary_path);
  ROS_INFO_STREAM("Loaded vocabulary with " << vocabulary.size() << " visual words.");

  // Part 3.1: Compute matches between images
  Matches matches = findLoopClosures(config, data, vocabulary);

  // Part 3.2 Prune all matches that correspond to sequential frames, as these loop closures will not add much value.
  pruneSequentialLoopClosures(matches);

  // Part 3.3: Prune all matches with a low score.
  if (config.score_threshold > 0) {
    const size_t num_matches_before = matches.size();
    pruneLoopClosures(matches, config.score_threshold);
    ROS_INFO_STREAM("Pruned " << num_matches_before - matches.size()
                              << " matches with a score less than "
                              << config.score_threshold);
  }

  // Prune all remaining matches with a low number of inliers.
  if (config.inlier_threshold > 0) {
    // Part 3.4 [Optional] Do geometric verification (lab5) of the remaining matches to make sure we filter out bad candidates.
    verifyLoopClosuresRansac(config, data, matches);

    // Note that the score of each match is now the number of inliers.
    const size_t num_matches_before = matches.size();
    pruneLoopClosures(matches, config.inlier_threshold);
    ROS_INFO_STREAM("Pruned " << num_matches_before - matches.size()
                              << " matches with  less than " << config.inlier_threshold
                              << " inliers.");
  }

  // Visualize all detected matches.
  const bool color_by_score =
      true;  // If true color all matches by their score from red (lowest) to green (highest). If false just show them in green.
  Visualizer visualizer(data, matches, color_by_score);
  visualizer.spin();
  return 0;
}
