#include "euroc_dataset.h"

#include <ros/ros.h>

#include <opencv2/imgcodecs.hpp>

#include "csv_parser.h"

bool EurocDataset::valid() const {
  return time_stamps.size() == frames.size() && time_stamps.size() == poses.size() &&
         !time_stamps.empty();
}

EurocDataset EurocDataset::load(const std::string& dataset_path, int image_stride) {
  EurocDataset result;
  const auto mav_path = dataset_path + "/mav0";
  const auto csv_path = mav_path + "/cam0/data.csv";

  // Read the data.csv file and extract the filenames for the images.
  std::ifstream img_file(csv_path);
  if (!img_file.good()) {
    ROS_ERROR_STREAM("invalid euroc path: '" << csv_path << "'");
    return result;
  }

  CSVIterator img_it(img_file);
  ++img_it;  // Jump first definition line in data.csv.
  size_t i = 0;
  while (img_it != CSVIterator()) {
    // Ignore all frames that are not every nth frame.
    if (i++ % image_stride != 0) {
      ++img_it;
      continue;
    }

    // Get the filename and load the image.
    const std::string filename = mav_path + "/cam0/data/" + (*img_it)[1];
    const cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    if (image.empty()) {
      std::cout << "Could not load image '" << filename << "'." << std::endl;
      ++img_it;
      continue;
    }
    result.frames.emplace_back(std::move(image));

    // Get the timestamp.
    uint64_t stamp;
    std::istringstream iss((*img_it)[0]);
    iss >> stamp;
    result.time_stamps.emplace_back(stamp);
    ++img_it;
  }

  // Read the poses.
  std::ifstream pose_file(mav_path + "/state_groundtruth_estimate0/data.csv");
  CSVIterator pose_it(pose_file);
  ++pose_it;
  size_t current_frame = 0;
  uint64_t current_frame_stamp = result.time_stamps[current_frame];

  while (pose_it != CSVIterator() && current_frame < result.time_stamps.size()) {
    // Get the timestamp.
    uint64_t stamp;
    std::istringstream iss((*pose_it)[0]);
    iss >> stamp;

    // Find the corresponding frame.
    if (stamp < current_frame_stamp) {
      ++pose_it;
      continue;
    }

    // Get the pose.
    geometry_msgs::Pose pose;
    pose.position.x = std::stod((*pose_it)[1]);
    pose.position.y = std::stod((*pose_it)[2]);
    pose.position.z = std::stod((*pose_it)[3]);
    pose.orientation.w = std::stod((*pose_it)[4]);
    pose.orientation.x = std::stod((*pose_it)[5]);
    pose.orientation.y = std::stod((*pose_it)[6]);
    pose.orientation.z = std::stod((*pose_it)[7]);
    result.poses.emplace_back(std::move(pose));
    ++pose_it;
    ++current_frame;
    current_frame_stamp = result.time_stamps[current_frame];
  }

  // clip the images and timestamps to the number of gt poses
  result.time_stamps.resize(result.poses.size());
  result.frames.resize(result.poses.size());
  return result;
}
