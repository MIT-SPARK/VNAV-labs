#pragma once
#include <geometry_msgs/Pose.h>

#include <cstdint>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

struct EurocDataset {
  /**
   * Load images from cam0 from EuRoc.
   */
  static EurocDataset load(const std::string& dataset_path, int image_stride);

  /**
   * Get whether the dataset is valid.
   */
  bool valid() const;

  /**
   * Get whether the dataset is valid.
   */
  inline operator bool() const { return valid(); }

 public:
  std::vector<uint64_t> time_stamps;
  std::vector<cv::Mat> frames;
  std::vector<geometry_msgs::Pose> poses;
};
