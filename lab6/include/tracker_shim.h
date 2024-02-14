#pragma once
#include <vector>
#include <utility>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

namespace cv {
    class Mat;
    class KeyPoint;
}

class FeatureTracker;
using FeatureResult = std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;

struct TrackerWrapper {
  TrackerWrapper();
  ~TrackerWrapper();
  void track(const cv::Mat& prev, const cv::Mat& curr, FeatureResult* result);

 private:
  std::unique_ptr<FeatureTracker> impl_;
};
