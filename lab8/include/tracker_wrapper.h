#pragma once
#include <memory>
#include <utility>
#include <vector>

namespace cv {
class Mat;
class KeyPoint;
}  // namespace cv

class FeatureTracker;
using FeatureResult = std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;

namespace lab8 {
struct TrackerWrapper {
  TrackerWrapper();
  ~TrackerWrapper();
  void track(const cv::Mat& from, const cv::Mat& to, FeatureResult* result);

 private:
  std::unique_ptr<FeatureTracker> impl_;
};

}  // namespace lab8
