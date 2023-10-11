#include "tracker_shim.h"
#include "feature_tracker.h"
#include "sift_feature_tracker.h"


TrackerWrapper::TrackerWrapper() {
  impl_.reset(new SiftFeatureTracker());
}

TrackerWrapper::~TrackerWrapper() {}

void TrackerWrapper::track(const cv::Mat& prev, const cv::Mat& curr, FeatureResult* ret) {
  impl_->trackFeatures(prev, curr, ret);
}
