#include "tracker_wrapper.h"

#include "feature_tracker.h"
#include "orb_feature_tracker.h"

namespace lab8 {

TrackerWrapper::TrackerWrapper() { impl_.reset(new OrbFeatureTracker()); }

TrackerWrapper::~TrackerWrapper() {}

void TrackerWrapper::track(const cv::Mat& from,
                           const cv::Mat& to,
                           FeatureResult* ret) {
  impl_->trackFeatures(from, to, ret);
}

}  // namespace lab8
