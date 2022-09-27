// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2019  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement an ORB feature tracker derived class
//  that inherits from your FeatureTracker base class.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 6 (continued) | Comparing Feature Matching on Real Data
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// For this part, you will need to implement the same functions you've just
// implemented in the case of SIFT, SURF, and ORB, but now for FAST+BRIEF.
//
// NOTE: We do not provide much template code for this section or the corresponding
// source file (fast_feature_tracker.cpp), but you should define and implement
// the same set of functions as for SIFT and SURF (see the template code for
// those cases for reference as needed)
//
// NOTE: Don't forget to modify the CMakeLists.txt to add these files!
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "feature_tracker.h"

class FastFeatureTracker : public FeatureTracker {
public:
  FastFeatureTracker();

  ~FastFeatureTracker() override = default;

protected:

  /**
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
  */
  void detectKeypoints(const cv::Mat &img,
                       std::vector<cv::KeyPoint> *keypoints) const override;

  /**
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image. Depending
    on the detector used some keypoints might be added or removed.
    @param[out] descriptors List of descriptors for the given keypoints.
  */
  void describeKeypoints(const cv::Mat &img,
                         std::vector<cv::KeyPoint> *keypoints,
                         cv::Mat *descriptors) const override;

  /** This function matches descriptors.
      @param[in] descriptors_1 First list of descriptors.
      @param[in] descriptors_2 Second list of descriptors.
      @param[out] matches List of k best matches between descriptors.
      @param[out] good_matches List of descriptors classified as "good"
  */
  void matchDescriptors(const cv::Mat &descriptors_1,
                        const cv::Mat &descriptors_2,
                        std::vector<std::vector<cv::DMatch>> *matches,
                        std::vector<cv::DMatch> *good_matches) const override;

private:
  // Detector
  cv::Ptr<cv::FeatureDetector> detector;
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
