#include "sift_feature_tracker.h"

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

using namespace cv;

SiftFeatureTracker::SiftFeatureTracker() : FeatureTracker(), detector(SIFT::create()) {}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 3 | Feature Descriptors (SIFT)
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// Complete the `detectKeypoints` and `describeKeypoints`

/** TODO: this function detects keypoints in an image.
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
*/
void SiftFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  //
  //     Hint: look at the header file for this class in the include folder.
  //     There is a private member of the class that you can use directly for
  //     this function, and you should only need one function call.
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** TODO: this function describes keypoints in an image.
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image. Depending
    on the detector used, some keypoints might be added or removed.
    @param[out] descriptors List of descriptors for the given keypoints.
*/
void SiftFeatureTracker::describeKeypoints(const cv::Mat& img,
                                           std::vector<KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  //
  //     Hint: look at the header file for this class in the include folder.
  //     There is a private member of the class that you can use directly for
  //     this function, and you should only need one function call.
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 4 | Feature Descriptors (SIFT)
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// Complete the matchDescriptors function

void SiftFeatureTracker::matchDescriptors(const cv::Mat& descriptors_1,
                                          const cv::Mat& descriptors_2,
                                          std::vector<std::vector<DMatch>>* matches,
                                          std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);

  // Here we initialize a cv::FlannBasedMatcher for you
  //
  // Note: if you look at the OpenCV documentation, you should be able to find
  // some tutorials that can greatly help you complete this function.
  FlannBasedMatcher matcher;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // Match the descriptor vectors using FLANN (See Deliverable 4). Specifically:
  //
  //   1. Take the best 2 using the function FLANN function knnMatch, which
  //   takes the best k nearest neighbours.
  //   Store your matches in the argument 'matches'.
  //
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  //     Hint: you should be able to do it using one function call on `matcher`
  //
  // ~~~~ end solution

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //   2. Remove ambiguous matches, using SIFT's authors approach (detailed in
  //   the handout). Make use of the DMatch structure.

  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  //     Hint: you need to write a for loop that iterate through all the matches
  //     obtained from the previous step, and perform a check on it to see
  //     whether it can go into the good_matches vector.
  //     Hint 2: make sure to consider the situation where matches[i] is fewer
  //     than 2.
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
