// LaneDetector.h
// CSS 487 - Final Project: Lane Detector
// Author: Ruslan and Tom
// Date: Jun 5 2026
//

// Declares the LaneDetector class.
// One instance processes the video frame by frame. The public interface is just
// processFrame — everything else is internal pipeline stages.

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// LaneDetector
// runs the full lane detection pipeline on a single frame and returns a steering angle
// internally: HSV mask -> canny edges -> perspective warp -> sliding windows -> polyfit -> angle
class LaneDetector {
public:
    // Constructor
    // Preconditions:  none
    // Postconditions: sets up the hardcoded src/dst warp points for the test video
    LaneDetector();

    // processFrame
    // main entry point — call once per frame
    // Preconditions:  inputFrame is a valid BGR image at 1280x720
    // Postconditions: draws ROI trapezoid onto inputFrame in-place, fills debugView with
    //                 the bird's eye sliding window image, fills debugEdges with the canny
    //                 edge mask, and returns the steering angle in degrees
    double processFrame(cv::Mat& inputFrame, cv::Mat& debugView, cv::Mat& debugEdges);

private:
    // applyHSVMask
    // Preconditions:  frame is a valid BGR image
    // Postconditions: returns a binary canny edge image with only white/yellow lane pixels kept
    cv::Mat applyHSVMask(const cv::Mat& frame);

    // getBirdsEyeView
    // Preconditions:  edges is a grayscale binary edge image the same size as the src trapezoid
    // Postconditions: returns a 400x600 perspective-warped top-down version of the edges
    cv::Mat getBirdsEyeView(const cv::Mat& edges);

    // calculateCurveAngle
    // Preconditions:  warpedEdges is a 400x600 grayscale binary edge image; debugView is BGR same size
    // Postconditions: draws sliding windows and fitted curves onto debugView, returns steering angle
    //                 in degrees. Returns 0 if not enough lane pixels were found.
    double calculateCurveAngle(const cv::Mat& warpedEdges, cv::Mat& debugView);

    // polyfit
    // least squares 2nd degree polynomial fit: x = Ay^2 + By + C
    // axes are swapped on purpose because lane lines run vertically in the warped view
    // Preconditions:  pts has at least 3 points
    // Postconditions: returns a 3x1 Mat of doubles [A, B, C]
    cv::Mat polyfit(const std::vector<cv::Point>& pts);

    std::vector<cv::Point2f> srcPoints; // the 4 corners of the trapezoid ROI on the original frame
    std::vector<cv::Point2f> dstPoints; // the 4 corners of the flat 400x600 output rectangle
};
