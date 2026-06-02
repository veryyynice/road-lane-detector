#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetector {
public:
    LaneDetector();

    // Changed to pass inputFrame by reference (removed const) so we can draw on it
    // Added debugEdges to visualize the color filter
    double processFrame(cv::Mat& inputFrame, cv::Mat& debugView, cv::Mat& debugEdges);

private:
    cv::Mat applyHSVMask(const cv::Mat& frame);
    cv::Mat getBirdsEyeView(const cv::Mat& edges);
    double calculateCurveAngle(const cv::Mat& warpedEdges);

    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;
};