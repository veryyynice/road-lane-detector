#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetector {
public:
    LaneDetector();
    double processFrame(cv::Mat& inputFrame, cv::Mat& debugView, cv::Mat& debugEdges);

private:
    cv::Mat applyHSVMask(const cv::Mat& frame);
    cv::Mat getBirdsEyeView(const cv::Mat& edges);

    // UPDATED: Now takes debugView so we can draw the sliding windows on it
    double calculateCurveAngle(const cv::Mat& warpedEdges, cv::Mat& debugView);

    // NEW: Our Least Squares Polynomial Fitter
    cv::Mat polyfit(const std::vector<cv::Point>& pts);

    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;
};