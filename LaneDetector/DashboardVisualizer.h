#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class DashboardVisualizer {
public:
    DashboardVisualizer(const std::string& wheelImagePath);
    void drawWheel(cv::Mat& mainFrame, double angle);

private:
    cv::Mat originalWheel;
};