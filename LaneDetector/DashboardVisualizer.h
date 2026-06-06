// DashboardVisualizer.h
// CSS 487 - Final Project Lane Detector
// Author: Ruslan and Tom
// Date: Jun 5 2026

#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class DashboardVisualizer {
public:
    /*
    Constructor

    Preconditions: wheelImagePath is a valid path to a PNG with an alpha channel
    Postconditions: loads and resizes the wheel image. Prints an error if the file isn't found
    */
    DashboardVisualizer(const std::string& wheelImagePath);

    /*
    drawWheel
    Rotates the wheel by angle degrees and blends it into the bottom-right corner of mainFrame.

    Preconditions: mainFrame is a valid BGR image large enough to fit the wheel. originalWheel was loaded
    Postconditions: mainFrame is modified in-place with the rotated wheel composited on top
    */
    void drawWheel(cv::Mat& mainFrame, double angle);

private:
    cv::Mat originalWheel; // the source wheel image kept at full quality for rerotation each frame
};