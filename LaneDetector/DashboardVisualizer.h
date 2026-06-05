// DashboardVisualizer.h
// CSS 487 - Final Project: Lane Detector
// Author: Ruslan and Tom
// Date: Jun 5 2026
//

// Declares the DashboardVisualizer class.
// Loads a steering wheel PNG once at startup, then each frame rotates it
// by the computed steering angle and composites it onto the video frame.

#pragma once
#include <opencv2/opencv.hpp>
#include <string>

// DashboardVisualizer
// handles loading and drawing a rotating steering wheel overlay on the video frame
// uses the PNG's alpha channel for clean transparent compositing
class DashboardVisualizer {
public:
    // Constructor
    // Preconditions:  wheelImagePath is a valid path to a PNG with an alpha channel
    // Postconditions: loads and resizes the wheel image; prints an error if the file isn't found
    DashboardVisualizer(const std::string& wheelImagePath);

    // drawWheel
    // rotates the wheel by angle degrees and blends it into the bottom-right corner of mainFrame
    // Preconditions:  mainFrame is a valid BGR image large enough to fit the wheel (200px tall);
    //                 originalWheel was loaded successfully
    // Postconditions: mainFrame is modified in-place with the rotated wheel composited on top
    void drawWheel(cv::Mat& mainFrame, double angle);

private:
    cv::Mat originalWheel; // the source wheel image, kept at full quality for re-rotation each frame
};
