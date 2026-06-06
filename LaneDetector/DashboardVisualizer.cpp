// DashboardVisualizer.cpp
// CSS 487 - Final Project Lane Detector
// Author: Ruslan and Tom
// Date: Jun 5 2026


#include "DashboardVisualizer.h"
#include <iostream>



/*
DashboardVisualizer (constructor)
Loads the wheel image and scales it down to 200px tall while preserving aspect ratio.

Preconditions: wheelImagePath points to a PNG in the working directory
Postconditions: originalWheel holds the resized BGRA image, or is empty on failure
*/
DashboardVisualizer::DashboardVisualizer(const std::string& wheelImagePath) {
    // IMREAD_UNCHANGED keeps alpha channel
    originalWheel = cv::imread(wheelImagePath, cv::IMREAD_UNCHANGED);

    if (originalWheel.empty()) {
        std::cout << "ERROR: Could not load steering wheel PNG!" << std::endl;

    } else {
        // scale to 200px tall, preserve aspect ratio so it doesn't look squished
        double ratio = (double)originalWheel.cols / originalWheel.rows;
        int targetHeight = 200;
        int targetWidth = (int)(targetHeight * ratio);

        cv::resize(originalWheel, originalWheel, cv::Size(targetWidth, targetHeight));
    }
}


/*
drawWheel
Rotates the stored wheel image by angle degrees and blends it into the bottom-right corner of mainFrame using the alpha channel as a transparency mask.

Preconditions: mainFrame is a valid BGR image - originalWheel was loaded successfully
Postconditions: mainFrame is modified in-place with the rotated wheel in the bottom-right corner
*/
void DashboardVisualizer::drawWheel(cv::Mat& mainFrame, double angle) {
    if (originalWheel.empty()) {
        return;
    }

    // Rotate around the center of the wheel image
    cv::Point2f center((originalWheel.cols - 1) / 2.0f, (originalWheel.rows - 1) / 2.0f);
    cv::Mat rotMatrix = cv::getRotationMatrix2D(center, angle, 1.0);

    // warpAffine with BORDER_CONSTANT and alpha=0 fills the corners with transparent black
    // instead of stretching the edge pixels which would look bad
    cv::Mat rotatedWheel;
    cv::warpAffine(originalWheel, rotatedWheel, rotMatrix, originalWheel.size(),
        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));


    // position 20px from the bottom-right corner
    int xOffset = mainFrame.cols - rotatedWheel.cols - 20;
    int yOffset = mainFrame.rows - rotatedWheel.rows - 20;

    cv::Mat roi = mainFrame(cv::Rect(xOffset, yOffset, rotatedWheel.cols, rotatedWheel.rows));

    // split out the 4 channels so we can grab the alpha mask separately
    std::vector<cv::Mat> channels;
    cv::split(rotatedWheel, channels);


    if (channels.size() == 4) {

        cv::Mat mask = channels[3]; // channel 3 is alpha - white where wheel exists, black where transparent
        cv::Mat wheelBGR;
        cv::cvtColor(rotatedWheel, wheelBGR, cv::COLOR_BGRA2BGR); // drop alpha before copying
        wheelBGR.copyTo(roi, mask); // only copies wheel pixels where mask is non-zero
    
    }
}