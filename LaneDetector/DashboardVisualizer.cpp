#include "DashboardVisualizer.h"
#include <iostream>

DashboardVisualizer::DashboardVisualizer(const std::string& wheelImagePath) {
    originalWheel = cv::imread(wheelImagePath, cv::IMREAD_UNCHANGED);

    if (originalWheel.empty()) {
        std::cout << "ERROR: Could not load steering wheel PNG!" << std::endl;
    }
    else {
        // Calculate the aspect ratio so it doesn't get stretched or squished
        double ratio = (double)originalWheel.cols / originalWheel.rows;
        int targetHeight = 200;
        int targetWidth = (int)(targetHeight * ratio);
        cv::resize(originalWheel, originalWheel, cv::Size(targetWidth, targetHeight));
    }
}

void DashboardVisualizer::drawWheel(cv::Mat& mainFrame, double angle) {
    if (originalWheel.empty()) return;

    // Added 'f' to 2.0f to resolve the double/float conversion warning
    cv::Point2f center((originalWheel.cols - 1) / 2.0f, (originalWheel.rows - 1) / 2.0f);
    cv::Mat rotMatrix = cv::getRotationMatrix2D(center, angle, 1.0);

    cv::Mat rotatedWheel;
    cv::warpAffine(originalWheel, rotatedWheel, rotMatrix, originalWheel.size(),
        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));

    int xOffset = mainFrame.cols - rotatedWheel.cols - 20;
    int yOffset = mainFrame.rows - rotatedWheel.rows - 20;

    cv::Mat roi = mainFrame(cv::Rect(xOffset, yOffset, rotatedWheel.cols, rotatedWheel.rows));

    std::vector<cv::Mat> channels;
    cv::split(rotatedWheel, channels);

    if (channels.size() == 4) {
        cv::Mat mask = channels[3];
        cv::Mat wheelBGR;
        cv::cvtColor(rotatedWheel, wheelBGR, cv::COLOR_BGRA2BGR);
        wheelBGR.copyTo(roi, mask);
    }
}