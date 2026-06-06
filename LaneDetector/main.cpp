// main.cpp
// CSS 487 - Final Project Lane Detector
// Author: Ruslan and Tom

#include <opencv2/opencv.hpp>
#include <iostream>
#include "LaneDetector.h"
#include "DashboardVisualizer.h"

/*
onMouse
Mouse callback attached to the main window so you can click points on a paused frame
prints the (x, y) in a format we can copy straight into LaneDetector.cpp srcPoints.

Preconditions: window has been created with namedWindow before attaching this callback
Postconditions: prints one cv::Point2f line to stdout on each left click, nothing else changes
*/
void onMouse(int event, int x, int y, int flags, void* userdata) {
    // we only care when the left mouse button is clicked down
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "        cv::Point2f(" << x << ", " << y << ")," << std::endl;
    }
}

/*
main
Main execution loop for the lane detection demo.

Preconditions: car_driving_front.mp4 and steering_wheel.png are in the working directory
Postconditions: Runs the lane detection loop until video ends or user quits. Returns -1 on failed open
*/
int main() {
    cv::VideoCapture cap("car_driving_front.mp4");

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    LaneDetector detector;
    DashboardVisualizer dashboard("steering_wheel.png");

    cv::Mat frame;
    cv::Mat debugBirdsEye; // warped top-down view with sliding windows drawn on it
    cv::Mat debugEdges;    // raw canny edge output after HSV masking

    // create and position all three windows upfront so they don't stack on each other on mac
    cv::namedWindow("Main Dashboard View");
    cv::namedWindow("Debug Bird's Eye View");
    cv::namedWindow("Debug Edge Mask");

    cv::moveWindow("Main Dashboard View", 0, 0);
    cv::moveWindow("Debug Bird's Eye View", 1300, 0);
    cv::moveWindow("Debug Edge Mask", 1300, 650);

    // attach the click listener so you can pick warp calibration points interactively
    cv::setMouseCallback("Main Dashboard View", onMouse, nullptr);

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        // resize every frame to a consistent resolution before processing
        cv::resize(frame, frame, cv::Size(1280, 720));

        double steeringAngle = detector.processFrame(frame, debugBirdsEye, debugEdges);
        dashboard.drawWheel(frame, steeringAngle); // draws rotating wheel onto frame in-place

        cv::imshow("Main Dashboard View", frame);
        cv::imshow("Debug Bird's Eye View", debugBirdsEye);
        cv::imshow("Debug Edge Mask", debugEdges);

        char c = (char)cv::waitKey(30);

        // press 'p' to pause so you can click your 4 warp points without the video moving
        if (c == 'p') {
            std::cout << "VIDEO PAUSED. Click your 4 points, then press any key in the video window to resume." << std::endl;
            cv::waitKey(0); // waits indefinitely until another key is pressed
        }
        else if (c == 27 || c == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}