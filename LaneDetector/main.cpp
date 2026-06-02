#include <opencv2/opencv.hpp>
#include <iostream>
#include "LaneDetector.h"
#include "DashboardVisualizer.h"

// --- NEW: The Mouse Callback Function ---
// This function will automatically run every time your mouse does something over the window
void onMouse(int event, int x, int y, int flags, void* userdata) {
    // We only care when the left mouse button is clicked down
    if (event == cv::EVENT_LBUTTONDOWN) {
        // Print it perfectly formatted so you can copy/paste it into LaneDetector.cpp!
        std::cout << "        cv::Point2f(" << x << ", " << y << ")," << std::endl;
    }
}

int main() {
    cv::VideoCapture cap("car_driving_front.mp4");

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    LaneDetector detector;
    DashboardVisualizer dashboard("steering_wheel.png");

    cv::Mat frame;
    cv::Mat debugBirdsEye;
    cv::Mat debugEdges;

    // --- NEW: Explicitly create the window and attach the mouse listener ---
    cv::namedWindow("Main Dashboard View");
    cv::setMouseCallback("Main Dashboard View", onMouse, nullptr);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::resize(frame, frame, cv::Size(1280, 720));

        double steeringAngle = detector.processFrame(frame, debugBirdsEye, debugEdges);
        dashboard.drawWheel(frame, steeringAngle);

        cv::imshow("Main Dashboard View", frame);
        cv::imshow("Debug Bird's Eye View", debugBirdsEye);
        cv::imshow("Debug Edge Mask", debugEdges);

        char c = (char)cv::waitKey(30);

        // Pause the video if you press 'p' so you can easily click your points!
        if (c == 'p') {
            std::cout << "VIDEO PAUSED. Click your 4 points, then press any key in the video window to resume." << std::endl;
            cv::waitKey(0); // Waits indefinitely until another key is pressed
        }
        else if (c == 27 || c == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}