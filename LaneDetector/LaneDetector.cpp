#include "LaneDetector.h"
#include <iostream>

LaneDetector::LaneDetector() {
    // We will likely need to adjust these after you see the red box on the screen!
    srcPoints = {
        //cv::Point2f(757, 381),
        //cv::Point2f(501, 371),
        //cv::Point2f(4, 654),
        //cv::Point2f(1222, 666),
         cv::Point2f(513, 404),  // 1. Top-Left
        cv::Point2f(739, 407),  // 2. Top-Right
        cv::Point2f(5, 654),    // 3. Bottom-Left
        cv::Point2f(1269, 661)  // 4. Bottom-Right
    };

    dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(400, 0),
        cv::Point2f(0, 600),
        cv::Point2f(400, 600)
    };
}

double LaneDetector::processFrame(cv::Mat& inputFrame, cv::Mat& debugView, cv::Mat& debugEdges) {
    // 1. Get edges and output them to our new debug window
    cv::Mat edges = applyHSVMask(inputFrame);
    edges.copyTo(debugEdges);

    // --- UPDATED: Draw the trapezoid without crossing the wires ---
    // We manually push the points in a clockwise circle: 
    // Top-Left (0) -> Top-Right (1) -> Bottom-Right (3) -> Bottom-Left (2)
    std::vector<cv::Point> intPoints;
    intPoints.push_back(cv::Point((int)srcPoints[0].x, (int)srcPoints[0].y));
    intPoints.push_back(cv::Point((int)srcPoints[1].x, (int)srcPoints[1].y));
    intPoints.push_back(cv::Point((int)srcPoints[3].x, (int)srcPoints[3].y)); // Swap order here
    intPoints.push_back(cv::Point((int)srcPoints[2].x, (int)srcPoints[2].y)); // Swap order here

    // Draws a closed polygon with a thickness of 2
    cv::polylines(inputFrame, intPoints, true, cv::Scalar(0, 0, 255), 2);

    // 2. Warp into Bird's Eye View
    cv::Mat warpedEdges = getBirdsEyeView(edges);
    warpedEdges.copyTo(debugView);

    // 3. Calculate how much the road is turning
    return calculateCurveAngle(warpedEdges);
}
cv::Mat LaneDetector::applyHSVMask(const cv::Mat& frame) {
    cv::Mat hsv, whiteMask, yellowMask, finalMask, edges;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // WIDENED MASK: Dropped the minimum Value to 150 to catch darker, shadowy gray lines
    cv::inRange(hsv, cv::Scalar(0, 0, 150), cv::Scalar(180, 50, 255), whiteMask);
    cv::inRange(hsv, cv::Scalar(15, 100, 100), cv::Scalar(35, 255, 255), yellowMask);

    cv::bitwise_or(whiteMask, yellowMask, finalMask);
    cv::GaussianBlur(finalMask, finalMask, cv::Size(5, 5), 0);
    cv::Canny(finalMask, edges, 50, 150);

    return edges;
}

cv::Mat LaneDetector::getBirdsEyeView(const cv::Mat& edges) {
    cv::Mat warped;
    cv::Mat matrix = cv::getPerspectiveTransform(srcPoints, dstPoints);
    cv::warpPerspective(edges, warped, matrix, cv::Size(400, 600));
    return warped;
}

double LaneDetector::calculateCurveAngle(const cv::Mat& warpedEdges) {
    int topSumX = 0, topCount = 0;
    int bottomSumX = 0, bottomCount = 0;
    int midY = warpedEdges.rows / 2;

    for (int y = 0; y < warpedEdges.rows; y++) {
        for (int x = 0; x < warpedEdges.cols; x++) {
            if (warpedEdges.at<uchar>(y, x) > 128) {
                if (y < midY) {
                    topSumX += x;
                    topCount++;
                }
                else {
                    bottomSumX += x;
                    bottomCount++;
                }
            }
        }
    }

    if (topCount == 0 || bottomCount == 0) return 0.0;

    int topAverageX = topSumX / topCount;
    int bottomAverageX = bottomSumX / bottomCount;

    double shiftX = topAverageX - bottomAverageX;
    double sensitivity = 0.5;
    return shiftX * sensitivity;
}