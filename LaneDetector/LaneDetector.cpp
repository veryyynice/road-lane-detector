#include "LaneDetector.h"
#include <iostream>

LaneDetector::LaneDetector() {
    // We will likely need to adjust these after you see the red box on the screen!
    srcPoints = {


        // cv::Point2f(513, 404),  // 1. Top-Left
        //cv::Point2f(739, 407),  // 2. Top-Right
        //cv::Point2f(5, 654),    // 3. Bottom-Left
        //cv::Point2f(1269, 661)  // 4. Bottom-Right

        cv::Point2f(371, 401),
        cv::Point2f(744, 408),
        cv::Point2f(7, 564),
        cv::Point2f(1276, 571),
    };

    dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(400, 0),
        cv::Point2f(0, 600),
        cv::Point2f(400, 600)
    };
}

double LaneDetector::processFrame(cv::Mat& inputFrame, cv::Mat& debugView, cv::Mat& debugEdges) {
    cv::Mat edges = applyHSVMask(inputFrame);
    edges.copyTo(debugEdges);

    // Draw the red trapezoid clockwise
    std::vector<cv::Point> intPoints;
    intPoints.push_back(cv::Point((int)srcPoints[0].x, (int)srcPoints[0].y));
    intPoints.push_back(cv::Point((int)srcPoints[1].x, (int)srcPoints[1].y));
    intPoints.push_back(cv::Point((int)srcPoints[3].x, (int)srcPoints[3].y));
    intPoints.push_back(cv::Point((int)srcPoints[2].x, (int)srcPoints[2].y));
    cv::polylines(inputFrame, intPoints, true, cv::Scalar(0, 0, 255), 2);

    cv::Mat warpedEdges = getBirdsEyeView(edges);

    // --- NEW: Convert grayscale warped edges to BGR so we can draw colored boxes on it ---
    cv::cvtColor(warpedEdges, debugView, cv::COLOR_GRAY2BGR);

    // Pass the colored debugView into the curve calculator
    return calculateCurveAngle(warpedEdges, debugView);
}

double LaneDetector::calculateCurveAngle(const cv::Mat& warpedEdges, cv::Mat& debugView) {
    std::vector<cv::Point> nonZeroPts;
    cv::findNonZero(warpedEdges, nonZeroPts);

    std::vector<int> histogram(warpedEdges.cols, 0);
    for (const auto& pt : nonZeroPts) {
        if (pt.y > warpedEdges.rows / 2) {
            histogram[pt.x]++;
        }
    }

    // --- THE FIX: Ignore the outer 40 pixels where border artifacts live ---
    int midPoint = warpedEdges.cols / 2;
    int ignoreMargin = 40;

    // Find Left Peak (search only from x=40 to x=200)
    auto leftStart = histogram.begin() + ignoreMargin;
    auto leftEnd = histogram.begin() + midPoint;
    int leftBaseX = (int)std::distance(histogram.begin(), std::max_element(leftStart, leftEnd));

    // Find Right Peak (search only from x=200 to x=360)
    auto rightStart = histogram.begin() + midPoint;
    auto rightEnd = histogram.end() - ignoreMargin;
    int rightBaseX = (int)std::distance(histogram.begin(), std::max_element(rightStart, rightEnd));

    // WIDENED WINDOWS: Increased margin to 50 to track sharper curves
    int numWindows = 9;
    int windowHeight = warpedEdges.rows / numWindows;
    int margin = 50;
    int minPix = 15;

    int leftCurrentX = leftBaseX;
    int rightCurrentX = rightBaseX;

    std::vector<cv::Point> leftLanePts;
    std::vector<cv::Point> rightLanePts;

    for (int window = 0; window < numWindows; window++) {
        int winY_top = warpedEdges.rows - (window + 1) * windowHeight;
        int winY_bottom = warpedEdges.rows - window * windowHeight;

        int winX_left_min = leftCurrentX - margin;
        int winX_left_max = leftCurrentX + margin;
        int winX_right_min = rightCurrentX - margin;
        int winX_right_max = rightCurrentX + margin;

        cv::rectangle(debugView, cv::Point(winX_left_min, winY_bottom), cv::Point(winX_left_max, winY_top), cv::Scalar(0, 255, 0), 2);
        cv::rectangle(debugView, cv::Point(winX_right_min, winY_bottom), cv::Point(winX_right_max, winY_top), cv::Scalar(0, 0, 255), 2);

        int leftPixCount = 0;
        int leftPixSumX = 0;
        int rightPixCount = 0;
        int rightPixSumX = 0;

        for (const auto& pt : nonZeroPts) {
            if (pt.y >= winY_top && pt.y < winY_bottom) {
                if (pt.x >= winX_left_min && pt.x < winX_left_max) {
                    leftLanePts.push_back(pt);
                    leftPixSumX += pt.x;
                    leftPixCount++;
                }
                if (pt.x >= winX_right_min && pt.x < winX_right_max) {
                    rightLanePts.push_back(pt);
                    rightPixSumX += pt.x;
                    rightPixCount++;
                }
            }
        }

        if (leftPixCount > minPix) leftCurrentX = leftPixSumX / leftPixCount;
        if (rightPixCount > minPix) rightCurrentX = rightPixSumX / rightPixCount;
    }

    // 5. Fit Polynomials
    if (leftLanePts.size() > 10 && rightLanePts.size() > 10) {
        cv::Mat leftFit = polyfit(leftLanePts);
        cv::Mat rightFit = polyfit(rightLanePts);

        std::vector<cv::Point> leftCurve, rightCurve;
        for (int y = 0; y < warpedEdges.rows; y += 5) {
            double lX = leftFit.at<double>(0, 0) * y * y + leftFit.at<double>(1, 0) * y + leftFit.at<double>(2, 0);
            double rX = rightFit.at<double>(0, 0) * y * y + rightFit.at<double>(1, 0) * y + rightFit.at<double>(2, 0);
            leftCurve.push_back(cv::Point((int)lX, y));
            rightCurve.push_back(cv::Point((int)rX, y));
        }
        cv::polylines(debugView, leftCurve, false, cv::Scalar(0, 255, 255), 3);
        cv::polylines(debugView, rightCurve, false, cv::Scalar(0, 255, 255), 3);

        // 6. Calculate Steering Angle (THE FIXED MATH)
        // Evaluate the polynomial at the BOTTOM of the screen (where the car is)
        int y_bottom = warpedEdges.rows;
        double bottomLeftX = leftFit.at<double>(0, 0) * y_bottom * y_bottom + leftFit.at<double>(1, 0) * y_bottom + leftFit.at<double>(2, 0);
        double bottomRightX = rightFit.at<double>(0, 0) * y_bottom * y_bottom + rightFit.at<double>(1, 0) * y_bottom + rightFit.at<double>(2, 0);
        double bottomMidX = (bottomLeftX + bottomRightX) / 2.0;

        // Evaluate the polynomial a quarter of the way down from the TOP
        int y_top = warpedEdges.rows / 4;
        double topLeftX = leftFit.at<double>(0, 0) * y_top * y_top + leftFit.at<double>(1, 0) * y_top + leftFit.at<double>(2, 0);
        double topRightX = rightFit.at<double>(0, 0) * y_top * y_top + rightFit.at<double>(1, 0) * y_top + rightFit.at<double>(2, 0);
        double topMidX = (topLeftX + topRightX) / 2.0;

        // THE FIX: Swap the order to invert the angle!
        // Now, a right turn (topMidX > bottomMidX) results in a negative angle.
        // OpenCV turns clockwise for negative angles.
        double shift = bottomMidX - topMidX;

        return shift * 0.4;
    }

    return 0.0;
}

// Helper Function: Least Squares solver for x = Ay^2 + By + C
cv::Mat LaneDetector::polyfit(const std::vector<cv::Point>& pts) {
    // Cast to int once to silence the size_t warnings
    int n = (int)pts.size();

    cv::Mat X(n, 3, CV_64F);
    cv::Mat Y(n, 1, CV_64F);

    for (int i = 0; i < n; i++) {
        X.at<double>(i, 0) = pts[i].y * pts[i].y;
        X.at<double>(i, 1) = pts[i].y;
        X.at<double>(i, 2) = 1.0;
        Y.at<double>(i, 0) = pts[i].x;
    }

    cv::Mat coeffs;
    cv::solve(X, Y, coeffs, cv::DECOMP_SVD);
    return coeffs;
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

