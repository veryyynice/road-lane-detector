// LaneDetector.cpp
// CSS 487 - Final Project: Lane Detector
// Author: Ruslan and Tom
// Date: Jun 5 2026
//

// Implements the full lane detection pipeline.
// Each frame goes through: HSV color masking, canny edge detection,
// perspective warp to a bird's eye view, sliding window lane search,
// 2nd degree polynomial fit, and finally a steering angle calculation.

#include "LaneDetector.h"
#include <iostream>

// LaneDetector (constructor)
// hardcodes the 4 warp source points for the test dashcam video
// these were picked by pausing the video and clicking with the mouse callback
// Preconditions:  none
// Postconditions: srcPoints and dstPoints are ready for getPerspectiveTransform
LaneDetector::LaneDetector() {

    srcPoints = {
        // previous calibration attempt, kept for reference
        // cv::Point2f(513, 404),  // 1. Top-Left
        // cv::Point2f(739, 407),  // 2. Top-Right
        // cv::Point2f(5, 654),    // 3. Bottom-Left
        // cv::Point2f(1269, 661)  // 4. Bottom-Right

        cv::Point2f(371, 401),  // 1. Top-Left
        cv::Point2f(744, 408),  // 2. Top-Right
        cv::Point2f(7, 564),    // 3. Bottom-Left
        cv::Point2f(1276, 571), // 4. Bottom-Right
    };

    // flat destination rectangle — all lane math happens in this 400x600 space
    dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(400, 0),
        cv::Point2f(0, 600),
        cv::Point2f(400, 600)
    };
}

// processFrame
// top-level pipeline call — runs all stages and returns the steering angle
// Preconditions:  inputFrame is a valid BGR 1280x720 image
// Postconditions: draws red ROI trapezoid onto inputFrame in-place, fills debugView with
//                 the annotated bird's eye image, fills debugEdges with the canny mask,
//                 returns steering angle in degrees (positive = left curve, negative = right)
double LaneDetector::processFrame(cv::Mat& inputFrame, cv::Mat& debugView, cv::Mat& debugEdges) {
    cv::Mat edges = applyHSVMask(inputFrame);
    edges.copyTo(debugEdges); // save the edge mask before warping so the debug window shows it flat

    // draw the red trapezoid so you can see exactly what region is being warped
    // points go clockwise: top-left, top-right, bottom-right, bottom-left
    std::vector<cv::Point> intPoints;
    intPoints.push_back(cv::Point((int)srcPoints[0].x, (int)srcPoints[0].y));
    intPoints.push_back(cv::Point((int)srcPoints[1].x, (int)srcPoints[1].y));
    intPoints.push_back(cv::Point((int)srcPoints[3].x, (int)srcPoints[3].y));
    intPoints.push_back(cv::Point((int)srcPoints[2].x, (int)srcPoints[2].y));
    cv::polylines(inputFrame, intPoints, true, cv::Scalar(0, 0, 255), 2);

    cv::Mat warpedEdges = getBirdsEyeView(edges);

    // convert to BGR so we can draw colored boxes and curves on the debug view
    cv::cvtColor(warpedEdges, debugView, cv::COLOR_GRAY2BGR);

    return calculateCurveAngle(warpedEdges, debugView);
}

// calculateCurveAngle
// finds the left and right lane lines using a sliding window search, fits a polynomial
// to each, then computes how much the road curves by comparing the midpoint near the
// car vs the midpoint farther ahead
// Preconditions:  warpedEdges is a 400x600 grayscale binary edge image;
//                 debugView is a BGR image the same size
// Postconditions: draws sliding windows and yellow curves onto debugView,
//                 returns steering angle in degrees, 0 if lanes not found
double LaneDetector::calculateCurveAngle(const cv::Mat& warpedEdges, cv::Mat& debugView) {
    std::vector<cv::Point> nonZeroPts;
    cv::findNonZero(warpedEdges, nonZeroPts); // collect all edge pixels at once so we don't scan per window

    // build a histogram of edge pixel x-positions in the bottom half of the image
    // the two peaks tell us where each lane starts at the bottom
    std::vector<int> histogram(warpedEdges.cols, 0);
    for (const auto& pt : nonZeroPts) {
        if (pt.y > warpedEdges.rows / 2) {
            histogram[pt.x]++;
        }
    }

    // ignore the outer 40 pixels — the warp creates bright border artifacts there
    // that would pull the histogram peak to the wrong spot
    int midPoint = warpedEdges.cols / 2;
    int ignoreMargin = 40;

    // left peak: search left half only (x=40 to x=200)
    auto leftStart = histogram.begin() + ignoreMargin;
    auto leftEnd   = histogram.begin() + midPoint;
    int leftBaseX  = (int)std::distance(histogram.begin(), std::max_element(leftStart, leftEnd));

    // right peak: search right half only (x=200 to x=360)
    auto rightStart = histogram.begin() + midPoint;
    auto rightEnd   = histogram.end() - ignoreMargin;
    int rightBaseX  = (int)std::distance(histogram.begin(), std::max_element(rightStart, rightEnd));

    int numWindows   = 9;                                  // how many windows to stack vertically
    int windowHeight = warpedEdges.rows / numWindows;      // each window covers this many rows
    int margin       = 50;   // half-width of each window; widened to 50 to track sharper curves
    int minPix       = 15;   // minimum pixels needed before we re-center the window

    int leftCurrentX  = leftBaseX;
    int rightCurrentX = rightBaseX;

    std::vector<cv::Point> leftLanePts;
    std::vector<cv::Point> rightLanePts;

    // walk from the bottom up, one window at a time
    for (int window = 0; window < numWindows; window++) {
        int winY_top    = warpedEdges.rows - (window + 1) * windowHeight;
        int winY_bottom = warpedEdges.rows - window * windowHeight;

        int winX_left_min  = leftCurrentX  - margin;
        int winX_left_max  = leftCurrentX  + margin;
        int winX_right_min = rightCurrentX - margin;
        int winX_right_max = rightCurrentX + margin;

        // draw the search windows on the debug view so you can see them
        cv::rectangle(debugView, cv::Point(winX_left_min,  winY_bottom), cv::Point(winX_left_max,  winY_top), cv::Scalar(0, 255, 0), 2); // green = left
        cv::rectangle(debugView, cv::Point(winX_right_min, winY_bottom), cv::Point(winX_right_max, winY_top), cv::Scalar(0, 0, 255), 2); // red = right

        int leftPixCount  = 0;
        int leftPixSumX   = 0;
        int rightPixCount = 0;
        int rightPixSumX  = 0;

        // scan all edge pixels and collect the ones that fall inside this window
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

        // re-center the next window on the mean x of found pixels, but only if
        // we found enough pixels — otherwise we'd just chase noise
        if (leftPixCount  > minPix) leftCurrentX  = leftPixSumX  / leftPixCount;
        if (rightPixCount > minPix) rightCurrentX = rightPixSumX / rightPixCount;
    }

    // need at least 10 points on each side or the polyfit will be garbage
    if (leftLanePts.size() > 10 && rightLanePts.size() > 10) {
        cv::Mat leftFit  = polyfit(leftLanePts);
        cv::Mat rightFit = polyfit(rightLanePts);

        // evaluate the polynomials at every y to draw the smooth curves
        std::vector<cv::Point> leftCurve, rightCurve;
        for (int y = 0; y < warpedEdges.rows; y += 5) {
            double lX = leftFit.at<double>(0, 0)  * y * y + leftFit.at<double>(1, 0)  * y + leftFit.at<double>(2, 0);
            double rX = rightFit.at<double>(0, 0) * y * y + rightFit.at<double>(1, 0) * y + rightFit.at<double>(2, 0);
            leftCurve.push_back(cv::Point((int)lX, y));
            rightCurve.push_back(cv::Point((int)rX, y));
        }
        cv::polylines(debugView, leftCurve,  false, cv::Scalar(0, 255, 255), 3); // yellow
        cv::polylines(debugView, rightCurve, false, cv::Scalar(0, 255, 255), 3);

        // steering angle: compare the lane midpoint near the car (bottom) vs far ahead (top quarter)
        // shift = how much the midpoint moves from bottom to top — positive means curving left
        int y_bottom = warpedEdges.rows;
        double bottomLeftX  = leftFit.at<double>(0, 0)  * y_bottom * y_bottom + leftFit.at<double>(1, 0)  * y_bottom + leftFit.at<double>(2, 0);
        double bottomRightX = rightFit.at<double>(0, 0) * y_bottom * y_bottom + rightFit.at<double>(1, 0) * y_bottom + rightFit.at<double>(2, 0);
        double bottomMidX   = (bottomLeftX + bottomRightX) / 2.0;

        int y_top = warpedEdges.rows / 4;
        double topLeftX  = leftFit.at<double>(0, 0)  * y_top * y_top + leftFit.at<double>(1, 0)  * y_top + leftFit.at<double>(2, 0);
        double topRightX = rightFit.at<double>(0, 0) * y_top * y_top + rightFit.at<double>(1, 0) * y_top + rightFit.at<double>(2, 0);
        double topMidX   = (topLeftX + topRightX) / 2.0;

        // subtraction order matters: bottom - top gives positive for left curves,
        // negative for right curves. OpenCV rotates clockwise for negative angles, which
        // is correct — right curve should turn the wheel clockwise.
        double shift = bottomMidX - topMidX;

        return shift * 0.4; // scale down so the wheel doesn't spin too wildly
    }

    return 0.0; // no lanes found this frame
}

// polyfit
// least squares 2nd degree polynomial fit: x = Ay^2 + By + C
// uses y as the input because lane lines run vertically in the bird's eye view —
// fitting x = f(y) instead of y = f(x) avoids the vertical line singularity
// SVD is used instead of normal equations because SVD is stable even when
// the points are nearly collinear
// Preconditions:  pts has at least 3 points
// Postconditions: returns a 3x1 Mat<double> of coefficients [A, B, C]
cv::Mat LaneDetector::polyfit(const std::vector<cv::Point>& pts) {
    int n = (int)pts.size(); // cast once to avoid size_t warnings in the loop

    // build the Vandermonde matrix X where each row is [y^2, y, 1]
    // and Y is the corresponding x value we're trying to predict
    cv::Mat X(n, 3, CV_64F);
    cv::Mat Y(n, 1, CV_64F);

    for (int i = 0; i < n; i++) {
        X.at<double>(i, 0) = pts[i].y * pts[i].y; // y^2 term
        X.at<double>(i, 1) = pts[i].y;             // y term
        X.at<double>(i, 2) = 1.0;                  // constant term
        Y.at<double>(i, 0) = pts[i].x;             // what we want to predict
    }

    cv::Mat coeffs;
    cv::solve(X, Y, coeffs, cv::DECOMP_SVD); // SVD minimizes least-squares error stably
    return coeffs;
}

// applyHSVMask
// isolates white and yellow lane pixels by converting to HSV and thresholding,
// then runs gaussian blur and canny to get clean edges
// Preconditions:  frame is a valid BGR image
// Postconditions: returns a binary grayscale canny edge image the same size as frame,
//                 with only edges along white/yellow regions kept
cv::Mat LaneDetector::applyHSVMask(const cv::Mat& frame) {
    cv::Mat hsv, whiteMask, yellowMask, finalMask, edges;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV); // HSV separates color from brightness

    // white: any hue, low saturation (it's washed out), high brightness
    // V floor is 150 instead of 200 so we still catch lane lines in shadow
    cv::inRange(hsv, cv::Scalar(0, 0, 150),    cv::Scalar(180, 50, 255),  whiteMask);

    // yellow: narrow hue band around yellow (15-35), vivid, bright
    cv::inRange(hsv, cv::Scalar(15, 100, 100), cv::Scalar(35, 255, 255),  yellowMask);

    cv::bitwise_or(whiteMask, yellowMask, finalMask); // keep pixels that match either color

    // blur before canny so isolated noise pixels don't produce false edges
    cv::GaussianBlur(finalMask, finalMask, cv::Size(5, 5), 0);

    // thresholds 50/150: strong edges above 150, weak edges 50-150 kept only if
    // connected to a strong edge, anything below 50 discarded
    cv::Canny(finalMask, edges, 50, 150);

    return edges;
}

// getBirdsEyeView
// applies the perspective warp to flatten the road into a top-down view
// Preconditions:  edges is a grayscale image; srcPoints and dstPoints are set in constructor
// Postconditions: returns a 400x600 warped grayscale image
cv::Mat LaneDetector::getBirdsEyeView(const cv::Mat& edges) {
    cv::Mat warped;
    cv::Mat matrix = cv::getPerspectiveTransform(srcPoints, dstPoints);
    cv::warpPerspective(edges, warped, matrix, cv::Size(400, 600));
    return warped;
}
