#include <snake_gate_detector/snake_gate_detector.hpp>

namespace snake_gate_detector {

cv::Point SnakeGateDetector::randomPoint(int& width, int& height) {
    srand(time(0));
    return cv::Point(rand() % width, rand() % height);
}

bool SnakeGateDetector::isTargetColor(cv::Point& P) {
    cv::Vec3b pixel = frame.at<cv::Vec3b>(P);

    bool lower_bound = hsv_threshold_lower.val[0] <= pixel.val[0] && hsv_threshold_lower.val[1] <= pixel.val[1] && hsv_threshold_lower.val[2] <= pixel.val[2];
    bool upper_bound = hsv_threshold_upper.val[0] >= pixel.val[0] && hsv_threshold_upper.val[1] >= pixel.val[1] && hsv_threshold_upper.val[2] >= pixel.val[2];

    return lower_bound && upper_bound;
}

double SnakeGateDetector::Norm(cv::Point& P1, cv::Point& P2) {
    return sqrt(pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2));
}

}  // namespace snake_gate_detector