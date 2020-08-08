#include <snake_gate_detector/libsnake_gate_detector.hpp>

namespace snake_gate_detector {

void SnakeGateDetector::setLengthThreshold(int length_threshold) {
    length_threshold_ = length_threshold;
}

void SnakeGateDetector::setColorFitnessThreshold(int color_fitness) {
    color_fitness_threshold_ = color_fitness;
}

void SnakeGateDetector::setHSVThreshold(cv::Vec3b& upper, cv::Vec3b& lower) {
    hsv_threshold_lower_ = lower;
    hsv_threshold_upper_ = upper;
}

void SnakeGateDetector::setMaxGates(int max_gates) {
    max_gates_ = max_gates;
}

void SnakeGateDetector::setImageFrame(cv::Mat frame) {
    frame_ = frame;
}

cv::Point SnakeGateDetector::randomPoint(int width, int height) {
    srand(time(0));
    return cv::Point(rand() % width, rand() % height);
}

bool SnakeGateDetector::isTargetColor(const cv::Point& P) {
    cv::Vec3b pixel = frame_.at<cv::Vec3b>(P);

    bool lower_bound =
        hsv_threshold_lower_.val[0] <= pixel.val[0] && hsv_threshold_lower_.val[1] <= pixel.val[1] && hsv_threshold_lower_.val[2] <= pixel.val[2];
    bool upper_bound =
        hsv_threshold_upper_.val[0] >= pixel.val[0] && hsv_threshold_upper_.val[1] >= pixel.val[1] && hsv_threshold_upper_.val[2] >= pixel.val[2];

    return lower_bound && upper_bound;
}

double SnakeGateDetector::norm(cv::Point& P1, cv::Point& P2) {
    return sqrt(pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2));
}

}  // namespace snake_gate_detector
