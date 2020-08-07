#include <snake_gate_detector/snake_gate_detector.hpp>

namespace snake_gate_detector {

cv::Point SnakeGateDetector::randomPoint(int& width, int& height) {
    srand(time(0));
    return cv::Point(rand() % width, rand() % height);
}
}