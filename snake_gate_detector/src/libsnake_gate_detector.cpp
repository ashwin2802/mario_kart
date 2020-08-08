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

void SnakeGateDetector::findGates() {
    int num_gates = 0;
    for (int i = 0; i < (frame_.size().height) * (frame_.size().height); i++) {
        cv::Point P0 = randomPoint(frame_.size().width, frame_.size().height);

        if (isTargetColor(P0)) {
            cv::Point* P12 = searchUpDown(P0);

            if (norm(P12[0], P12[1]) > length_threshold_) {
                cv::Point P3 = searchLeft(P12[0]);
                cv::Point P4 = searchLeft(P12[1]);

                if (norm(P12[0], P3) > length_threshold_ || norm(P12[1], P4) > length_threshold_) {
                    cv::Point* S = findMinimalSquare(P12[0], P12[1], P3, P4);
                    cv::Point* detected_gate = refineCorner(S[0], S[1], S[2], S[3]);

                    if (colorFitness(detected_gate) > color_fitness_threshold_)
                        detected_gates_.push_back(detected_gate);

                    num_gates++;
                    if (num_gates == max_gates_)
                        break;
                    continue;
                }

                P3 = searchRight(P12[0]);
                P4 = searchRight(P12[1]);

                if (norm(P12[0], P3) > length_threshold_ || norm(P12[1], P4) > length_threshold_) {
                    cv::Point* S = findMinimalSquare(P12[0], P12[1], P3, P4);
                    cv::Point* detected_gate = refineCorner(S[0], S[1], S[2], S[3]);

                    if (colorFitness(detected_gate) > color_fitness_threshold_)
                        detected_gates_.push_back(detected_gate);

                    num_gates++;
                    if (num_gates == max_gates_)
                        break;
                }
            }
        }
    }
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

cv::Point* SnakeGateDetector::searchUpDown(cv::Point& P) {
    cv::Point* P12 = new cv::Point[2];

    P12[0] = P;
    P12[1] = P;

    while (1) {
        if (isTargetColor(cv::Point(P12[0].x, P12[0].y - 1))) {
            P12[0].y--;
        } else if (isTargetColor(cv::Point(P12[0].x - 1, P12[0].y - 1))) {
            P12[0].x--;
            P12[0].y--;
        } else if (isTargetColor(cv::Point(P12[0].x + 1, P12[0].y - 1))) {
            P12[0].x++;
            P12[0].y--;
        } else {
            break;
        }
    }

    while (1) {
        if (isTargetColor(cv::Point(P12[1].x, P12[1].y + 1))) {
            P12[1].y++;
        } else if (isTargetColor(cv::Point(P12[1].x - 1, P12[1].y + 1))) {
            P12[1].x--;
            P12[1].y++;
        } else if (isTargetColor(cv::Point(P12[1].x + 1, P12[1].y + 1))) {
            P12[1].x++;
            P12[1].y++;
        } else {
            break;
        }
    }

    return P12;
}

cv::Point SnakeGateDetector::searchLeft(cv::Point& P) {
    cv::Point P_left = P;

    while (1) {
        if (isTargetColor(cv::Point(P_left.x - 1, P_left.y))) {
            P_left.x--;
        } else if (isTargetColor(cv::Point(P_left.x - 1, P_left.y - 1))) {
            P_left.y--;
            P_left.x--;
        } else if (isTargetColor(cv::Point(P_left.x - 1, P_left.y + 1))) {
            P_left.y++;
            P_left.x--;
        } else {
            break;
        }
    }

    return P_left;
}

cv::Point SnakeGateDetector::searchRight(cv::Point& P) {
    cv::Point P_right = P;

    while (1) {
        if (isTargetColor(cv::Point(P_right.x + 1, P_right.y))) {
            P_right.x++;
        } else if (isTargetColor(cv::Point(P_right.x + 1, P_right.y - 1))) {
            P_right.y--;
            P_right.x++;
        } else if (isTargetColor(cv::Point(P_right.x + 1, P_right.y + 1))) {
            P_right.y++;
            P_right.x++;
        } else {
            break;
        }
    }

    return P_right;
}



}  // namespace snake_gate_detector
