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
            cv::Point P[4];
            searchUpDown(P0, P);

            if (norm(P[0], P[1]) > length_threshold_) {
                searchLeft(P[0], P + 2);
                searchLeft(P[1], P + 3);

                if (norm(P[0], P[2]) > length_threshold_ || norm(P[1], P[3]) > length_threshold_) {
                    cv::Point S[4];
                    cv::Point* detected_gate = new cv::Point[4];
                    findMinimalSquare(P, S);
                    refineCorner(S, detected_gate);

                    if (colorFitness(detected_gate) > color_fitness_threshold_) {
                        detected_gates_.push_back(detected_gate);
                        num_gates++;
                        if (num_gates == max_gates_) {
                            break;
                        }
                        continue;
                    } else {
                        delete[] detected_gate;
                    }
                }

                searchRight(P[0], P+2);
                searchRight(P[1], P+3);

                if (norm(P[0], P[2]) > length_threshold_ || norm(P[1], P[3]) > length_threshold_) {
                    cv::Point S[4];
                    cv::Point* detected_gate = new cv::Point[4];
                    findMinimalSquare(P, S);
                    refineCorner(S, detected_gate);

                    if (colorFitness(detected_gate) > color_fitness_threshold_) {
                        detected_gates_.push_back(detected_gate);
                        num_gates++;
                        if (num_gates == max_gates_) {
                            break;
                        }
                        continue;
                    } else {
                        delete[] detected_gate;
                    }
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

void SnakeGateDetector::searchUpDown(cv::Point& P0, cv::Point* P) {
    P[0] = P0;
    P[1] = P0;

    while (1) {
        if (isTargetColor(cv::Point(P[0].x, P[0].y - 1))) {
        P[0].y--;
        } else if (isTargetColor(cv::Point(P[0].x - 1, P[0].y - 1))) {
            P[0].x--;
            P[0].y--;
        } else if (isTargetColor(cv::Point(P[0].x + 1, P[0].y - 1))) {
            P[0].x++;
            P[0].y--;
        } else {
            break;
        }
    }

    while (1) {
        if (isTargetColor(cv::Point(P[1].x, P[1].y + 1))) {
            P[1].y++;
        } else if (isTargetColor(cv::Point(P[1].x - 1, P[1].y + 1))) {
            P[1].x--;
            P[1].y++;
        } else if (isTargetColor(cv::Point(P[1].x + 1, P[1].y + 1))) {
            P[1].x++;
            P[1].y++;
        } else {
            break;
        }
    }

}

void SnakeGateDetector::searchLeft(cv::Point& P0, cv::Point* P) {
    P[0] = P0;

    while (1) {
        if (isTargetColor(cv::Point(P[0].x - 1, P[0].y))) {
            P[0].x--;
        } else if (isTargetColor(cv::Point(P[0].x - 1, P[0].y - 1))) {
            P[0].y--;
            P[0].x--;
        } else if (isTargetColor(cv::Point(P[0].x - 1, P[0].y + 1))) {
            P[0].y++;
            P[0].x--;
        } else {
            break;
        }
    }

}

void SnakeGateDetector::searchRight(cv::Point& P0, cv::Point* P) {
    P[0] = P0;

    while (1) {
        if (isTargetColor(cv::Point(P[0].x + 1, P[0].y))) {
            P[0].x++;
        } else if (isTargetColor(cv::Point(P[0].x + 1, P[0].y - 1))) {
            P[0].y--;
            P[0].x++;
        } else if (isTargetColor(cv::Point(P[0].x + 1, P[0].y + 1))) {
            P[0].y++;
            P[0].x++;
        } else {
            break;
        }
    }

}

// void SnakeGateDetector::findMinimalSquare(const cv::Point* P_in, cv::Point* P_out);

}  // namespace snake_gate_detector
