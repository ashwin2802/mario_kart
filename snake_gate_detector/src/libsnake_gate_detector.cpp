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
    static bool if_first_frame = true;
    if (if_first_frame) {
        if_first_frame = false;
        for (int i = 0; i < (frame.size().width) * (frame.size().height); i++) {
            random_sample_.push_back(i);
        }
    }

    for (int i = detected_gates_.size() - 1; i >= 0; i--) {
        delete[] detected_gates_.at(i);
        detected_gates_.pop_back();
    }

    frame_ = frame;
    if (isTestImage) {
        frame_.copyTo(gate_progress_image_);
    }
    randomize();
    findGates();
}

std::vector<cv::Point*>* SnakeGateDetector::getDetectedGates() {
    return &detected_gates_;
}

cv::Mat SnakeGateDetector::getFrameWithGates() {
    return frame_;
}

void SnakeGateDetector::findGates() {
    int num_gates = 0;
    isGateFound = false;
    for (int i = 0; i < (frame_.size().width) * (frame_.size().height); i++) {
        std::cout << i << "th elem" << std::endl; 
        cv::Point P0 = cv::Point(random_sample_.at(i) % frame_.size().width, random_sample_.at(i) / frame_.size().width);

        if (isTestImage && !isGateFound) {
            cv::Mat img;
            frame_.copyTo(img);
            cv::circle(img, P0, 10, cv::Scalar(0, 0, 0), -1);
            cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/find" + std::to_string(i) + ".jpg", img);
        }

        if (isTargetColor(P0)) {
            std::cout << "Found First Point of Potential Gate: " << P0.x << " " << P0.y << std::endl;

            cv::Point P[4];

            if (isTestImage) {
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD/point_found.jpg", gate_progress_image_);
            }

            searchUpDown(P0, P);

            if (norm(P[0], P[1]) > length_threshold_) {
                if (isTestImage) {
                    gate_progress_image_.copyTo(gate_progress_image_copy_);
                }

                searchLeft(P[0], P + 2);
                searchLeft(P[1], P + 3);

                if (norm(P[0], P[2]) > length_threshold_ || norm(P[1], P[3]) > length_threshold_) {
                    // cv::Point S[4];
                    cv::Point* detected_gate = new cv::Point[4];
                    // findMinimalSquare(P, S);
                    // refineCorner(S, detected_gate);
                    detected_gate[0] = P[0];
                    detected_gate[1] = P[1];
                    detected_gate[2] = P[2];
                    detected_gate[3] = P[3];

                    if (/*colorFitness(detected_gate) > color_fitness_threshold_*/ 1) {
                        detected_gates_.push_back(detected_gate);
                        cv::line(frame_, detected_gate[0], detected_gate[1], cv::Scalar(0, 0, 128), 5);
                        cv::line(frame_, detected_gate[0], detected_gate[2], cv::Scalar(0, 0, 128), 5);
                        cv::line(frame_, detected_gate[1], detected_gate[3], cv::Scalar(0, 0, 128), 5);
                        cv::line(frame_, detected_gate[2], detected_gate[3], cv::Scalar(0, 0, 128), 5);
                        cv::circle (frame_, detected_gate[0], 15, cv::Scalar(47, 152, 89), 5);
                        cv::circle (frame_, detected_gate[1], 15, cv::Scalar(47, 152, 89), 5);
                        cv::circle (frame_, detected_gate[2], 15, cv::Scalar(47, 152, 89), 5);
                        cv::circle (frame_, detected_gate[3], 15, cv::Scalar(47, 152, 89), 5);

                        num_gates++;
                        isGateFound = true;
                        if (num_gates == max_gates_) {
                            break;
                        }
                        continue;
                    } else {
                        delete[] detected_gate;
                    }
                }

                searchRight(P[0], P + 2);
                searchRight(P[1], P + 3);

                if (norm(P[0], P[2]) > length_threshold_ || norm(P[1], P[3]) > length_threshold_) {
                    // cv::Point S[4];
                    cv::Point* detected_gate = new cv::Point[4];
                    // findMinimalSquare(P, S);
                    // refineCorner(S, detected_gate);
                    detected_gate[0] = P[0];
                    detected_gate[1] = P[1];
                    detected_gate[2] = P[2];
                    detected_gate[3] = P[3];

                    if (/*colorFitness(detected_gate) > color_fitness_threshold_*/ 1) {
                        detected_gates_.push_back(detected_gate);
                        cv::line(frame_, detected_gate[0], detected_gate[1], cv::Scalar(0, 0, 128), 5);
                        cv::line(frame_, detected_gate[0], detected_gate[2], cv::Scalar(0, 0, 128), 5);
                        cv::line(frame_, detected_gate[1], detected_gate[3], cv::Scalar(0, 0, 128), 5);
                        cv::line(frame_, detected_gate[2], detected_gate[3], cv::Scalar(0, 0, 128), 5);
                        cv::circle (frame_, detected_gate[0], 15, cv::Scalar(47, 152, 89), 5);
                        cv::circle (frame_, detected_gate[1], 15, cv::Scalar(47, 152, 89), 5);
                        cv::circle (frame_, detected_gate[2], 15, cv::Scalar(47, 152, 89), 5);
                        cv::circle (frame_, detected_gate[3], 15, cv::Scalar(47, 152, 89), 5);                        num_gates++;
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

void SnakeGateDetector::randomize() {
    srand(unsigned(time(0)));
    std::random_shuffle(random_sample_.begin(), random_sample_.end());
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

    int num_points = 0;

    while (1) {
        if (isTargetColor(cv::Point(P[0].x, P[0].y - 1))) {
            P[0].y--;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/up" + std::to_string(++num_points) + ".jpg", gate_progress_image_);
            }
        } else if (isTargetColor(cv::Point(P[0].x - 1, P[0].y - 1))) {
            P[0].x--;
            P[0].y--;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/up" + std::to_string(++num_points) + ".jpg", gate_progress_image_);
            }
        } else if (isTargetColor(cv::Point(P[0].x + 1, P[0].y - 1))) {
            P[0].x++;
            P[0].y--;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/up" + std::to_string(++num_points) + ".jpg", gate_progress_image_);
            }
        } else {
            cv::circle(gate_progress_image_, P[0], 15, cv::Scalar(47, 152, 89), 5);
            cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD/topfound.jpg", gate_progress_image_);
            break;
        }
    }

    while (1) {
        if (isTargetColor(cv::Point(P[1].x, P[1].y + 1))) {
            P[1].y++;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[1], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/down" + std::to_string(++num_points) + ".jpg", gate_progress_image_);
            }
        } else if (isTargetColor(cv::Point(P[1].x - 1, P[1].y + 1))) {
            P[1].x--;
            P[1].y++;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[1], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/down" + std::to_string(++num_points) + ".jpg", gate_progress_image_);
            }
        } else if (isTargetColor(cv::Point(P[1].x + 1, P[1].y + 1))) {
            P[1].x++;
            P[1].y++;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[1], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/down" + std::to_string(++num_points) + ".jpg", gate_progress_image_);
            }
        } else {
            cv::circle(gate_progress_image_, P[1], 15, cv::Scalar(47, 152, 89), 5);
            cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD/bottomfound.jpg", gate_progress_image_);
            break;
        }
    }
}

void SnakeGateDetector::searchLeft(cv::Point& P0, cv::Point* P) {
    P[0] = P0;

    int num_points = 0;

    while (1) {
        if (isTargetColor(cv::Point(P[0].x - 1, P[0].y))) {
            P[0].x--;
            if (isTestImage) {
                cv::circle(gate_progress_image_copy_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/left_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_" +
                                std::to_string(++num_points) + ".jpg",
                    gate_progress_image_copy_);
            }
        } else if (isTargetColor(cv::Point(P[0].x - 1, P[0].y - 1))) {
            P[0].y--;
            P[0].x--;
            if (isTestImage) {
                cv::circle(gate_progress_image_copy_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/left_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_" +
                                std::to_string(++num_points) + ".jpg",
                    gate_progress_image_copy_);
            }
        } else if (isTargetColor(cv::Point(P[0].x - 1, P[0].y + 1))) {
            P[0].y++;
            P[0].x--;
            if (isTestImage) {
                cv::circle(gate_progress_image_copy_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/left_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_" +
                                std::to_string(++num_points) + ".jpg",
                    gate_progress_image_copy_);
            }
        } else {
            cv::circle(gate_progress_image_copy_, P[0], 15, cv::Scalar(47, 152, 89), 5);
            cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD/left_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_found.jpg", gate_progress_image_copy_);
            break;
        }
    }
}

void SnakeGateDetector::searchRight(cv::Point& P0, cv::Point* P) {
    P[0] = P0;
    int num_points = 0;

    while (1) {
        if (isTargetColor(cv::Point(P[0].x + 1, P[0].y))) {
            P[0].x++;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/right_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_" +
                                std::to_string(++num_points) + ".jpg",
                    gate_progress_image_);
            }
        } else if (isTargetColor(cv::Point(P[0].x + 1, P[0].y - 1))) {
            P[0].y--;
            P[0].x++;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/right_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_" +
                                std::to_string(++num_points) + ".jpg",
                    gate_progress_image_);
            }
        } else if (isTargetColor(cv::Point(P[0].x + 1, P[0].y + 1))) {
            P[0].y++;
            P[0].x++;
            if (isTestImage) {
                cv::circle(gate_progress_image_, P[0], 7, cv::Scalar(32, 46, 9), -1);
                cv::circle (gate_progress_image_, P0, 15, cv::Scalar(47, 152, 89), 5);
                cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/right_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_" +
                                std::to_string(++num_points) + ".jpg",
                    gate_progress_image_);
            }
        } else {
            cv::circle(gate_progress_image_, P[0],15, cv::Scalar(47, 152, 89), 5);
            cv::imwrite(std::string(std::getenv("HOME")) + "/Desktop/SGD/right_" + std::to_string(P0.x) + "_" + std::to_string(P0.y) + "_found.jpg", gate_progress_image_);
            break;
        }
    }
}

void SnakeGateDetector::findMinimalSquare(const cv::Point* P_in, cv::Point* P_out) {
    int x_array[4] = {P_in[0].x, P_in[1].x, P_in[2].x, P_in[3].x};
    int y_array[4] = {P_in[0].y, P_in[1].y, P_in[2].y, P_in[3].y};

    int min_x = *std::min_element(x_array, x_array + 3);
    int max_x = *std::max_element(x_array, x_array + 3);
    int min_y = *std::min_element(y_array, y_array + 3);
    int max_y = *std::max_element(y_array, y_array + 3);

    P_out[0].x = min_x;
    P_out[0].y = min_y;

    P_out[1].x = min_x;
    P_out[1].y = max_y;

    P_out[2].x = max_x;
    P_out[2].y = min_y;

    P_out[3].x = max_x;
    P_out[3].y = max_y;
}

void refineCorner(const cv::Point* P_in, cv::Point* P_out) {
}

}  // namespace snake_gate_detector
