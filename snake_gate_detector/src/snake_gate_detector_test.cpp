#include <snake_gate_detector/libsnake_gate_detector.hpp>

int main() {
    cv::Mat frame;
    // std::cout << std::string(std::getenv("HOME")) + "/Desktop/SGD" + "/test.jpg" << std::endl;
    frame = cv::imread(std::string(std::getenv("HOME")) + "/Desktop" + "/test.jpg");


    snake_gate_detector::SnakeGateDetector snek;
    cv::Vec3b lower(10, 0, 0);
    cv::Vec3b upper(100, 255, 255);
    
    snek.setHSVThreshold(upper, lower);
    snek.setLengthThreshold(40);
    snek.setMaxGates(1);
    snek.isTestImage = true;
    snek.setImageFrame(frame);

    cv::Mat result = snek.getFrameWithGates();

    cv::namedWindow("Detected gates", cv::WINDOW_NORMAL);

    cv::imshow("Detected gates", result);
    while (cv::waitKey(0) != 113);
    return 0;
}