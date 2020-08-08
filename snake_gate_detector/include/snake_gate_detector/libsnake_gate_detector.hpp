#pragma once

#include <opencv2/opencv.hpp>
#include <cstdlib>

namespace snake_gate_detector {

class SnakeGateDetector {
  public:
    void setLengthThreshold(int length_threshold);
    void setColorFitnessThreshold(int color_fitness);
    void setImageFrame(cv::Mat frame);
    void setHSVThreshold(cv::Vec3b& upper, cv::Vec3b& lower);
    void setMaxGates(int max_gates);
    std::vector<cv::Point*>* getDetectedGates(); 
    cv::Mat getFrameWithGates();
  
  private:
    int length_threshold_;
    int color_fitness_threshold_;
    int max_gates_;
    
    std::vector<int> random_sample_;

    cv::Mat frame_;
    cv::Vec3b hsv_threshold_upper_;
    cv::Vec3b hsv_threshold_lower_;
    
    std::vector<cv::Point*> detected_gates_;
    
    void findGates();

    void randomize();
    bool isTargetColor(const cv::Point& P);
    double norm(cv::Point& P1, cv::Point& P2);
    double colorFitness(cv::Point* detected_gate);

    void searchUpDown(cv::Point& P0, cv::Point* P);
    void searchLeft(cv::Point& P0, cv::Point* P);
    void searchRight(cv::Point& P0, cv::Point* P);

    void findMinimalSquare(const cv::Point* P_in, cv::Point* P_out);
    void refineCorner(const cv::Point* P_in, cv::Point* P_out);

};
}  // namespace snake_gate_detector
