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
    std::vector<cv::Point[4]>* getDetectedGates(); 
  
  private:
    int length_threshold_;
    int color_fitness_threshold_;
    int max_gates_;
    
    cv::Mat frame_;
    cv::Vec3b hsv_threshold_upper_;
    cv::Vec3b hsv_threshold_lower_;
    
    std::vector<cv::Point[4]> detected_gates_;
    
    void findGates();

    cv::Point randomPoint(int width, int height);
    bool isTargetColor(cv::Point& P);
    double norm(cv::Point& P1, cv::Point& P2);
    double colorFitness(cv::Point* detected_gate);

    cv::Point* searchUpDown(cv::Point& P);
    cv::Point searchLeftRight(cv::Point& P);

    cv::Point* findMinimalSquare(cv::Point& P1, cv::Point& P2, cv::Point& P3, cv::Point& P4);
    cv::Point* refineCorner(cv::Point& S1, cv::Point& S2, cv::Point& S3, cv::Point& S4);    

};
}  // namespace snake_gate_detector
