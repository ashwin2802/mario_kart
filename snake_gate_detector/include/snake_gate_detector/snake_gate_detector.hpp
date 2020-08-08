#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <snake_gate_detector/libsnake_gate_detector.hpp>

namespace snake_gate_detector {

class SnakeGateDetectorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat img_;

    ros::Subscriber img_sub_;

    // ros::Publisher centre_pub_;
    // ros::Publisher thresh_pub_;
    // ros::Publisher contour_pub_;

    SnakeGateDetector detect_;
};

}  // namespace snake_gate_detector
