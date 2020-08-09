#include <snake_gate_detector/snake_gate_detector.hpp>

namespace snake_gate_detector {

void SnakeGateDetectorNode::init(ros::NodeHandle& nh) {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    int max_gates;
    int length_threshold;

    img_sub_ = nh.subscribe("image_raw", 1, &SnakeGateDetectorNode::imageCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("h_min", h_min);
    nh_private.getParam("s_min", s_min);
    nh_private.getParam("v_min", v_min);
    nh_private.getParam("h_max", h_max);
    nh_private.getParam("s_max", s_max);
    nh_private.getParam("v_max", v_max);
    nh_private.getParam("max_gates", max_gates);
    nh_private.getParam("length_threshold", length_threshold);

    cv::Vec3b lower(h_min, s_min, v_min);
    cv::Vec3b upper(h_max, s_max, v_max);

    detect_.setHSVThreshold(upper, lower);
    detect_.setMaxGates(max_gates);
    detect_.setLengthThreshold(length_threshold);
    detect_.isTestImage = false;
    // centre_pub_ = nh_private.advertise<detector_msgs::Centre>("centre_coord", 10);
    // thresh_pub_ = nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    result_pub_ = nh_private.advertise<sensor_msgs::Image>("result_image", 10);
}

void SnakeGateDetectorNode::run() {
    if (img_.empty()) { return; };
    detect_.setImageFrame(img_);
    cv::Mat frame = detect_.getFrameWithGates();

    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    result_pub_.publish(result_msg);
}

void SnakeGateDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img_ = cv_ptr->image;
}

}  // namespace snake_gate_detector
