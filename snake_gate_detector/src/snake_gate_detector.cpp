#include <snake_gate_detector/snake_gate_detector.hpp>

namespace snake_gate_detector {

void SnakeGateDetectorNode::init(ros::NodeHandle& nh) {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    int max_gates;
    int length_threshold;
    cv::Vec3b lower, upper;

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

    lower.val[0] = h_min;
    lower.val[1] = s_min;
    lower.val[2] = v_min;

    upper.val[0] = h_max;
    upper.val[1] = s_max;
    upper.val[2] = v_max;

    detect_.setHSVThreshold(lower, upper);
    detect_.setMaxGates(max_gates);
    detect_.setLengthThreshold(length_threshold);
    // centre_pub_ = nh_private.advertise<detector_msgs::Centre>("centre_coord", 10);
    // thresh_pub_ = nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    // contour_pub_ = nh_private.advertise<sensor_msgs::Image>("contours", 10);
}

void SnakeGateDetectorNode::run() {
    if (img_.empty()) { return; };
    detect_.setImageFrame(img_);
    // detect_.thresholdImage(img_);
    // detect_.findGoodContours();
    // detect_.drawContours(img_);
    // detect_.fitRect(img_);

    // std::pair<int, int> centre_pair = detect_.getCentre();
    // double distance = detect_.getDistance();
    // centre_coord_.x = centre_pair.first;
    // centre_coord_.y = centre_pair.second;
    // centre_coord_.d = (float) distance;
    // centre_coord_.header.stamp = ros::Time::now();

    // sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThresh()).toImageMsg();
    // sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();

    // thresh_pub_.publish(thresh_msg);
    // contour_pub_.publish(contour_msg);
    // centre_pub_.publish(centre_coord_);
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
