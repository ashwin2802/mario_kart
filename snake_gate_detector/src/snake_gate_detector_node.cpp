# include <snake_gate_detector/snake_gate_detector.hpp>

using namespace snake_gate_detector;

int main(int argc, char** argv) {
    ros::init(argc, argv, "snake_gate_detector_node");
    ros::NodeHandle nh;

    SnakeGateDetectorNode detect;

    detect.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}