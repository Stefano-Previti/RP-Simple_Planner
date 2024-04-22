#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;
    ROS_INFO("first try");
    ros::spin();
    return 0;
}
