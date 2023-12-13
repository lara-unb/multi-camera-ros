#include <ros/ros.h>
#include "ros_filter.h"
#include "filter_variables.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman_aruco");

    RosFilter filter;

    ROS_INFO_STREAM("Kalman filter on!");
    filter.getCameraTopics();
    filter.subscribeTopics();
    //filter.insertCameraBasis(3); //is private function
    ros::Timer timer = filter.createTimer(ros::Duration(0.005));
    ros::spin();

    return 0;
}