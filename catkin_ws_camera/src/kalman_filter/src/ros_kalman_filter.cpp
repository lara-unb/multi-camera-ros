#include <ros/ros.h>
#include "kalman_filter.hpp"

#define queue_size 1

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "stochastic_filter_node");
    ros::NodeHandle nh;

    // Create an instance of the kalman_filter::KalmanFilter class
    kalman_filter::filter kf;

    // Set the dimensions and other parameters for kf_params as needed
    kf.setDimensions(7, 6, 6);

    // Reset the Kalman filter with the defined parameters
    kf.reset();

    // Subscribing to input data topic
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("kalman_filter_output", queue_size);
    //ros::Subscriber sub = nh.subscribe("topic_name", queue_size, callback_function);

    // Main loop
    ros::Rate rate(10);  // Adjust the loop rate as needed

    while (ros::ok())
    {
        kf.predict();
        kf.correct(Y);

        // Publish or use the filtered data as needed
        // pub.publish(filtered_data);
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
