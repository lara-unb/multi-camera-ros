#ifndef MARKER_H
#define MARKER_H

#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Marker{
    visualization_msgs::Marker marker;
    tf::Transform tf;
    std::string parent_frame;
    
    public:
        Marker();
        // Marker(visualization_msgs::Marker marker_shape, std::string frame_id);
        // Marker(tf::Transform marker_tf, std::string frame_id);

        // tf::StampedTransform get_tf();
        // void set_tf(tf::Vector3 tr, tf::Quaternion rot);
        visualization_msgs::Marker get_marker();
};


#endif