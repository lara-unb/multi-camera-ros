#ifndef VISUALIZATION_MARKER_H
#define VISUALIZATION_MARKER_H

#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <aruco_msgs/Marker.h>

class VisualizationMarker{
    visualization_msgs::Marker marker;
    tf::Transform tf;
    std::string parent_frame;

    public:
        VisualizationMarker();
        VisualizationMarker(aruco_msgs::Marker m);

        tf::StampedTransform get_tf_stamped() const;
        tf::Transform get_tf() const;
        visualization_msgs::Marker get_marker() const;
        bool operator==(const VisualizationMarker& obj);
        void set_marker(VisualizationMarker m);
        void set_pose(tf::Transform new_pose);
};

class Camera {
    tf::Transform tf;
    std::string frame_id; 
    
    public:
        Camera(std::string id);
        Camera(tf::Vector3 tr, tf::Quaternion rot, std::string id);
        
        tf::StampedTransform get_tf_stamped() const ;
        void set_tf(tf::Vector3 tr, tf::Quaternion rot);
        bool operator==(const Camera& obj);
        std::string get_frame_id() const ;    

};

#endif