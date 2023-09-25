#ifndef MARKER_VIZ_H
#define MARKER_VIZ_H

#include <vector>
#include <iterator>
#include <algorithm>
#include <ros/ros.h>
#include "visualization_marker.h"
#include "camera.h"
#include <tf/transform_broadcaster.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h> // Library effective with Linux


class VisualizationHandler{    
    std::vector<Camera> sys_cameras;
    std::vector<VisualizationMarker> sys_markers;
    std::vector<ros::Subscriber> sys_subs;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    ros::Publisher marker_pub;
    
    int find_marker(VisualizationMarker m);
    void callback(const aruco_msgs::MarkerArray& msg);
    
    public:
        VisualizationHandler(ros::NodeHandle& node_handle);
        void add_camera(Camera camera, std::string topic);
        void send_tfs();
        void clear_markers();
        void publish_markers();
};

std::vector<std::string> get_camera_topics();
tf::Matrix3x3  get_rot_from_quat(tf::Quaternion q);
tf::Quaternion get_quat_from_rot(tf::Matrix3x3 r);

#endif