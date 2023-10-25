#ifndef MARKER_VIZ_H
#define MARKER_VIZ_H

#include <vector>
#include <iterator>
#include <algorithm>
#include <ros/ros.h>
#include "visualization_objects.h"
#include <tf/transform_broadcaster.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

class VisualizationHandler{    
    protected:
        std::vector<Camera> sys_cameras;
        std::vector<VisualizationMarker> sys_markers;
        std::vector<ros::Subscriber> sys_subs;
        ros::NodeHandle nh;
        tf::TransformBroadcaster br;
        ros::Publisher marker_pub;

        int find_marker(VisualizationMarker m);
        void callback_markers(const aruco_msgs::MarkerArray& msg);
        void callback_cameras(const geometry_msgs::PoseArray& msg);
        void add_cameras(std::string topic);
        void add_markers(std::string topic);
    
    public:
        VisualizationHandler(ros::NodeHandle& node_handle);
        void start(std::string camera_topic, std::string markers_topic);
        void send_tfs();
        void clear_markers();
        void publish_markers();
};

std::vector<std::string> get_camera_topics();
tf::Matrix3x3  get_rot_from_quat(tf::Quaternion q);
tf::Quaternion get_quat_from_rot(tf::Matrix3x3 r);
void test_quaternion(tf::Quaternion q);

#endif