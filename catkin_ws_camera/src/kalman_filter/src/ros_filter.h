#pragma once

#include <map>

#include <ros/ros.h>
#include <ros/package.h>

#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include "filter_variables.h"
#include "kalman_filter.hpp"

#define DEBUG true

class RosFilter {
    public:
        RosFilter(); 

        // Find all aruco marker publisher topics for each camera
        void getCameraTopics();
        
        // Subscribe for all topics found by getCameraTopics
        void subscribeTopics();
        
        // Create Timer responsible for update the state of the system 
        ros::Timer createTimer(ros::Duration period);

    private:
        // The Kalman filter itself
        filter kf;

        // Node handle responsible for communication with ROS processes
        ros::NodeHandle nh;

        // Map of marker list on the system with cameras on the system
         std::map<int, std::vector<trackedMarker>> cam_markers;

        // Map of publishers for filtered marker data, one for each camera
        std::map<int, ros::Publisher> filtered_marker_publishers;

        // Map of publishers for camera data
        std::map<int, ros::Publisher> camera_publishers;

        // List of camera topics
        std::vector<std::string> topics;

        // Subscribers for each camera topic
        std::map<int, ros::Subscriber> camera_subs;

        // Map of cameras tracked by the system and their info
        std::map<int, cameraBasis> camera_poses;

        // number of poses that are tracked in the system (there may be multiple version of the same aruco tag for different cameras)
        // used for new indexes in state vector calculation without the need to iterate over cam_markers for all cameras 
        int tracked_poses = 0; // the same as stateVector size (from the filter)
        
        // Saves the last observation published by aruco ros node of each camera
        std::vector<std::shared_ptr<aruco_msgs::MarkerArray>> last_msgs;

        // Insert a camera on the system for fututre use on the filter
        void insertCameraBasis(int camera_id);

        // Create the filtered markers publishers and store the references in a map
        void createPublisherFiltered(std::string filtered_marker_topic, int camera_id, ros::NodeHandle nh);

        // Create the camera publishers and store the references in a map
        void createPublisherCameras(std::string camera_topic, int camera_id, ros::NodeHandle nh);
        
        // Extract the camera ID from the topic name (assuming "cam_x" format)
        int extractCameraID(const std::string& camera_topic);

        // Receives a msg with a pose, process it and convert it to eigen::vector
        Eigen::VectorXd msgToVector(geometry_msgs::Pose pose);

        // Finds the tf that transform the base of a camera to the closest it can to the origin/world (cam_1)
        tf::Transform tfToWorldBasis(int this_camera_id);

        // Evaluate if the marker is already tracked by te system and updates it. If not, creates it
        void insertUpdateMarker(aruco_msgs::Marker marker, int camera_id);
        
        // Insert new state variable on the filter
        void insertPosesOnStateVector(Eigen::VectorXd& newState);
    
        // Extracting a marker data from the state vector. Overload dedicated to markers
        aruco_msgs::Marker extractDataFromState(trackedMarker data, Eigen::VectorXd filtered_states, Eigen::MatrixXd covariance_matrix);
        
        // Extracting a camera data from the state vector. Overload dedicated to cameras
        geometry_msgs::PoseWithCovariance extractDataFromState(cameraBasis data, Eigen::VectorXd filtered_states, Eigen::MatrixXd covariance_matrix);
        
        // Publish filter data on ROS topics
        void publishData(const std::shared_ptr<aruco_msgs::MarkerArray>& msg, const int& camera_id);
    
        // Callback to handle marker data from cameras
        void cameraCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, const int& camera_id);
        
        // Callback to handle filter update event
        void timerCallback(const ros::TimerEvent& event);

};
