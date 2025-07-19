#pragma once

#include <map>

#include <ros/ros.h>
#include <ros/package.h>

#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <std_srvs/SetBool.h>


#include "filter_variables.h"
#include "kalman_filter.hpp"

#define DEBUG true

class RosFilter {
    public:
        RosFilter(); 

        // Find all aruco marker publisher topics for each camera
        void getCameraTopics();
        
        // Subscribe for all topics found by getCameraTopics and created the publishers
        void subscribeTopics();
        
        // Create Timer responsible for update the state of the system 
        ros::Timer createTimer(ros::Duration period);

        // Insert a camera on the system for fututre use on the filter
        void insertCameraBasis(int camera_id);

        // Saves the timestamp and filter covariance main diagonal on a csv file on the readings/filtered folder
        void saveFullLog();

        // Sets up a service to enable or disable logging
        void setupLoggingService(ros::NodeHandle& nh);
        bool loggingServiceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

        // Flag to control logging
        bool logging_enabled = false;

        // Service server for logging control
        ros::ServiceServer logging_service_server;

        // Save camera logs to separate files
        void saveCameraLogs();

        // Save marker logs to separate files
        void saveMarkerLogs();

        // Save unfiltered marker logs to separate files
        void saveUnfilteredMarkerLogs();

    private:
        // The Kalman filter itself
        filter kf;

        // Node handle responsible for communication with ROS processes
        ros::NodeHandle nh;

        // Map of marker list on the system with cameras on the system
        std::map<int, std::vector<trackedMarker>> cam_markers;

        // Publisher for filtered marker data
        ros::Publisher filtered_marker_publisher;

        // Publisher for camera data
        ros::Publisher camera_publisher;

        // List of camera topics
        std::vector<std::string> topics;

        // Subscribers for each camera topic
        std::map<int, ros::Subscriber> camera_subs;

        // Map of cameras tracked by the system and their info
        std::map<int, cameraBasis> camera_poses;

        // number of poses that are tracked in the system (there may be multiple version of the same aruco tag for different cameras)
        // used for new indexes in state vector calculation without the need to iterate over cam_markers for all cameras 
        int tracked_poses = 0; // the same as stateVector size (from the filter)

        // Saves the last observation published by aruco ros node of each camera and correlate to the camera that observes it
        std::map<int, std::shared_ptr<aruco_msgs::MarkerArray>> last_msgs; 

        // Create the filtered markers publishers and store the references in a map
        void createPublisherFiltered(std::string filtered_marker_topic, ros::NodeHandle nh);

        // Create the camera poses publisher
        void createPublisherCameras(std::string camera_topic, ros::NodeHandle nh);

        // Extract the camera ID from the topic name (assuming "cam_x" format)
        int extractCameraID(const std::string& camera_topic);

        // Receives a msg with a pose, process it and convert it to eigen::vector
        Eigen::VectorXd msgToPose(geometry_msgs::Pose pose);

       // Converts a given pose from Eigen::VectorXd  to poseTransform
        poseTransform poseToTf(Eigen::VectorXd pose);

        // Converts a given pose from poseTransform to Eigen::VectorXd
        Eigen::VectorXd tfToPose(poseTransform transform); 

        // Evaluate if the marker is already tracked by te system and updates it. If not, creates it
        void insertUpdateMarker(aruco_msgs::Marker marker, int camera_id);

        // Insert new marker state variables on the filter and updates camera poses
        void insertPosesOnStateVector(Eigen::VectorXd& newState, int cam_id);

        // Extracting a marker data from the state vector. Overload dedicated to markers
        aruco_msgs::Marker extractDataFromState(trackedMarker data, Eigen::VectorXd filtered_states, Eigen::MatrixXd covariance_matrix);

        // Extracting a camera data from the state vector. Overload dedicated to cameras which dont return covariances (since ros doesnt has a message for vector of poseWithCovariance)
        geometry_msgs::Pose extractDataFromState(cameraBasis data, Eigen::VectorXd filtered_states);

        // Prepare the data to publish them properly
        void preparePublishData(geometry_msgs::PoseArray& camera_poses_msg, aruco_msgs::MarkerArray& filtered_markers_msg);

        // Publish filter data on ROS topics
        void publishData(const geometry_msgs::PoseArray camera_poses_msg, const aruco_msgs::MarkerArray filtered_markers_msg);

        // Callback to handle marker data from cameras
        void cameraCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, const int& camera_id);

        // Callback to handle filter update event
        void timerCallback(const ros::TimerEvent& event);
        
        // Resize the state vector based on the number of objects detected 
        void resizeState();

        // Resize the state vector based on the number of objects detected, and inserts if there is a base data
        void resizeState(Eigen::VectorXd newData);

};