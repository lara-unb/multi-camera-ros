// Example in your ROS node
#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include "kalman_filter.hpp"

// Structure to correlate marker poses with aruco id tags
struct trackedMarker {
    int arucoId;
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
};

class ros_filter{

    // The Kalman filter itself
    filter kf;

    ros::NodeHandle nh;

    // Data structure to store marker poses correlated with camera IDs
    std::map<std::string, aruco_msgs::MarkerArray> camera_marker_data;

    // Map of publishers for filtered marker data, one for each camera
    std::map<std::string, ros::Publisher> filtered_marker_publishers;

    // List of camera topics
    std::vector<std::string> topics;

    // Subscribers for each camera topic
    std::map<std::string, ros::Subscriber> camera_subs;

    // List of trackedMarkers
    std::vector<trackedMarker> trackedMarkers;

    public:
        ros_filter(){
            // Set the dimensions and other parameters for kf_params as needed
            kf.setDimensions(7, 7);

            kf.initNormalDist(0.0001, 0.1);

            // Reset the Kalman filter with the defined parameters
            kf.reset();
        }

        // Find all aruco marker publisher topics for each camera
        void getCameraTopics(){ 
            ros::master::V_TopicInfo master_topics;
            ros::master::getTopics(master_topics);
            std::string marker_topic_name("/aruco_marker_publisher/markers");

            for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
                const ros::master::TopicInfo& info = *it;
                if(info.name.find(marker_topic_name) != std::string::npos){
                    if(info.name.find("list") == std::string::npos){  // filters "markers_list" topics from the vector
                        topics.push_back(info.name);
                    }  
                }
            }
        }

        // Extract the camera ID from the topic name (assuming "cam_x" format)
        std::string extractCameraID(const std::string& camera_topic) {
            std::size_t found = camera_topic.find("cam_");
            if (found != std::string::npos) {
                return camera_topic.substr(found, 4); // Extract the ID part
            }
            return ""; // Return an empty string if not found
        }

        void subscribeTopics(){
            for (const std::string& camera_topic : topics) {
                camera_subs[camera_topic] = nh.subscribe<aruco_msgs::MarkerArray>(
                    camera_topic, 10, boost::bind(&ros_filter::cameraCallback, this, _1, camera_topic));

                // Create publishers for filtered marker data, one for each camera
                std::string camera_id = extractCameraID(camera_topic);

                if (!camera_id.empty()) {
                    std::string filtered_marker_topic = "/cam_" + camera_id + "/filtered_markers";
                    publishFiltered( filtered_marker_topic, camera_id, nh);
                }
            }
        }

        // Callback to handle marker data from different cameras
        void cameraCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, const std::string& camera_topic) {
            // Extract the camera ID from the topic name
            std::string camera_id = extractCameraID(camera_topic);

            // Store the marker data correlated with the camera ID
            camera_marker_data[camera_id] = *msg;

            // Implement Kalman filter and markerArray publishing

            for (const auto& marker : msg->markers){
                int markerTagId = marker.id;
                geometry_msgs::Pose pose = marker.pose.pose;

                tf::Transform transform;
                tf::Quaternion quaternion, rotation;

                transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
                tf::quaternionMsgToTF(pose.orientation, quaternion);
                rotation.setRPY(0, 0, M_PI); // rotating 180 degrees in the z axis
                quaternion = quaternion * rotation;
                transform.setRotation(quaternion);

                Eigen::VectorXd poseVector(7); // storing the poses in a vector with the format [xt,yt,zt,xr,yr,zr,wr]

                poseVector << transform.getOrigin().x(),
                              transform.getOrigin().y(),
                              transform.getOrigin().z(),
                              transform.getRotation().x(),
                              transform.getRotation().y(),
                              transform.getRotation().z(),
                              transform.getRotation().w();

                kf.predict();

                // transform an eigen vector to matrix for compatibility with filter function
                Eigen::MatrixXd auxVector(poseVector.size(), 1);
                auxVector = poseVector;

                kf.correct(auxVector);

                Eigen::VectorXd poseVectorNew(7);

                auto it = std::find_if(trackedMarkers.begin(), trackedMarkers.end(),
                                       [markerTagId](const trackedMarker& tracked_marker){
                                            return tracked_marker.arucoId == markerTagId;
                                       });

                if(it != trackedMarkers.end()){
                    // The marker was already detected by the system

                }else{
                    // The marker is a new find
                    trackedMarker auxMarker;
                    auxMarker.arucoId = markerTagId;
                    auxMarker.state = kf.getState();
                    auxMarker.covariance = kf.getCovariance();

                    //trackedMarkers.push_back();

                }

            }

            // After filtering, publish the filtered marker data for the respective camera
            if (filtered_marker_publishers.find(camera_id) != filtered_marker_publishers.end()) {
                filtered_marker_publishers[camera_id].publish(*msg);
            }
        }

        void publishFiltered(std::string filtered_marker_topic, std::string camera_id, ros::NodeHandle nh){
            filtered_marker_publishers[camera_id] = nh.advertise<aruco_msgs::MarkerArray>(
                filtered_marker_topic, 10);
        }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_data_processor");

    ros_filter filter;

    filter.getCameraTopics();


    ros::spin();

    return 0;
}