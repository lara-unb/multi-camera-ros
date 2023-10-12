// Example in your ROS node
#include <ros/ros.h>
#include <ros/package.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include "kalman_filter.hpp"

#define DEBUG true

// Structure to correlate marker poses with aruco id tags
typedef struct trackedMarker {
    int stateVectorAddr; // index where its found in the state vector of the kalman filter
    int arucoId;         // aruco tag id
    Eigen::VectorXd pose;
    Eigen::MatrixXd covariance;
}trackedMarker;

class ros_filter{

    // The Kalman filter itself
    filter kf;

    ros::NodeHandle nh;

    // Map of marker list on the system with cameras on the system
    std::map<int, std::vector<trackedMarker>> cam_markers;

    // Map of publishers for filtered marker data, one for each camera
    std::map<int, ros::Publisher> filtered_marker_publishers;

    // List of camera topics
    std::vector<std::string> topics;

    // Subscribers for each camera topic
    std::map<int, ros::Subscriber> camera_subs;

    // number of poses that are tracked in the system (there may be multiple version of the same aruco tag for different cameras)
    // used for new indexes in state vector calculation without the need to iterate over cam_markers for all cameras 
    int tracked_poses = 0;

    public:
        ros_filter(){
            // Prepare the filter by defining the initial dimension
            kf.set_dimensions(0);
            ROS_INFO("Starting kalman filter aruco node");
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

        void subscribeTopics(){
            for (const std::string& camera_topic : topics) {
                int camera_id = extractCameraID(camera_topic);

                camera_subs[camera_id] = nh.subscribe<aruco_msgs::MarkerArray>(
                    camera_topic, 10, boost::bind(&ros_filter::cameraCallback, this, _1, camera_id));

                // Create publishers for filtered marker data, one for each camera

                if (camera_id != -1) {
                    std::string filtered_marker_topic = "/cam_" + std::to_string(camera_id) + "/filtered_markers";
                    createPublisherFiltered( filtered_marker_topic, camera_id, nh);
                }
            }
        }

    private:
        // Evaluate if the marker is already tracked by te system and updates it. If not, creates it
        void insertUpdateMarker(aruco_msgs::Marker marker, int camera_id){
            int markerTagId = marker.id;
            geometry_msgs::Pose pose = marker.pose.pose;

            Eigen::VectorXd poseVector = msgToVector(pose);

            // Verifying if this marker is already tracked by this camera, if not, insert in the system
            auto it = std::find_if(cam_markers[camera_id].begin(), cam_markers[camera_id].end(),
                                    [markerTagId](const trackedMarker& tracked_marker){
                                        return tracked_marker.arucoId == markerTagId;
                                    });

            if(it != cam_markers[camera_id].end()){
                // The marker is already tracked
                trackedMarker& auxMarker = cam_markers[camera_id][std::distance(cam_markers[camera_id].begin(), it)];
                auxMarker.pose = poseVector;
                //auxMarker.covariance = Eigen::MatrixXd::Identity() * HIGH_COVARIANCE_PRESET;

            }else{
                // The marker is new
                trackedMarker auxMarker;
                auxMarker.arucoId = markerTagId;
                auxMarker.stateVectorAddr = tracked_poses * POSE_VECTOR_SIZE;
                auxMarker.pose = poseVector;
                auxMarker.covariance = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) * HIGH_COVARIANCE_PRESET;

                cam_markers[camera_id].push_back(auxMarker);
                tracked_poses++;
            }
        }

        void insertPosesOnStateVector(Eigen::VectorXd& newState){
            for(const auto& pair : cam_markers){
                for(const auto& marker : pair.second){
                    ROS_INFO("stateVectorAddr: %d", marker.stateVectorAddr);
                    newState.segment(marker.stateVectorAddr, POSE_VECTOR_SIZE) = marker.pose;
                }
            }
        }

        void publishData(const aruco_msgs::MarkerArray::ConstPtr& msg, const int& camera_id){
            // Retrieve the filtered data from the Kalman filter (kf)
            Eigen::VectorXd filtered_states = kf.getState();
            Eigen::MatrixXd covariance_matrix = kf.getCovariance();

            aruco_msgs::MarkerArray filtered_markers_msg;
            filtered_markers_msg.markers.reserve(cam_markers[camera_id].size());

            for (const auto& marker : cam_markers[camera_id]) {
                aruco_msgs::Marker filtered_marker;
                filtered_marker.id = marker.arucoId;
                filtered_marker.header = msg->header; // Use the same header as the input marker data

                // Extract the filtered data corresponding to this marker
                if (marker.stateVectorAddr + POSE_VECTOR_SIZE <= filtered_states.size()) {
                    Eigen::VectorXd marker_pose = filtered_states.segment(marker.stateVectorAddr, POSE_VECTOR_SIZE);

                    // Create a geometry_msgs::Pose message from the filtered pose data
                    geometry_msgs::Pose filtered_pose;
                    filtered_pose.position.x = marker_pose(0);
                    filtered_pose.position.y = marker_pose(1);
                    filtered_pose.position.z = marker_pose(2);
                    filtered_pose.orientation.x = marker_pose(3);
                    filtered_pose.orientation.y = marker_pose(4);
                    filtered_pose.orientation.z = marker_pose(5);
                    filtered_pose.orientation.w = marker_pose(6);

                    filtered_marker.pose.pose = filtered_pose;

                    // Populate the filtered_marker.pose.covariance with the covariance matrix
                    filtered_marker.pose.covariance.fill(0.0); // Initialize to zero

                    #if DEBUG == true
                    ROS_INFO("covariance_matrix_size: %ld", covariance_matrix.size());
                    ROS_INFO("filtered_marker.pose.covariance_size: %ld", filtered_marker.pose.covariance.size());
                    #endif

                    // Extract the corresponding covariance matrix for this marker
                    // adding the '-1' part is important because the covariance matrix is 7x7 and the last row and cols relate to the
                    // imaginary 'w' rotation which is not really meaningful, and the output is a 6x6 matrix by the format 
                    // specified in geometry_msgs

                    if (marker.stateVectorAddr + POSE_VECTOR_SIZE <= covariance_matrix.rows()  &&
                        marker.stateVectorAddr + POSE_VECTOR_SIZE <= covariance_matrix.cols() ) {
                        for (int i = 0; i < POSE_VECTOR_SIZE -1; ++i) {
                            for (int j = 0; j < POSE_VECTOR_SIZE -1; ++j) {
                                filtered_marker.pose.covariance[i * (POSE_VECTOR_SIZE -1) + j] =
                                    covariance_matrix(marker.stateVectorAddr + i, marker.stateVectorAddr + j);
                            }
                        }
                    }
                    filtered_markers_msg.markers.push_back(filtered_marker);
                }
            }

            // Publish the filtered marker data
            if (filtered_marker_publishers.find(camera_id) != filtered_marker_publishers.end()) {
                filtered_marker_publishers[camera_id].publish(filtered_markers_msg);
            }
        }

        // Create the publishers and store the references in a map
        void createPublisherFiltered(std::string filtered_marker_topic, int camera_id, ros::NodeHandle nh){
            filtered_marker_publishers[camera_id] = nh.advertise<aruco_msgs::MarkerArray>(
                filtered_marker_topic, 10);
        }

        // Extract the camera ID from the topic name (assuming "cam_x" format)
        int extractCameraID(const std::string& camera_topic) {
            std::size_t found = camera_topic.find("cam_");
            if (found != std::string::npos) {
                std::string aux = camera_topic.substr(found + 4); // Extract the ID part
                try {
                    return std::stoi(aux);
                } catch(const std::invalid_argument& e){
                    return -1;
                }
            }
            return -1;
        }

        // Receives a msg with a pose, process it and convert it to eigen::vector
        Eigen::VectorXd msgToVector(geometry_msgs::Pose pose){
            tf::Transform transform;
            tf::Quaternion adjust_rotation, msg_rotation;
            tf::Vector3 msg_orientation;

            msg_orientation = tf::Vector3(pose.position.x,
                                          pose.position.y,
                                          pose.position.z);

            msg_rotation = tf::Quaternion(pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w);
            
            transform = tf::Transform(msg_rotation, msg_orientation);
            adjust_rotation = tf::Quaternion(0, 0, 1, 0);
            auto adjust_rotation_tf =  tf::Transform(adjust_rotation);
            transform = adjust_rotation_tf * transform;

            Eigen::VectorXd poseVector(7); // storing the poses in a vector with the format [xt,yt,zt,xr,yr,zr,wr]

            poseVector << transform.getOrigin().x(),
                            transform.getOrigin().y(),
                            transform.getOrigin().z(),
                            transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w();
            return poseVector;
        }
        
        // Callback to handle marker data from cameras
        void cameraCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, const int& camera_id) {

            // Insert the markers detected in the system
            for (const auto& marker : msg->markers){
                insertUpdateMarker(marker, camera_id);
            }

            // Adapt the filter for the new data/change of state vector size
            Eigen::VectorXd oldState = kf.getState();

            // Verifying if there is a need to extend the state vector (new states)
            if(oldState.size() < tracked_poses * POSE_VECTOR_SIZE){
                oldState.conservativeResize(tracked_poses * POSE_VECTOR_SIZE);
                oldState.tail(POSE_VECTOR_SIZE).setZero();
                kf.insertState(oldState);
            }

            Eigen::VectorXd newState(tracked_poses * POSE_VECTOR_SIZE);
            insertPosesOnStateVector(newState);

            #if DEBUG == true
                ROS_INFO("cam_marker size: %lu", cam_markers[camera_id].size());
            #endif

            // Filter the data
            kf.predict();
            kf.correct(newState);

            // Construct the to-be-published data structure and publish them
            publishData(msg, camera_id);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman_aruco");

    ros_filter filter;

    ROS_INFO_STREAM("Kalman filter on!");
    filter.getCameraTopics();
    filter.subscribeTopics();

    ros::spin();

    return 0;
}