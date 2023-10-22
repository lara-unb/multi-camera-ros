#include "ros_filter.h"

#include <tf/transform_broadcaster.h>


RosFilter::RosFilter() {
    // Prepare the filter by defining the initial dimension
    kf.set_dimensions(0);
    ROS_INFO("Starting kalman filter aruco node");
}

void RosFilter::getCameraTopics() {
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

    for (int i = 0; i < topics.size(); i++)
    {   
        last_msgs.push_back(std::make_shared<aruco_msgs::MarkerArray>());
    }
            
}

void RosFilter::subscribeTopics() {
    for (const std::string& camera_topic : topics) {
        int camera_id = extractCameraID(camera_topic);

        insertCameraBasis(camera_id);

        camera_subs[camera_id] = nh.subscribe<aruco_msgs::MarkerArray>(
            camera_topic, 10, boost::bind(&RosFilter::cameraCallback, this, _1, camera_id));

        // Create publishers for filtered marker data, one for each camera; and for the camera poses
        if (camera_id != -1) {
            std::string topic_name = "/cam_" + std::to_string(camera_id);
            createPublisherCameras(topic_name + "/filtered_cam_pose", camera_id, nh);
            createPublisherFiltered(topic_name + "/filtered_markers", camera_id, nh);
        }
    }
}
     
ros::Timer RosFilter::createTimer(ros::Duration period) {
    return nh.createTimer(period, &RosFilter::timerCallback, this);
}

void RosFilter::insertCameraBasis(int camera_id) {
    // Verifying if this camera is already tracked. If not, insert in the system
    if(!camera_poses.contains(camera_id)) {
        cameraBasis aux;
        aux.stateVectorAddr = (tracked_poses++) * POSE_VECTOR_SIZE;
        aux.pose = Eigen::VectorXd::Zero(POSE_VECTOR_SIZE);
        aux.covariance = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE);
        
        if(camera_id != 1) {
            aux.covariance *= HIGH_COVARIANCE_PRESET;
        }
        camera_poses[camera_id] = aux;
    }
}

void RosFilter::createPublisherFiltered(std::string filtered_marker_topic, int camera_id, ros::NodeHandle nh) {
    filtered_marker_publishers[camera_id] = nh.advertise<aruco_msgs::MarkerArray>(filtered_marker_topic, 5);
}

void RosFilter::createPublisherCameras(std::string camera_topic, int camera_id, ros::NodeHandle nh) {
    camera_publishers[camera_id] = nh.advertise<geometry_msgs::PoseWithCovariance>(camera_topic, 5);
}

int  RosFilter::extractCameraID(const std::string& camera_topic) {
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

Eigen::VectorXd RosFilter::msgToVector(geometry_msgs::Pose pose) {
    tf::Transform transform;
    tf::Quaternion adjust_rotation, msg_orientation;
    tf::Vector3 msg_pose;

    msg_pose = tf::Vector3(pose.position.x,
                            pose.position.y,
                            pose.position.z);

    msg_orientation = tf::Quaternion(pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z,
                                    pose.orientation.w);

    transform = tf::Transform(msg_orientation, msg_pose);
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

void RosFilter::insertUpdateMarker(aruco_msgs::Marker marker, int camera_id) {
    int markerTagId = marker.id;
    geometry_msgs::Pose pose = marker.pose.pose;

    Eigen::VectorXd poseVector = msgToVector(pose);

    // Verifying if this marker is already tracked by this camera, if not, insert in the system
    auto it = std::find_if(cam_markers[camera_id].begin(), cam_markers[camera_id].end(),
                            [markerTagId](const trackedMarker& tracked_marker){
                                return tracked_marker.arucoId == markerTagId;
                            });

    if(it != cam_markers[camera_id].end()) {
        // The marker is already tracked
        trackedMarker& auxMarker = cam_markers[camera_id][std::distance(cam_markers[camera_id].begin(), it)];
        auxMarker.pose = poseVector;
        //auxMarker.covariance = Eigen::MatrixXd::Identity() * HIGH_COVARIANCE_PRESET;

    }else {
        // The marker is new
        trackedMarker auxMarker;
        auxMarker.arucoId = markerTagId;
        auxMarker.stateVectorAddr = (tracked_poses++) * POSE_VECTOR_SIZE;
        auxMarker.pose = poseVector;
        auxMarker.covariance = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) * HIGH_COVARIANCE_PRESET;

        cam_markers[camera_id].push_back(auxMarker);
    }
}

void RosFilter::insertPosesOnStateVector(Eigen::VectorXd& newState) {
    for(const auto& pair : cam_markers) {
        ROS_INFO("stateVectorAddr: %d", camera_poses[pair.first].stateVectorAddr);
        newState.segment(camera_poses[pair.first].stateVectorAddr, POSE_VECTOR_SIZE) = camera_poses[pair.first].pose;
        for(const auto& marker : pair.second) {
            ROS_INFO("stateVectorAddr: %d", marker.stateVectorAddr);
            newState.segment(marker.stateVectorAddr, POSE_VECTOR_SIZE) = marker.pose;
        }
    }
}

aruco_msgs::Marker RosFilter::extractDataFromState(trackedMarker data, Eigen::VectorXd filtered_states, Eigen::MatrixXd covariance_matrix) {
    aruco_msgs::Marker filtered_data;
    filtered_data.id = data.arucoId;

    // Extract the filtered data corresponding to this marker
    if (data.stateVectorAddr + POSE_VECTOR_SIZE <= filtered_states.size()) {
        Eigen::VectorXd data_pose = filtered_states.segment(data.stateVectorAddr, POSE_VECTOR_SIZE);

        // Create a geometry_msgs::Pose message from the filtered pose data
        geometry_msgs::Pose filtered_pose;
        filtered_pose.position.x = data_pose(0);
        filtered_pose.position.y = data_pose(1);
        filtered_pose.position.z = data_pose(2);
        filtered_pose.orientation.x = data_pose(3);
        filtered_pose.orientation.y = data_pose(4);
        filtered_pose.orientation.z = data_pose(5);
        filtered_pose.orientation.w = data_pose(6);

        filtered_data.pose.pose = filtered_pose;

        // Populate the filtered_data.pose.covariance with the covariance matrix
        filtered_data.pose.covariance.fill(0.0); // Initialize to zero

        #if DEBUG == true
        ROS_INFO("covariance_matrix_size: %ld", covariance_matrix.size());
        ROS_INFO("filtered_data.pose.covariance_size: %ld", filtered_data.pose.covariance.size());
        #endif

        // Extract the corresponding covariance matrix for this marker
        // adding the '-1' part is important because the covariance matrix is 7x7 and the last row and cols relate to the
        // imaginary 'w' rotation which is not really meaningful, and the output is a 6x6 matrix by the format 
        // specified in geometry_msgs

        if (data.stateVectorAddr + POSE_VECTOR_SIZE <= covariance_matrix.rows()  &&
            data.stateVectorAddr + POSE_VECTOR_SIZE <= covariance_matrix.cols() ) {
            for (int i = 0; i < POSE_VECTOR_SIZE -1; ++i) {
                for (int j = 0; j < POSE_VECTOR_SIZE -1; ++j) {
                    filtered_data.pose.covariance[i * (POSE_VECTOR_SIZE -1) + j] =
                        covariance_matrix(data.stateVectorAddr + i, data.stateVectorAddr + j);
                }
            }
        }
    }
    return filtered_data;
}

geometry_msgs::PoseWithCovariance RosFilter::extractDataFromState(cameraBasis data, Eigen::VectorXd filtered_states, Eigen::MatrixXd covariance_matrix) {
    geometry_msgs::PoseWithCovariance filtered_data;

    // Extract the filtered data corresponding to this marker
    if (data.stateVectorAddr + POSE_VECTOR_SIZE <= filtered_states.size()) {
        Eigen::VectorXd data_pose = filtered_states.segment(data.stateVectorAddr, POSE_VECTOR_SIZE);

        // Create a geometry_msgs::Pose message from the filtered pose data
        geometry_msgs::Pose filtered_pose;
        filtered_pose.position.x = data_pose(0);
        filtered_pose.position.y = data_pose(1);
        filtered_pose.position.z = data_pose(2);
        filtered_pose.orientation.x = data_pose(3);
        filtered_pose.orientation.y = data_pose(4);
        filtered_pose.orientation.z = data_pose(5);
        filtered_pose.orientation.w = data_pose(6);

        filtered_data.pose = filtered_pose;

        // Populate the filtered_data.covariance with the covariance matrix
        filtered_data.covariance.fill(0.0); // Initialize to zero

        #if DEBUG == true
        ROS_INFO("covariance_matrix_size: %ld", covariance_matrix.size());
        ROS_INFO("filtered_data.pose.covariance_size: %ld", filtered_data.covariance.size());
        #endif

        // Extract the corresponding covariance matrix for this marker
        // adding the '-1' part is important because the covariance matrix is 7x7 and the last row and cols relate to the
        // imaginary 'w' rotation which is not really meaningful, and the output is a 6x6 matrix by the format 
        // specified in geometry_msgs

        if (data.stateVectorAddr + POSE_VECTOR_SIZE <= covariance_matrix.rows()  &&
            data.stateVectorAddr + POSE_VECTOR_SIZE <= covariance_matrix.cols() ) {
            for (int i = 0; i < POSE_VECTOR_SIZE -1; ++i) {
                for (int j = 0; j < POSE_VECTOR_SIZE -1; ++j) {
                    filtered_data.covariance[i * (POSE_VECTOR_SIZE -1) + j] =
                        covariance_matrix(data.stateVectorAddr + i, data.stateVectorAddr + j);
                }
            }
        }
    }
    return filtered_data;
}

void RosFilter::publishData(const std::shared_ptr<aruco_msgs::MarkerArray>& msg, const int& camera_id){
    // Retrieve the filtered data from the Kalman filter (kf)
    Eigen::VectorXd filtered_states = kf.getState();
    Eigen::MatrixXd covariance_matrix = kf.getCovariance();

    geometry_msgs::PoseWithCovariance camera_msg;
    camera_msg = extractDataFromState(camera_poses[camera_id], filtered_states, covariance_matrix);

    aruco_msgs::MarkerArray filtered_markers_msg;
    filtered_markers_msg.markers.reserve(cam_markers[camera_id].size());

    for (const auto& marker : cam_markers[camera_id]) {
        aruco_msgs::Marker markerAux = extractDataFromState(marker, filtered_states, covariance_matrix);
        markerAux.header = msg->header; // Use the same header as the input marker data
        filtered_markers_msg.markers.push_back(markerAux);
    }

    // Publish the filtered marker data
    if (filtered_marker_publishers.find(camera_id) != filtered_marker_publishers.end()){
        filtered_marker_publishers[camera_id].publish(filtered_markers_msg);
    }

    // Publish the filtered camera data
    if (camera_publishers.find(camera_id) != camera_publishers.end()){
        camera_publishers[camera_id].publish(camera_msg);
    }
}

void RosFilter::cameraCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, const int& camera_id) {
    int index  = camera_id - 1;
    auto p  = last_msgs.at(index);
    *p = *msg;
}

void RosFilter::timerCallback(const ros::TimerEvent& event) {
    for (int i = 0; i < last_msgs.size(); i++) {
        if(last_msgs.at(i) != nullptr) {
            auto& msg = last_msgs.at(i);
            int camera_id = i + 1;

            // Insert the markers detected in the system
            for (const auto& marker : msg->markers) {
                insertUpdateMarker(marker, camera_id);
            }

            // Adapt the filter for the new data/change of state vector size
            Eigen::VectorXd oldState = kf.getState();

            ROS_INFO_STREAM(tracked_poses);
            ROS_INFO_STREAM(oldState);

            // Verifying if there is a need to extend the state vector (new states)
            if(oldState.size() < tracked_poses * POSE_VECTOR_SIZE) {
                int size_difference = (tracked_poses * POSE_VECTOR_SIZE) - oldState.size();
                oldState.conservativeResize(tracked_poses * POSE_VECTOR_SIZE);
                oldState.tail(size_difference).setZero();
                kf.insertState(oldState);
                // Dont think this is necessary but just in case, considering its being tested
                // Makes so that the pose it's covariance related to the world (initial camera) is 0.
                // kf.resetWorld(camera_poses[1].stateVectorAddr);
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
    }   
}