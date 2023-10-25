#include "ros_filter.h"
#include <tf/transform_broadcaster.h>
#include <vector>
#include <algorithm>

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
        last_msgs[i] = (std::make_shared<aruco_msgs::MarkerArray>());
    }
}

void RosFilter::subscribeTopics() {
    // Create universal publisher for all camera poses
    createPublisherCameras("kalman/filtered_cam_poses", nh);

    // Create publisher for filtered marker data
    createPublisherFiltered("kalman/filtered_markers_poses", nh);

    for (const std::string& camera_topic : topics) {
        int camera_id = extractCameraID(camera_topic);

        insertCameraBasis(camera_id);

        camera_subs[camera_id] = nh.subscribe<aruco_msgs::MarkerArray>(
            camera_topic, 10, boost::bind(&RosFilter::cameraCallback, this, _1, camera_id));
    }
}

ros::Timer RosFilter::createTimer(ros::Duration period) {
    return nh.createTimer(period, &RosFilter::timerCallback, this);
}

void RosFilter::insertCameraBasis(int camera_id) {
    // Verifying if this camera is already tracked. If not, insert in the system
    tf::Transform tf = tf::Transform();
    tf.setIdentity();
    tf.setRotation(tf::Quaternion(0.5,0.5,0.5,0.5));
    if(!camera_poses.contains(camera_id)) {
        cameraBasis aux((tracked_poses++) * POSE_VECTOR_SIZE,
                         tfToPose(tf),
                         Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE),
                         tf::Transform());

        if(camera_id != 1) {
            aux.covariance *= HIGH_COVARIANCE_PRESET;
        }
        camera_poses[camera_id] = aux;
    }
}

void RosFilter::createPublisherFiltered(std::string filtered_marker_topic, ros::NodeHandle nh) {
    filtered_marker_publisher = nh.advertise<aruco_msgs::MarkerArray>(filtered_marker_topic, 5);
}

void RosFilter::createPublisherCameras(std::string camera_topic, ros::NodeHandle nh) {
    camera_publisher = nh.advertise<geometry_msgs::PoseArray>(camera_topic, 5);
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

Eigen::VectorXd RosFilter::msgToPose(geometry_msgs::Pose pose) {
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
    adjust_rotation = tf::Quaternion(0, 0, 1, 0);   // Rotation on the z axis to adjust to our system from what we receive from aruco_ros
    auto adjust_rotation_tf =  tf::Transform(adjust_rotation);
    transform = adjust_rotation_tf * transform;
    return tfToPose(transform);
}

tf::Transform RosFilter::tfToWorldFrame(int this_camera_id){    
    int current_id = this_camera_id;
    int previous_id = camera_poses[current_id].previous_id;
    tf::Transform tf_to_base = camera_poses[current_id].previous_tf;

    while(current_id != previous_id) {
        tf_to_base = camera_poses[previous_id].previous_tf * tf_to_base;
        current_id = previous_id;
        previous_id = camera_poses[current_id].previous_id;
    }
    return tf_to_base;
}

void RosFilter::insertUpdateMarker(aruco_msgs::Marker marker, int camera_id) {
    int markerTagId = marker.id;
    geometry_msgs::Pose pose = marker.pose.pose;

    Eigen::VectorXd poseVector = msgToPose(pose);

    // Verifying if this marker is already tracked by any camera, if not, insert in the system
    int marker_index = -1;
    int marker_found_camera_id;

    for(const auto& [id, marker_list] : cam_markers){
        auto it = std::find_if(marker_list.begin(), marker_list.end(),
                        [markerTagId](const trackedMarker& tracked_marker){
                            return tracked_marker.arucoId == markerTagId;
                        });
        if(it != marker_list.end()) {
            marker_index = std::distance(marker_list.begin(), it);
            marker_found_camera_id = id;
            break;
        }
    }

    if(marker_index == -1) {
        // The marker is new 
        trackedMarker auxMarker;
        auxMarker.arucoId = markerTagId;
        auxMarker.stateVectorAddr = (tracked_poses++) * POSE_VECTOR_SIZE;
        auxMarker.pose = poseVector;
        auxMarker.covariance = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) * HIGH_COVARIANCE_PRESET;

        cam_markers[camera_id].push_back(auxMarker);
    }
    else if(marker_found_camera_id == camera_id) {
        // The marker is already tracked by this camera
        trackedMarker& auxMarker = cam_markers[camera_id][marker_index];

        auxMarker.pose = poseVector;
        //auxMarker.covariance = Eigen::MatrixXd::Identity() * HIGH_COVARIANCE_PRESET;

    }
    else {
        // Marker is being tracked by another camera 

        // Updates the tf_previous from the camera based on marker pose of the another camera with id lesser than this one (otherwise it would fall on the first if)
        camera_poses[camera_id].updateTfPrevious(poseToTf(poseVector), 
                                                 poseToTf(cam_markers[marker_found_camera_id][marker_index].pose),
                                                 marker_found_camera_id);

        // Checks if the marker already exists a the camera
        auto it = std::find_if(cam_markers[camera_id].begin(), cam_markers[camera_id].end(),
                            [markerTagId](const trackedMarker& tracked_marker){
                                return tracked_marker.arucoId == markerTagId;
                            });

        if(it != cam_markers[camera_id].end()) {
            // The marker already exists in a camera 
            trackedMarker& auxMarker = cam_markers[camera_id][std::distance(cam_markers[camera_id].begin(), it)];
            auxMarker.pose = poseVector;
            //auxMarker.covariance = Eigen::MatrixXd::Identity() * HIGH_COVARIANCE_PRESET;

        } else {
            // The marker doesn't exists in a camera
            trackedMarker auxMarker = cam_markers[marker_found_camera_id][marker_index];
            auxMarker.pose = poseVector;
            cam_markers[camera_id].push_back(auxMarker);
        }
    }
}

void RosFilter::insertPosesOnStateVector(Eigen::VectorXd& newState, int cam_id) {
    std::vector<int> inserted_markers;
    // Inserts camera and all it's markers observations into the state
    ROS_INFO("stateVectorAddr: %d", camera_poses[cam_id].stateVectorAddr);
    // Updates camera pose to the world frame
    camera_poses[cam_id].pose = tfToPose(tfToWorldFrame(cam_id).inverse());
    newState.segment(camera_poses[cam_id].stateVectorAddr, POSE_VECTOR_SIZE) = camera_poses[cam_id].pose;

    for(const auto& marker : cam_markers[cam_id]){
        ROS_INFO("stateVectorAddr: %d", marker.stateVectorAddr);
        // Updates marker pose to the world frame
        newState.segment(marker.stateVectorAddr, POSE_VECTOR_SIZE) = tfToPose(tfToWorldFrame(cam_id) * 
                                                                                     poseToTf(marker.pose));
        inserted_markers.push_back(marker.arucoId);                                                                                
    }
    // Insert other cameras and their obsevations into the state
    for(const auto& [id, marker_list] : cam_markers) {
        if(id != cam_id)
        {
            ROS_INFO("stateVectorAddr: %d", camera_poses[id].stateVectorAddr);
            // Updates camera pose to the world frame
            camera_poses[id].pose = tfToPose(tfToWorldFrame(id).inverse());
            newState.segment(camera_poses[id].stateVectorAddr, POSE_VECTOR_SIZE) = camera_poses[id].pose;
            for(const auto& marker : marker_list) {
                // Checks if the marker isn't already inserted by the current camera
                if(std::find(inserted_markers.begin(),
                             inserted_markers.end(), 
                             marker.arucoId) == inserted_markers.end()) 
                {
                    ROS_INFO("stateVectorAddr: %d", marker.stateVectorAddr);
                    // Updates marker pose to the world frame
                    newState.segment(marker.stateVectorAddr, POSE_VECTOR_SIZE) = tfToPose(tfToWorldFrame(id) * poseToTf(marker.pose));
                }
            }
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

geometry_msgs::Pose RosFilter::extractDataFromState(cameraBasis data, Eigen::VectorXd filtered_states) {
    // Extract the filtered data corresponding to this marker
    geometry_msgs::Pose filtered_pose;
    if (data.stateVectorAddr + POSE_VECTOR_SIZE <= filtered_states.size()) {
        Eigen::VectorXd data_pose = filtered_states.segment(data.stateVectorAddr, POSE_VECTOR_SIZE);

        // Create a geometry_msgs::Pose message from the filtered pose data
        filtered_pose.position.x = data_pose(0);
        filtered_pose.position.y = data_pose(1);
        filtered_pose.position.z = data_pose(2);
        filtered_pose.orientation.x = data_pose(3);
        filtered_pose.orientation.y = data_pose(4);
        filtered_pose.orientation.z = data_pose(5);
        filtered_pose.orientation.w = data_pose(6);
    }
    return filtered_pose;
}

void RosFilter::preparePublishData(const std::shared_ptr<aruco_msgs::MarkerArray>& msg, geometry_msgs::PoseArray camera_poses_msg, aruco_msgs::MarkerArray filtered_markers_msg){
    // Retrieve the filtered data from the Kalman filter (kf)
    Eigen::VectorXd filtered_states = kf.getState();
    Eigen::MatrixXd covariance_matrix = kf.getCovariance();

    geometry_msgs::Pose camera_msg;

    aruco_msgs::Marker markerAux;

    for(const auto& [camera_id, camera_basis] : camera_poses){
        camera_msg = extractDataFromState(camera_basis, filtered_states);

        camera_poses_msg.poses.push_back(camera_msg);

        for (const auto& marker : cam_markers[camera_id]) {
            markerAux = extractDataFromState(marker, filtered_states, covariance_matrix);
            markerAux.header = msg->header; // Use the same header as the input marker data
            markerAux.header.frame_id = "cam_1"; // publish everything related to origin (since the system is suposed to have only same referenced data)
            filtered_markers_msg.markers.push_back(markerAux);
        }
    }
}

void RosFilter::publishData(geometry_msgs::PoseArray camera_poses_msg, aruco_msgs::MarkerArray filtered_markers_msg){

    // Publish the filtered marker data
    filtered_marker_publisher.publish(filtered_markers_msg);

    // Publish the filtered camera data
    camera_publisher.publish(camera_poses_msg);
}

void RosFilter::cameraCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, const int& camera_id) {
    auto p  = last_msgs[camera_id];
    *p = *msg;
}

void RosFilter::timerCallback(const ros::TimerEvent& event) {
    geometry_msgs::PoseArray camera_poses_msg;
    aruco_msgs::MarkerArray filtered_markers_msg;

    if(last_msgs.empty()) {
        // Filter the data
        kf.predict();
    }
    else {
        for (const auto& [camera_id, msg]: last_msgs) {
            if(last_msgs[camera_id] != nullptr) {
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
                    // Makes so that the pose's covariance related to the world (initial camera) is 0.
                    // ...
                    // kf.resetWorld(camera_poses[1].stateVectorAddr);
                }

                Eigen::VectorXd newState(tracked_poses * POSE_VECTOR_SIZE);
                insertPosesOnStateVector(newState, camera_id);

                #if DEBUG == true
                    ROS_INFO("cam_marker size: %lu", cam_markers[camera_id].size());
                #endif

                // Filter the data
                kf.predict();
                kf.correct(newState);
            }
        }
    }
    // Construct the to-be-published data structure and publish them
    preparePublishData(last_msgs.begin()->second, camera_poses_msg, filtered_markers_msg); // using the header of the first msg received only for simplification
    publishData(camera_poses_msg, filtered_markers_msg);

    last_msgs.clear();
}

tf::Transform RosFilter::poseToTf( Eigen::VectorXd pose) {
    tf::Vector3 tr(pose(0), pose(1), pose(2));
    tf::Quaternion rot(pose(3), pose(4), pose(5), pose(6));
    return tf::Transform(rot, tr);
}

Eigen::VectorXd RosFilter::tfToPose(tf::Transform transform) {

    Eigen::VectorXd pose(7);  // storing the poses in a vector with the format [xt,yt,zt,xr,yr,zr,wr]

    pose << transform.getOrigin().x(),
                    transform.getOrigin().y(),
                    transform.getOrigin().z(),
                    transform.getRotation().x(),
                    transform.getRotation().y(),
                    transform.getRotation().z(),
                    transform.getRotation().w();

    return pose;
}

Eigen::VectorXd RosFilter::convertPoseToWorldFrame(Eigen::VectorXd pose, int cam_id) { 
    tf::Transform pose_tf =  poseToTf(pose);
    tf::Transform tf_to_world_frame = tfToWorldFrame(cam_id);
    pose_tf =  tf_to_world_frame * pose_tf;
    return tfToPose(pose_tf);
}