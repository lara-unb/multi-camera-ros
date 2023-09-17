#include "marker_viz.h"


VisualizationHandler::VisualizationHandler(ros::NodeHandle& node_handle){
    sys_cameras = std::vector<Camera>();
    sys_markers = std::vector<VisualizationMarker>();
    std::vector<ros::Subscriber> sys_subs = std::vector<ros::Subscriber>();
    nh = node_handle;
    br = tf::TransformBroadcaster();
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
}

//Find marker on all system markers
// Returns marker index if found, else returns -1  
int VisualizationHandler::find_marker(VisualizationMarker m){
    auto marker_it = std::find(sys_markers.begin(), sys_markers.end(), m);
    if(!sys_markers.empty() && marker_it != sys_markers.end()){ // Found the marker
        return std::distance(sys_markers.begin(), marker_it);
    }
    else { // Marker not found or empty vector
        return -1;
    }
}

//Clears all markers which lifetime expired
//Uses the erase-remove idiom
void VisualizationHandler::clear_markers(){
    sys_markers.erase(std::remove_if(sys_markers.begin(), sys_markers.end(), 
    [](const VisualizationMarker& m){return m.get_marker().header.stamp + m.get_marker().lifetime <= ros::Time::now();}), sys_markers.end());
}

// Callback method for the camera subscriber
// Responsible for update the system markers to be published
void VisualizationHandler::callback(const aruco_msgs::MarkerArray& msg){
    for(int i = 0; i < msg.markers.size(); i++){
        VisualizationMarker marker_candidate = VisualizationMarker(msg.markers.at(i));
        // Inverts transformation of coordinates to marker->camera to camera->marker for visualization 
        marker_candidate.set_pose(marker_candidate.get_tf().inverse());  
        int marker_index = find_marker(marker_candidate);
        if(marker_index != -1){ //Marker found: Update marker
            ROS_INFO_STREAM("marker ind:  " << marker_index );
            sys_markers.at(marker_index).set_marker(marker_candidate);
        }
        else{ //Marker not found: Add to the list of markers 
            sys_markers.push_back(marker_candidate);
        }
    }  
}

// Add camera to the environment and create the respective  ROS subscriber
// Checks for repeated frame_id's
void VisualizationHandler::add_camera(Camera camera, std::string topic){
    bool flag_repeated_value = false;
    for(int i  = 0; i < sys_cameras.size(); i++){
        if(sys_cameras.at(i) == camera){
            flag_repeated_value = false;
            ROS_WARN("Camera already exists in the system");
        }
    }
    if(!flag_repeated_value){
        ROS_INFO_STREAM("Adding camera " << camera.get_frame_id());
        sys_cameras.push_back(camera);
        ROS_INFO_STREAM("Subscribing to " << topic << std::endl);
        sys_subs.push_back(nh.subscribe(topic, 5, &VisualizationHandler::callback, this));
    }
}


// Send all objects transformations as TF objects to use on the RViz environment
// TF_translation = [x,y,z]
// TF_rotation(quaternion) = [x,y,z,w]
void  VisualizationHandler::send_tfs(){
    for(auto it = sys_cameras.begin(); it != sys_cameras.end(); it++){
        br.sendTransform(it->get_tf_stamped());
    }
    for(auto it = sys_markers.begin(); it != sys_markers.end(); it++){
        br.sendTransform(it->get_tf_stamped());
    }
}

//Publish all markers to the RVIZ environment
void VisualizationHandler::publish_markers(){
    visualization_msgs::MarkerArray arr;
    for(int i = 0; i < sys_markers.size(); i++){
        arr.markers.push_back(sys_markers.at(i).get_marker());
    }
    marker_pub.publish(arr);
}

//Utility Functions

// Find all aruco marker publisher topics for each camera
std::vector<std::string> get_camera_topics(){ 
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string marker_topic_name("/aruco_marker_publisher/markers");
    std::vector<std::string>topics;
    
    
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if(info.name.find(marker_topic_name) != std::string::npos){
            if(info.name.find("list") == std::string::npos){  // filters "markers_list" topics from the vector
                topics.push_back(info.name);
            }  
        }
    }
    return topics; 
}

