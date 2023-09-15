#include "marker_viz.h"

VisualizationHandler::VisualizationHandler(){
    sys_cameras = std::vector<Camera>();
    sys_markers = std::vector<Marker>();
    br = tf::TransformBroadcaster();
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
}

//Add camera to the environment
//Checks for repeated frame_id's
void VisualizationHandler::add_camera(Camera camera){
    
}

// Send all objects transformations as TF objects to use on the RViz environment
// TF_translation = [x,y,z]
// TF_rotation(quaternion) = [x,y,z,w]
void  VisualizationHandler::send_tfs(){
    for(auto it = sys_cameras.begin(); it != sys_cameras.end(); it++){
        br.sendTransform(it->get_tf());
    }
    for(auto it = sys_markers.begin(); it != sys_markers.end(); it++){
        br.sendTransform(it->get_tf());
    }
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

