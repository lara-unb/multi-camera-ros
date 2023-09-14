#include "marker_viz.h"

//CameraHandler

//MarkerHandler


//Utils

// Find all camera marker topics
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

