#include "marker_viz.h"

// Camera
Camera::Camera(std::string id){
    tf = tf::Transform();
    tf.setOrigin(tf::Vector3(0,0,0));
    tf.setRotation(tf::Quaternion(0,0,0,1));
    frame_id = id;
}

Camera::Camera(tf::Vector3 tr, tf::Quaternion rot, std::string id){
    tf = tf::Transform();
    tf.setOrigin(tr);
    tf.setRotation(rot);
    frame_id = id;
}

tf::StampedTransform Camera::get_stamped_tf(){
    return tf::StampedTransform(tf, ros::Time::now(),"map", frame_id);
}

void Camera::set_tf(tf::Vector3 tr, tf::Quaternion rot){
    tf.setOrigin(tr);
    tf.setRotation(rot);
}

// Marker

//CameraHandler

//MarkerHandler


//Utils
std::vector<std::string> get_camera_topics(){ 
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string marker_topic_name("/aruco_marker_publisher/markers");
    std::vector<std::string>topics;
    
    // Find all camera marker topics
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

