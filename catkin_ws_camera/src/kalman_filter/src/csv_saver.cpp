#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>


void saveReadingAruco(const aruco_msgs::MarkerArray& msg){
    for(const auto& marker: msg.markers){
        std::ofstream file;
        std::string path("./readings/unfiltered/markers/m_" + std::to_string(marker.id) + "_" + marker.header.frame_id + ".csv");
        file.open(path, std::ios::out | std::ios::app);
        file << (std::to_string(marker.header.stamp.sec) + "." + std::to_string(marker.header.stamp.nsec)) << ",";
        file << marker.pose.pose.position.x << ","
             << marker.pose.pose.position.y << "," 
             << marker.pose.pose.position.z << ","
             << marker.pose.pose.orientation.x << ","
             << marker.pose.pose.orientation.y << ","
             << marker.pose.pose.orientation.z << ","
             << marker.pose.pose.orientation.w << "\n"; 
        file.close();
    }
}

void saveReadingKalmanCamera(const geometry_msgs::PoseArray& msg){
    for(int i = 0; i <  msg.poses.size(); i++){
        std::ofstream file;
        auto camera = msg.poses[i];
        std::string path("./readings/filtered/cameras/cam_" + std::to_string(i+1) + ".csv");
        file.open(path, std::ios::out | std::ios::app);
        file << (std::to_string(msg.header.stamp.sec) + "." + std::to_string(msg.header.stamp.nsec)) << ",";
        file << camera.position.x << ","
             << camera.position.y << "," 
             << camera.position.z << ","
             << camera.orientation.x << ","
             << camera.orientation.y << ","
             << camera.orientation.z << ","
             << camera.orientation.w << "\n"; 
        file.close();
    }
}

void saveReadingKalmanMarkers(const aruco_msgs::MarkerArray& msg){
    for(const auto& marker: msg.markers){
        std::ofstream file;
        std::string path("./readings/filtered/markers/m_" + std::to_string(marker.id) + "_" + marker.header.frame_id + ".csv");
        file.open(path, std::ios::out | std::ios::app);
        file << (std::to_string(marker.header.stamp.sec) + "." + std::to_string(marker.header.stamp.nsec)) << ",";
        file << marker.pose.pose.position.x << ","
             << marker.pose.pose.position.y << "," 
             << marker.pose.pose.position.z << ","
             << marker.pose.pose.orientation.x << ","
             << marker.pose.pose.orientation.y << ","
             << marker.pose.pose.orientation.z << ","
             << marker.pose.pose.orientation.w << "\n"; 
        file.close();
    }
}

void subscribeArucoMarkersTopics(ros::NodeHandle& nh, std::vector<ros::Subscriber>& subs) {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string marker_topic_name("/aruco_marker_publisher/markers");

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if(info.name.find(marker_topic_name) != std::string::npos){
            if(info.name.find("list") == std::string::npos){  // filters "markers_list" topics from the vector
                ROS_INFO_STREAM("Subscribing to " << info.name);
                subs.push_back(nh.subscribe(info.name, 1, saveReadingAruco));
            }  
        }
    }
}

void clear(){
    std::filesystem::path aruco_markers("./readings/unfiltered/markers/");
    std::filesystem::path kalman_markers("./readings/filtered/markers/");
    std::filesystem::path kalman_cameras("./readings/filtered/cameras/"); 
    
    std::filesystem::remove_all(aruco_markers);
    std::filesystem::remove_all(kalman_markers);
    std::filesystem::remove_all(kalman_cameras);

    std::filesystem::create_directory(aruco_markers);
    std::filesystem::create_directory(kalman_markers); 
    std::filesystem::create_directory(kalman_cameras); 

}

void subscribeKalmanTopics(ros::NodeHandle& nh, std::vector<ros::Subscriber>& subs) {
    std::string kalman_camera("kalman/filtered_cam_poses");
    std::string kalman_markers("kalman/filtered_markers_poses");
    
    ROS_INFO_STREAM("Subscribing to " << kalman_camera);
    subs.push_back(nh.subscribe(kalman_camera, 1, saveReadingKalmanCamera));
    ROS_INFO_STREAM("Subscribing to " << kalman_markers);
    subs.push_back(nh.subscribe(kalman_markers, 1, saveReadingKalmanMarkers));
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_saver");
    ROS_INFO_STREAM("Saving system readings!");
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subs;
    clear();
    subscribeArucoMarkersTopics(nh, subs);
    subscribeKalmanTopics(nh, subs);
    ros::spin();
    return 0;
}