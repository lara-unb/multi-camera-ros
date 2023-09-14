#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include "marker_viz.h"
#include <vector>
#include<iterator> 

// Library effective with Linux
#include <unistd.h>

void callback_sub(const aruco_msgs::MarkerArray& msg){
  ROS_INFO_STREAM("teste");
}

int main(int argc, char** argv){
  // ROS node configuration
  ros::init(argc, argv, "test_viz");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  std::vector<ros::Subscriber> subscribers;

  // Setting the cameras reference frame in the origin 
  tf::TransformBroadcaster br;
  std::vector<Camera> cameras;
  visualization_msgs::MarkerArray arr;
  
  // Subscribe to each camera marker topic
  std::vector<std::string>topics = get_camera_topics();
  ROS_INFO_STREAM("subscribing to " << topics.at(0) << " ..." << std::endl);
  subscribers.push_back(n.subscribe(topics.at(0), 1, callback_sub));

  for(int i = 0; i < topics.size(); i++){
    std::string id = "cam_" + std::to_string(i+1);
    cameras.push_back(Camera(tf::Vector3(i,i,i), tf::Quaternion(0,0,0,1),"cam_" + std::to_string(i+1)));
  }
  
  // Node Loop
  while (ros::ok()){
    arr.markers.clear();
    Marker marker = Marker();
    br.sendTransform(cameras.at(0).get_tf());
    arr.markers.push_back(marker.get_marker());
    marker_pub.publish(arr);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}