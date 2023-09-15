#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

  //Setting the system visualization handler
  VisualizationHandler vh = VisualizationHandler();
  std::vector<std::string>topics = get_camera_topics();
  
  for(int i = 0; i < topics.size(); i++){
    std::string camera_id = topics.at(i).substr(0, s.find("/")); // get the "cam_x" token for the respecitve camera from topic path
    print(camera_id)
    vh.add_camera(Camera(tf::Vector3(i,i,i), tf::Quaternion(0,0,0,1),camera_id));
    vh.subscribe(topics.at(i));
  }

  // ROS_INFO_STREAM("subscribing to " << topics.at(0) << " ..." << std::endl);
  
  // Node Loop
  while (ros::ok()){
    vh.send_tfs();
    vh.publish_markers();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}