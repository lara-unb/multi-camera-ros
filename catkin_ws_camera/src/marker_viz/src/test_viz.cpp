#include <ros/ros.h>
#include "marker_viz.h"
#include <vector>


int main(int argc, char** argv){
  
  // ROS node configuration
  ros::init(argc, argv, "test_viz");
  ros::NodeHandle n;
  ros::Rate r(30);

  ROS_INFO_STREAM("Starting Node...");
  
  //Setting the system visualization handler
  VisualizationHandler vh = VisualizationHandler(n);
  std::vector<std::string>topics = get_camera_topics();
  
  for(int i = 0; i < topics.size(); i++){
    std::string camera_id = topics.at(i).substr(1,5); // get the "cam_x" token for the respecitve camera from topic path
    vh.add_camera(Camera(tf::Vector3(i,i,i), tf::Quaternion(0.5,0.5,0.5,0.5),camera_id), topics.at(i)); // Rotate rviz camera axes to coincide with aruco axes
  }

  // Node Loop
  while (ros::ok()){
    // vh.clear_markers();
    vh.send_tfs();
    vh.publish_markers();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}