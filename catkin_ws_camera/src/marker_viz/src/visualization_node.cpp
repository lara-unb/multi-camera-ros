#include <ros/ros.h>
#include "marker_viz.h" 
#include <vector>


int main(int argc, char** argv){
  
  // ROS node configuration
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle n;
  ros::Rate r(30);

  ROS_INFO_STREAM("Starting Node...");
  
  //Setting the system visualization handler
  VisualizationHandler vh = VisualizationHandler(n);
  std::string camera_topic = "kalman/filtered_cam_poses"; 
  std::string markers_topic = "kalman/filtered_markers_poses";
  
  vh.start(camera_topic, markers_topic); 
  
  // Node Loop
  while (ros::ok()){
    vh.clear_markers();
    vh.send_tfs();
    vh.publish_markers();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}