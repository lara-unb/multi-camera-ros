#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "aruco_ros_utils.h"
#include <tf/transform_broadcaster.h>
#include <vector>
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_viz");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  // Setting first camera reference frame in the origin 
  tf::TransformBroadcaster br;
  std::vector<tf::Transform> tf_cameras;
  tf_cameras.push_back(tf::Transform());
  visualization_msgs::MarkerArray arr;
  int pei = 1000;
  
  while (ros::ok())
  {
    arr.markers.clear();
    tf_cameras.at(0).setOrigin(tf::Vector3(0,0,0));
    tf_cameras.at(0).setRotation(tf::Quaternion(0,0,0,1)); 
    br.sendTransform(tf::StampedTransform(tf_cameras.at(0), ros::Time::now(),"map", "cam_1"));
    
    
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_2;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "cam_1";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "test_viz";
    marker.id = 0;

    // Set the marker type.
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;


    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_2 = marker;
    marker_2.ns = "test_viz";
    marker_2.id = 1;
    marker_2.pose.position.x = 1.0;
    marker_2.pose.position.z = 1.0;
    marker_2.pose.position.y = 1.0;
    marker_2.lifetime = ros::Duration(60);
    
    if(pei > 0){
      arr.markers.push_back(marker_2);
      pei--;
    }
      
   
    arr.markers.push_back(marker);
    
    marker_pub.publish(arr);
    r.sleep();
  }
}