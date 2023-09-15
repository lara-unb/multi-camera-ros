#include "marker.h"

Marker::Marker(){
    marker = visualization_msgs::Marker();
    
    // Setting the marker initial parameters 
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(2);
    int marker_id = 0;
    
    // Set the parent frame for the relative pose of the marker
    parent_frame = "cam_1";
    marker.header.frame_id = parent_frame;
    
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = parent_frame + "_m_" + std::to_string(marker_id); 
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    
    // Set the pose of the marker. 
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Set the scale of the marker -
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    //Set the TF transfor of the relative marker pose
    tf::Vector3 tr(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    tf::Quaternion rot(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
    tf  = tf::Transform(rot, tr);
}

visualization_msgs::Marker Marker::get_marker(){
    return marker;
}

tf::StampedTransform Marker::get_tf(){
    return tf::StampedTransform(tf, ros::Time::now(),parent_frame, "m_" + std::to_string(marker.id));
}