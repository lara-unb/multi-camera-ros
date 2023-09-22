#include "visualization_marker.h"

VisualizationMarker::VisualizationMarker(){
    marker = visualization_msgs::Marker();

    // Setting the marker initial parameters 
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(2);
    int marker_id = 0;
    parent_frame = "cam_1";

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.header.frame_id = parent_frame;
    marker.id = marker_id;
    marker.ns = marker.header.frame_id + "_m_" + std::to_string(marker.id); 
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

    //Set the TF transform of the relative marker pose
    tf::Vector3 tr(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    tf::Quaternion rot(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
    tf  = tf::Transform(rot, tr);
}

VisualizationMarker::VisualizationMarker(aruco_msgs::Marker m){
    marker = visualization_msgs::Marker();

    // Setting the marker initial parameters 
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(2);
    int marker_id = m.id;
    parent_frame = m.header.frame_id;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.header.frame_id = parent_frame;
    marker.id = marker_id;
    marker.ns = marker.header.frame_id + "_m_" + std::to_string(marker.id); 
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the pose of the marker. 
    marker.pose.position.x = m.pose.pose.position.x;
    marker.pose.position.y = m.pose.pose.position.y;
    marker.pose.position.z = m.pose.pose.position.z;
    marker.pose.orientation.x = m.pose.pose.orientation.x;
    marker.pose.orientation.y = m.pose.pose.orientation.y;
    marker.pose.orientation.z = m.pose.pose.orientation.z;
    marker.pose.orientation.w = m.pose.pose.orientation.w;

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

visualization_msgs::Marker VisualizationMarker::get_marker() const {
    return marker;
}

tf::Transform VisualizationMarker::get_tf() const {
    return tf;
}

tf::StampedTransform VisualizationMarker::get_tf_stamped() const {
    return tf::StampedTransform(tf, ros::Time::now(), parent_frame, "m_" + std::to_string(marker.id));
}

bool VisualizationMarker::operator==(const VisualizationMarker& obj){
   return (this->marker.ns == obj.marker.ns) ? true : false; 
}

void VisualizationMarker::set_marker(VisualizationMarker m){
    marker = m.marker;
    tf.setOrigin(tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
    tf.setRotation(tf::Quaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w));
}

void VisualizationMarker::set_pose(tf::Transform new_pose){
    tf = new_pose;
    marker.pose.position.x = new_pose.getOrigin().x();
    marker.pose.position.y = new_pose.getOrigin().y();
    marker.pose.position.z = new_pose.getOrigin().z();
    marker.pose.orientation.x = new_pose.getRotation().x();
    marker.pose.orientation.y = new_pose.getRotation().y();
    marker.pose.orientation.z = new_pose.getRotation().z();
    marker.pose.orientation.w = new_pose.getRotation().w();
}