#include "camera.h"

Camera::Camera(std::string id){
    tf = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    frame_id = id;
}

Camera::Camera(tf::Vector3 tr, tf::Quaternion rot, std::string id){
    tf = tf::Transform(rot, tr);
    frame_id = id;
}

tf::StampedTransform Camera::get_tf(){
    return tf::StampedTransform(tf, ros::Time::now(),"map", frame_id);
}

void Camera::set_tf(tf::Vector3 tr, tf::Quaternion rot){
    tf.setOrigin(tr);
    tf.setRotation(rot);
}