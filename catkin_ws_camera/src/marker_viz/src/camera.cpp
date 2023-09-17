#include "camera.h"

Camera::Camera(std::string id){
    tf = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    frame_id = id;
}

Camera::Camera(tf::Vector3 tr, tf::Quaternion rot, std::string id){
    tf = tf::Transform(rot, tr);
    frame_id = id;
}

tf::StampedTransform Camera::get_tf_stamped() const {
    return tf::StampedTransform(tf, ros::Time::now(),"map", frame_id);
}

bool Camera::operator==(const Camera& obj){
    return (this->frame_id == obj.frame_id) ? true : false;
}

void Camera::set_tf(tf::Vector3 tr, tf::Quaternion rot){
    tf.setOrigin(tr);
    tf.setRotation(rot);
}

std::string Camera::get_frame_id() const {
    return frame_id;
}