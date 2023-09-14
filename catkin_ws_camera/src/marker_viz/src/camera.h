#ifndef CAMERA_H
#define CAMERA_H

#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>

class Camera {
    tf::Transform tf;
    std::string frame_id;

    public:
        Camera(std::string id);
        Camera(tf::Vector3 tr, tf::Quaternion rot, std::string id);
        
        tf::StampedTransform get_tf();
        void set_tf(tf::Vector3 tr, tf::Quaternion rot);

};
#endif