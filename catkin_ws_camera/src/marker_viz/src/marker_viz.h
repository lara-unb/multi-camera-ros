#ifndef MARKER_VIZ_H
#define MARKER_VIZ_H
#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>

class Camera {
    tf::Transform tf;
    std::string frame_id;

    public:
        Camera(std::string id);
        Camera(tf::Vector3 tr, tf::Quaternion rot, std::string id);
        tf::StampedTransform get_stamped_tf();
        void set_tf(tf::Vector3 tr, tf::Quaternion rot);


};

std::vector<std::string> get_camera_topics();


#endif