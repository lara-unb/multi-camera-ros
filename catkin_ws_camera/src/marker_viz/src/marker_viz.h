#ifndef MARKER_VIZ_H
#define MARKER_VIZ_H

#include <ros/ros.h>
#include <vector>
#include "marker.h"
#include "camera.h"
#include <tf/transform_broadcaster.h>
#include <iterator>

class VisualizationHandler{    
    std::vector<Camera> sys_cameras;
    std::vector<Marker> sys_markers;
    tf::TransformBroadcaster br;
    ros::Publisher marker_pub;
    
    public:
        VisualizationHandler();
        void add_camera(Camera camera);
        void send_tfs();
};

std::vector<std::string> get_camera_topics();


#endif