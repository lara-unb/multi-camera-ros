#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_broadcaster.h>

#define POSE_VECTOR_SIZE 7

// Structure to correlate marker poses with aruco id tags
typedef struct trackedMarker {
    int stateVectorAddr;    // index where its found in the state vector of the kalman filter
    int arucoId;            // aruco tag id
    Eigen::VectorXd pose;
    Eigen::MatrixXd covariance;
}trackedMarker;

// Class to implement pose as Vector-Quaternion pair
// Used because tf::Transfornm yielded unsatisfactory results 
class poseTransform{
    public: 
        tf::Vector3 tf_tr;
        tf::Quaternion tf_q;
    
        poseTransform(tf::Quaternion q, tf::Vector3 v);
        poseTransform(tf::Quaternion& q);
        poseTransform();
        
        static tf::Vector3 vector_transformation(tf::Quaternion q, tf::Vector3 tr);
        poseTransform inverse();
        poseTransform operator*(poseTransform const& tf); // Composition operator    
};   

// Class to save a camera's pose
class cameraBasis {
    public:
        int stateVectorAddr;        // index where its found in the state vector of the kalman filter
        Eigen::VectorXd pose;
        Eigen::MatrixXd covariance;
        poseTransform previous_tf;  //relate to the pose of the previous camera. ex.: if it's cam3, it will give the tf to the cam2 pose.
        int previous_id;
        
        cameraBasis();
        cameraBasis(int address, Eigen::VectorXd pose, Eigen::MatrixXd cv, poseTransform tf);
        void updateTfPrevious(poseTransform new_marker_tf, poseTransform  marker_tf_from_previous, int new_previous_id);
};