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

// Class to save a camera's pose
class cameraBasis {
    public:
        int stateVectorAddr;        // index where its found in the state vector of the kalman filter
        Eigen::VectorXd pose;
        Eigen::MatrixXd covariance;
        tf::Transform tf_previous;  //relate to the pose of the previous camera. ex.: if it's cam3, it will give the tf to the cam2 pose.

        cameraBasis(int address, Eigen::VectorXd pose, Eigen::MatrixXd cv, tf::Transform tf);
        void updateTfPrevious(cameraBasis previous);
    
};