#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

// Structure to correlate marker poses with aruco id tags
typedef struct trackedMarker {
    int stateVectorAddr; // index where its found in the state vector of the kalman filter
    int arucoId;         // aruco tag id
    Eigen::VectorXd pose;
    Eigen::MatrixXd covariance;
}trackedMarker;

// Class to save a camera's pose
class cameraBasis {
    public:
        int stateVectorAddr;
        Eigen::VectorXd pose;
        Eigen::MatrixXd covariance;

        cameraBasis();
};