#include "filter_variables.h"

cameraBasis::cameraBasis():
stateVectorAddr(0),
pose(Eigen::VectorXd::Zero(7)),
covariance(Eigen::MatrixXd::Identity(7, 7)),
previous_tf(tf::Transform()),
previous_id(1)
{
}

cameraBasis::cameraBasis(int address, Eigen::VectorXd pose, Eigen::MatrixXd cv, tf::Transform tf):
stateVectorAddr(address),
pose(pose),
covariance(cv),
previous_tf(tf),
previous_id(1)
{
    previous_tf.setIdentity();
}

void cameraBasis::updateTfPrevious(tf::Transform new_marker_tf, tf::Transform  marker_tf_from_previous, int new_previous_id) {
    previous_id = new_previous_id;
    new_marker_tf = marker_tf_from_previous.inverse() * new_marker_tf; //Pode ser que a ordem esteja invertida 
    previous_tf =  new_marker_tf;
}

// ESTADOS TESTADOS
// new_marker_tf * marker_tf_from_previous.inverse();
//  new_marker_tf.inverse() * marker_tf_from_previous
//marker_tf_from_previous * new_marker_tf.inverse(); 