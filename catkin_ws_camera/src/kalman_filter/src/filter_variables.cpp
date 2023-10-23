#include "filter_variables.h"

cameraBasis::cameraBasis(int address, Eigen::VectorXd pose,Eigen::MatrixXd cv, tf::Transform tf) :
stateVectorAddr(address),
pose(pose),
covariance(cv),
tf_previous(tf),
previous_id(1)
{
    tf_previous.setIdentity();
}

void cameraBasis::updateTfPrevious(tf::Transform new_marker_tf, tf::Transform  marker_tf_from_previous, int new_previous_id) {
    previous_id = new_previous_id;
    new_marker_tf = marker_tf_from_previous.inverse() * new_marker_tf;
    tf_previous =  new_marker_tf;
}