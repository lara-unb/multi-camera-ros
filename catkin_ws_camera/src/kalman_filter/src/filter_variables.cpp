#include "filter_variables.h"

cameraBasis::cameraBasis(int address, Eigen::VectorXd pose,Eigen::MatrixXd cv, tf::Transform tf) :
stateVectorAddr(address),
pose(pose),
covariance(cv),
tf_previous(tf)
{
    tf_previous.setIdentity();
}

void updateTfPrevious(cameraBasis previous){
    
}