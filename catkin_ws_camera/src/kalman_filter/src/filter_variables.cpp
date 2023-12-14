#include "filter_variables.h"

cameraBasis::cameraBasis():
stateVectorAddr(0),
pose(Eigen::VectorXd::Zero(7)),
covariance(Eigen::MatrixXd::Identity(7, 7)),
previous_tf(poseTransform()),
previous_id(1)
{
}

poseTransform::poseTransform(tf::Quaternion q, tf::Vector3 v):
tf_tr(v),
tf_q(q)
{

};

poseTransform::poseTransform(tf::Quaternion& q):
tf_tr(),
tf_q(q)
{
    tf_tr = tf::Vector3(0,0,0);
};

poseTransform::poseTransform():
tf_tr(tf::Vector3(0,0,0)),
tf_q(tf::Quaternion(0,0,0,1))
{

};

tf::Vector3 poseTransform::vector_transformation(tf::Quaternion q, tf::Vector3 tr){
    tf::Quaternion q_tr = tf::Quaternion(tr.x(),tr.y(), tr.z(), 0);
    tf::Quaternion q_result = q*q_tr*q.inverse();
    return tf::Vector3(q_result.x(),q_result.y(),q_result.z());
}

poseTransform poseTransform::inverse(){
    tf::Vector3 tr_inverted = vector_transformation(tf_q.inverse(),tf_tr);
    tr_inverted *= -1;
    tf::Quaternion q_inverted = tf_q.inverse();
    return poseTransform(q_inverted, tr_inverted);  
}

poseTransform poseTransform::operator*(poseTransform const& tf2){
    tf::Vector3 transformed_tr2 = vector_transformation(tf_q, tf2.tf_tr);
    tf::Vector3 new_tr = tf::Vector3(tf_tr.x() + transformed_tr2.x(),
                                     tf_tr.y() + transformed_tr2.y(),
                                     tf_tr.z() + transformed_tr2.z());
    tf::Quaternion new_q = tf_q*tf2.tf_q;                                
    return poseTransform(new_q,new_tr);
}

cameraBasis::cameraBasis(int address, Eigen::VectorXd pose, Eigen::MatrixXd cv, poseTransform tf):
stateVectorAddr(address),
pose(pose),
covariance(cv),
previous_tf(tf),
previous_id(1)
{
    previous_tf.tf_tr = tf::Vector3(0,0,0);
    previous_tf.tf_q = tf::Quaternion(0,0,0,1);
}

void cameraBasis::updateTfPrevious(poseTransform new_marker_tf, poseTransform  marker_tf_from_previous, int new_previous_id) {
    previous_id = new_previous_id;
    previous_tf = new_marker_tf * marker_tf_from_previous.inverse();
    previous_tf = previous_tf;
}