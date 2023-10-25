#include "marker_viz.h"


VisualizationHandler::VisualizationHandler(ros::NodeHandle& node_handle){
    sys_cameras = std::vector<Camera>();
    sys_markers = std::vector<VisualizationMarker>();
    std::vector<ros::Subscriber> sys_subs = std::vector<ros::Subscriber>();
    nh = node_handle;
    br = tf::TransformBroadcaster();
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
}

//Find marker on all system markers
// Returns marker index if found, else returns -1  
int VisualizationHandler::find_marker(VisualizationMarker m){
    auto marker_it = std::find(sys_markers.begin(), sys_markers.end(), m);
    if(!sys_markers.empty() && marker_it != sys_markers.end()){ // Found the marker
        return std::distance(sys_markers.begin(), marker_it);
    }
    else { // Marker not found or empty vector
        return -1;
    }
}

//Clears all markers which lifetime expired
//Uses the erase-remove idiom
void VisualizationHandler::clear_markers(){
    sys_markers.erase(std::remove_if(sys_markers.begin(), sys_markers.end(), 
    [](const VisualizationMarker& m){return m.get_marker().header.stamp + m.get_marker().lifetime <= ros::Time::now();}), sys_markers.end());
}

// Callback method for the markers subscriber
// Responsible for update the system markers to be published
void VisualizationHandler::callback_markers(const aruco_msgs::MarkerArray& msg){
    for(int i = 0; i < msg.markers.size(); i++){
        VisualizationMarker marker_candidate = VisualizationMarker(msg.markers.at(i));
        int marker_index = find_marker(marker_candidate);
        if(marker_index != -1){ //Marker found: Update marker
            sys_markers.at(marker_index).set_marker(marker_candidate);
        }
        else{ //Marker not found: Add to the list of markers 
            ROS_INFO_STREAM("Adding marker " << marker_candidate.get_marker().id);
            sys_markers.push_back(marker_candidate);
        }
    }  
}

// Callback method for the camera subscriber
// Responsible for update the system cameras to be published
void VisualizationHandler::callback_cameras(const geometry_msgs::PoseArray& msg){
    for(int i = 0; i < msg.poses.size(); i++){
        
        tf::Vector3 tr(msg.poses.at(i).position.x,
                       msg.poses.at(i).position.y,
                       msg.poses.at(i).position.z);
        
        tf::Quaternion rot(msg.poses.at(i).orientation.x,
                           msg.poses.at(i).orientation.y,
                           msg.poses.at(i).orientation.z,
                           msg.poses.at(i).orientation.w);

        if(i >= sys_cameras.size()){
            ROS_INFO_STREAM("Adding camera " << i+1);
            sys_cameras.push_back(Camera(tr, rot, "cam_" + std::to_string(i+1)));  
        }
        else {
            sys_cameras.at(i).set_tf(tr, rot);
        }
    }  
}

void VisualizationHandler::start(std::string camera_topic, std::string markers_topic){
    add_cameras(camera_topic);
    add_markers(markers_topic);
}

// create the respective  ROS  camera subscriber
void VisualizationHandler::add_cameras(std::string topic){
    ROS_INFO_STREAM("Adding cameras...");
    ROS_INFO_STREAM("Subscribing to " << topic << std::endl);
    sys_subs.push_back(nh.subscribe(topic, 5, &VisualizationHandler::callback_cameras, this));
    
}

void VisualizationHandler::add_markers(std::string topic){
    ROS_INFO_STREAM("Adding markers ...");
    ROS_INFO_STREAM("Subscribing to " << topic << std::endl);
    sys_subs.push_back(nh.subscribe(topic, 5, &VisualizationHandler::callback_markers, this));
}


// Send all objects transformations as TF objects to use on the RViz environment
// TF_translation = [x,y,z]
// TF_rotation(quaternion) = [x,y,z,w]
void  VisualizationHandler::send_tfs(){
    for(auto it = sys_cameras.begin(); it != sys_cameras.end(); it++){
        br.sendTransform(it->get_tf_stamped());
    }
    for(auto it = sys_markers.begin(); it != sys_markers.end(); it++){
        br.sendTransform(it->get_tf_stamped());
    }
}

//Publish all markers to the RVIZ environment
void VisualizationHandler::publish_markers(){
    visualization_msgs::MarkerArray arr;
    for(int i = 0; i < sys_markers.size(); i++){
        arr.markers.push_back(sys_markers.at(i).get_marker());
    }
    marker_pub.publish(arr);
}

//Utility Functions

// Find all aruco marker publisher topics for each camera
std::vector<std::string> get_camera_topics(){ 
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string marker_topic_name("/filtered_markers");
    std::vector<std::string>topics;

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if(info.name.find(marker_topic_name) != std::string::npos){
            if(info.name.find("list") == std::string::npos){  // filters "markers_list" topics from the vector
                topics.push_back(info.name);
            }  
        }
    }
    return topics; 
}

// Converts Quaternion to Rotation Matrix (3x3)
// From Ken Shoemake's article "Quaternion Calculus and Fast Animation" 
tf::Matrix3x3  get_rot_from_quat(tf::Quaternion q){
    float norm = q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w();
    float s = (norm >= 0) ? 2.0/norm : 0.0;
    
    float r_xx = 1 - s*(q.y()*q.y() + q.z()*q.z());
    float r_xy = s*(q.x()*q.y() - q.w()*q.z());
    float r_xz = s*(q.x()*q.z() + q.w()*q.y());
    
    float r_yx = s*(q.x()*q.y() + q.w()*q.z());
    float r_yy = 1 - s*(q.x()*q.x() + q.z()*q.z());
    float r_yz = s*(q.y()*q.z() - q.w()*q.x());

    float r_zx = s*(q.x()*q.z() - q.w()*q.y());
    float r_zy = s*(q.y()*q.z() + q.w()*q.x());
    float r_zz = 1 - s*(q.x()*q.x() + q.y()*q.y());

    return tf::Matrix3x3(r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz); 
}

// Converts Rotation Matrix (3x3) to Quaternion
// From Ken Shoemake's article "Quaternion Calculus and Fast Animation" 
tf::Quaternion get_quat_from_rot(tf::Matrix3x3 r){
    float rot[3][3];

    rot[0][0] = r[0].x(); // rxx
    rot[0][1]= r[0].y(); // rxy
    rot[0][2] = r[0].z();// rxz
    
    rot[1][0] = r[1].x(); // ryx
    rot[1][1] = r[1].y(); // ryy
    rot[1][2] = r[1].z(); // ryz

    rot[2][0] = r[2].x(); // rzx
    rot[2][1] = r[2].y(); // rzy
    rot[2][2] = r[2].z(); // rzz

    float tr = rot[0][0] + rot[1][1] + rot[2][2];
   
    if(tr >= 0.0){
        float q_x,q_y,q_z,q_w;
        float s = std::sqrt(tr + 1.0);
        q_w = s*0.5;
        s = 0.5/s;
        q_x = (rot[2][1] - rot[1][2])*s;
        q_y = (rot[0][2] - rot[2][0])*s;
        q_z = (rot[1][0] - rot[0][1])*s;
        return tf::Quaternion(q_x, q_y, q_z, q_w);
    }
    else {
        int i = 0;
        float q[4]; 
        if(rot[1][1] > rot[0][0]){
            i = 1;
        }
        if (rot[2][2] > rot[i][i]){
            i = 2;
        }
        int j = (i+1)%3; 
        int k = (j+1)%3;      
        float s = sqrt(rot[i][i] - rot[j][j] - rot[k][k] + 1.0);
        q[i] = s*0.5;
        s = 0.5/s;
        q[j] = (rot[i][j] + rot[j][i]) * s;
        q[k] = (rot[k][i] + rot[i][k]) * s;
        q[3] = (rot[k][j] - rot[j][k]) * s;
        return tf::Quaternion(q[0], q[1], q[2], q[3]);
    }
}

//Quaternion conversion verification routine 
void test_quaternion(tf::Quaternion q){

    auto converted_q = get_quat_from_rot(get_rot_from_quat(q));
    
    std::cout << "q (Original):"
    <<"             x= " << q.x()
    << " y= " << q.y() 
    << " z = " << q.z()
    << " w= " << q.w() << std::endl;
    
    std::cout << "q (from Rotation Matrix):"
    << " x= " << converted_q.x()
    << " y= " << converted_q.y() 
    << " z = " << converted_q.z()
    << " w= " << converted_q.w() << std::endl;

}
