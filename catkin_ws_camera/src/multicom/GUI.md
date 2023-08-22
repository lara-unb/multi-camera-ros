# GUI

## Overview

This package offers a graphical user interface (GUI) to manage ROS nodes, topics, services, parameters, and launch files in a ROS network. Combined with other tools of the multimaster_fkie stack it is possible to operate a network with multiple masters.

![multimaster fkie diagram](/images/node_manager_overview.png)


## Usage
[ROS Network](http://fkie.github.io/multimaster_fkie/chapter_usage/ros_network.html)     
[Host description panel](http://fkie.github.io/multimaster_fkie/chapter_usage/host_description_panel.html)   
[ROS Nodes view and control](http://fkie.github.io/multimaster_fkie/chapter_usage/ros_nodes_view_and_control.html)    
[ROS Topics view](http://fkie.github.io/multimaster_fkie/chapter_usage/ros_topics_view.html)    
[ROS Services view](http://fkie.github.io/multimaster_fkie/chapter_usage/ros_services_view.html)    
[ROS Parameter view](http://fkie.github.io/multimaster_fkie/chapter_usage/ros_parameter_view.html)    
[Launch Dock](http://fkie.github.io/multimaster_fkie/chapter_usage/launch_dock.html)    
[Description Dock](http://fkie.github.io/multimaster_fkie/chapter_usage/description_dock.html)  
[Capabilities and additional description](http://fkie.github.io/multimaster_fkie/node_manager.html)   
[Node Manager Daemon](http://fkie.github.io/multimaster_fkie/node_manager_daemon.html)   
[Capability View](http://fkie.github.io/multimaster_fkie/chapter_usage/capability_view.html)      
[Settings](http://fkie.github.io/multimaster_fkie/chapter_usage/settings.html)      
[Key Bindings](http://fkie.github.io/multimaster_fkie/chapter_usage/key_bindings.html)      

##### Run the GUI
To run the GUI, you must install the multimaster that is shown [Readme.txt](https://bitbucket.org/automni/multicom/src/master/Readme.md) file.

Now, it is necessary to go to the workspace.

    cd 
    cd catkin_ws
Setup the directory

    source/deve/setup.bash
run the node [node_manager](http://fkie.github.io/multimaster_fkie/node_manager.html)

    rosrun fkie_node_manager node_manager

    







## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/automni/multicom/issues?status=new&status=open).
