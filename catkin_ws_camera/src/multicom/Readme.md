# multicom

## Overview

The purpose of this package is to control and configure the communication between a fleet of Rhinos using the packega multimaster_fkie. 
The package contains the launch files and configuration scripts needed to achieve this feature.
This package was developed using the Rhino's docker image.

### License

The source code is private code.

**Authors: Rafael Ramos**

The multicom package has been tested under Ubuntu 18.04, [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) and Gazebo 9.
This is a research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Usage

### Dependencies

#### Multimaster FKIE Meta-package
![multimaster fkie diagram](/images/multimaster.png)

##### Multimaster packages:

* [Master Discovery](http://fkie.github.io/multimaster_fkie/master_discovery.html) -- Discovery using multicast or zeroconf          
* [Master Synchronization](http://fkie.github.io/multimaster_fkie/master_sync.html) -- Synchronize local ROS master to remote ROS masters     
* [Node Manager](http://fkie.github.io/multimaster_fkie/node_manager.html) -- A GUI to manage the configuration on local and remote ROS masters   
* [Node Manager Daemon](http://fkie.github.io/multimaster_fkie/node_manager_daemon.html) -- Helper node allows an easy launch file management and (auto)start of {remote}nodes

##### Multimaster idea:
* The [master_discovery](http://fkie.github.io/multimaster_fkie/master_discovery.html) node connects to the ROS-Master, gets changes by polling (1.) and publish the changes (multicast or/and unicast) over the network (2.). The received changes of the remote ROS-Master are published to the local ROS topics (3.).
* The [master_sync](http://fkie.github.io/multimaster_fkie/master_sync.html) node connects to the discovered master_discovery nodes (4.), requests the actual ROS state and registers the remote topics/service by local ROS master (5.).
* The [node_manager](http://fkie.github.io/multimaster_fkie/node_manager.html) simplifies launching and managing the ROS multi-master system. It also offers options for managing nodes, topics, services, parameters and launch files through [node_manager_daemon](http://fkie.github.io/multimaster_fkie/node_manager.html). Can also be used in a single ROS master system!

##### Install PIP
For all dependences: yes for all

    sudo apt install python-pip 
    

##### Mulitmaster install
Before compiling the multimaster package, install its dependecies.

    sudo add-apt-repository ppa:roehling/grpc
    sudo apt update
    sudo apt install python-grpcio python-grpc-tools

Clone the stack into your workspace.

    cd catkin_ur3_ws/src
    git clone https://github.com/fkie/multimaster_fkie.git multimaster
    rosdep update
    rosdep install -i --as-root pip:false --reinstall --from-paths multimaster

Then build all packages:

    cd ..
    catkin build fkie_multimaster

### Config files

There is none, yet :)


### Launch files

* For Server
Inside the server's docker.
Start the `server.launch`:

    `roslaunch multicom server.launch`

* For Rhino
Inside the rhino's docker:
Start the `rhino.launch`:

    `roslaunch multicom rhino.launch`
 

## Nodes

There is none, yet :)


## Road map

The Multimaster works in an ideal network as three docker containers running inside the same machine.
Then the next steps to deploy in a real-world fleet of robots are:

- :white_check_mark: Test the packages in an ideal network.
- :white_check_mark: Test only the communication between different machines using the package in a not ideal network.
- :white_check_mark: Set and test a VPN and tunneling the package communication through the VPN.
- :white_check_mark: Test if the package allows only a few topics and services to be shared.
- :white_check_mark: Test the sync between published and subscribed topics and services.
- :heavy_check_mark: Implement security.

## Experiments

The description of the experiments are in the experiments dir

* [XP1](experiments/experiment1.md)
* [XP2](experiments/experiment2.md)
* [XP3](experiments/experiment3.md)
* [XP4](experiments/experiment4.md)


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/Gastd/multicom/issues).
