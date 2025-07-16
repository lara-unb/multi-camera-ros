> This tutorial is prepared for an environment of a Ubuntu 20.04. If you want to use on a different Ubuntu version, you need to change everything that containts 'noetic' to the apropriate correspondent and check for possible regressions.

# Preparing the environment

To prepare the environment, you have to individually install freenect ros lib manually. The overall process is quite simple so you just have to do the following steps:

Installing some dependencies:

```
sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
cd catkin_ws_camera/src
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build
cd build
cmake -L ..
make
```
Now you have to manually install freenect stack, which is the libfreenect ROS driver. Again, follow the steps:
```
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/freenect_stack.git
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro noetic
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
```
Now that freenect is properly installed you can use the kinect cameras. But the environment will need usb_cam library to suppor usb cameras too. Follow the steps:

`sudo apt-get install ros-noetic-usb-cam`

Now, to use the aruco markers ros package do the following:

`sudo apt-get install ros-noetic-aruco-ros`

Make sure you have installed the Eigen3 lib so the Kalman filter can work. Do the following:

`sudo apt-get install libeigen3-dev`

With those libs installed, you can build the entire environment together from the workspace folder. To do that, run the following commands:
```
cd ~/catkin_ws/
catkin_make
```
# Run the project
To run the project, firstly is necessary run the setup script for setup of the plugged cameras on ROS:

`python3 start_cameras.py` 

After that, is necessary run the ROS nodes of the kalman filter and the marker visualization, given by the following commands, respectively:
```
rosrun kalman_filter ros_kalman_filter
rosrun marker_viz visualization_node
```
Finally, the system visualization can be observer running the rviz ROS node:

`rosrun rviz rviz
`

# New planned features and fixes

At the moment, the system is only capable of utilizing cameras set on different usb buses. We plan to find a workaround, if possible. The solution will probably involve:
 - Reducing the quality of the video of each camera
 - Requiring a new hardware setup for the computer
 - Support for new protocols like firewire.

We also need to address an eventual divergence of positions and orientations of every marker in the system.

And we also plan to automate the environment setup with cmake.

For another perspective, we plan to develop a better model for the system, enabling tracking of acceleration and velocity for the objects of the system.
