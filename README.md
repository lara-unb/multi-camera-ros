To prepare the environment, you have to individually install freenect ros lib manually. The overall process is quite simple so you just have to do the following steps:

Installing some dependencies:

$sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev

$ cd catkin_ws_camera/src
$ git clone https://github.com/OpenKinect/libfreenect
$ cd libfreenect
$ mkdir build
$ cd build
$ cmake -L ..
$ make

Now you have to manually install freenect stack, which is the libfreenect ROS driver. Again, follow the steps:

$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/freenect_stack.git

$ rosdep update
$ rosdep check --from-paths . --ignore-src --rosdistro noetic
$ rosdep install --from-paths . --ignore-src --rosdistro noetic -y

Now that freenect is properly installed you can use the kinect cameras. But the environment will need usb_cam library to suppor usb cameras too. Follow the steps:

$ sudo apt-get install ros-noetic-usb-cam

Now, to use the aruco markers ros package do the following:

$ sudo apt-get install ros-noetic-aruco-ros

With those libs installed, you can build the entire environment together from the workspace folder. To do that, run the following commands:

$ cd ~/catkin_ws/
$ catkin_make

