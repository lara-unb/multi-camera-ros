# Experiment 1

## Overview

The purpose of this text is to explain the first experiment.
The first experiment aims to exploit the Multimaster packages, its capabilities, and its downsides.

---

## Running

First of all, we modified the script which Autonni uses to deploy the docker image.
We created the network with the following command:

```
docker network create --subnet=172.18.0.0/16 docker1
```

This docker1 network provides the static IPs to each container.
The Server has the IP: 172.18.0.10 and the first Rhino at 172.18.0.11.
The other Rhinos are in the next IPs.

Also, we modified the script which Autonni uses to deploy the docker image.
The diff below shows the differences:

```diff
 docker run -it \
-    --name="rhino_ros" \
+    --name="rhino_1" \
    --runtime=nvidia \
    --device /dev/dri \
    --device /dev/snd \
    -e DISPLAY \
    -e QT_GRAPHICSSYSTEM=native \
    -e XDG_RUNTIME_DIR=/tmp \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY=${DISPLAY} \
-    --net=host \
     --privileged \
+    --hostname=rhino_1 \
+    --net docker1 --ip 172.18.0.11 \
+    --add-host server_ros:172.18.0.10 \
+    --add-host rhino2_ros:172.18.0.12 \
     -v $XSOCK:$XSOCK:rw \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     -v /dev:/dev \
     -v /etc/machine-id:/etc/machine-id:ro \
     -v /var/run/dbus:/var/run/dbus:ro \
-    -v $(readlink -f ~/catkin_ws):/root/catkin_ws \
+    -v $(readlink -f ~/catkin_ws/src):/root/catkin_ws/src \
     -v $(readlink -f ~/clones):/root/clones \
     -v $(readlink -f ~/rhinos):/root/rhinos \
     -v $(readlink -f ~/bags):/root/bags \
```

The `--net` command sets which network the container should be connected.
The `--ip` sets the static IP.
We also add, using the `--add-host`, a list of known computers.
In this experiment, we added only the Server and the Rhino 2.
Also, we added only the catkin's source dir; then, each docker will remain with its build and devel dirs.

The table below shows the hostname and its IP.

| Hostname | IP Address |
| ------ | ----------- |
| Server | 172.18.0.10 |
| Rhino1 | 172.18.0.11 |
| Rhino2 | 172.18.0.12 |


### Server Side

The Server runs two communication nodes: the discovery node and the sync node.

### Robot Side

Each robot runs only the sync node, which allows its communication with only the Server.

---

## Results

When all system is running, only the Server sees the topics and services from other robots.
Only the machine in which the parameters are registered can view them.
We concluded that the discovery node is the one that forwards to the host machine the topics, and services from other robots.

The Server sees other robots even when it is the last one to startup.
And, when the Server fails, and it is brought back, it is capable of discovering other masters.
The same occurs when a robot goes down.
The Server sees the master not running anymore and removes it from its list, and when the same robot is alive again, the Server adds back.

There is no need to modify or add environment variables.
It will be helpful for each robot to have its namespace.
With this package, it seems that it is no need to add some procedure when starting or shutting down the robot.
