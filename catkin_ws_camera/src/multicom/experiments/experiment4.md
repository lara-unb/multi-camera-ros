# Experiment 4

## Overview

This experiment intends to tests the `multimaster_fkie` pkg and explores its capabilities to be used with our multi-robot system.

---

## Running

Three docker machines were prepared for this experiment to simulate two Rhinos and one server.
They share the same configuration as the machines in experiment 1.

After the conception of the multi-robot system, we needed to know how the Multimaster package will work with the whole system.
Then, several tests were brought/gathered to test this interface.
They are:

- Test the synchronization of defined topics and services.
- Test if a robot can listen to other robot's messages, which they are also publishing without the need of sync node.
- Test the ROS Time synchronization between multiple ROS masters.

---

## Results

### Synchronization of selected topics

The roslaunches made for experiment 1 were changed to allow the parameter definition for all nodes.

The Multimaster package documentation talks about several parameters to setup the topics and services which should be synced.
Each parameter behavior was tested.
In each Rhino ran the rhino roslaunch, which starts only the discovery node, and a publisher sending a string message in the topics rhino1 and rhino2.
Before the tests were run, the server could see both topics.

#### ignore_host param

The `ignore_hosts` param configures the sync node to not sync any topic and service from a machine listed in the param.
When Rhino1 is listed in the `ignore_hosts` param, neither server or Rhino2 can listen to its topics.

#### sync_host param

Similar behavior can be seen with the `sync_hosts` param.
This param forces the sync node to sync only the topics and services from machines in its list.
When Rhino1 is listed in the param, all its topics could be seen by the server.
Nevertheless, the Rhino2 could see only its own topics.

#### ignore_topics and ignore_services params

Like `ignore_hosts`, these params remove from synchronization the topics or services listed.

#### sync_topics and sync_services params

Similar behavior from `sync_hosts`, sync only the topics or services listed.

> Notes:
>
> 1.   All `ignore_*` params are processed first.
> 2.   Even when an ignored host or topic is listed and removed from communication, the discovery node running in the server machine knows when the host is or not alive.


### Listen to other robots' messages

The purpose of this test is if the robots can see each other's messages when they are publishing on the same topic without the use of the sync node.
So, Rhino1 and Rhino2 were configured to publish on the same topic with different messages.
As expected, the server could see both messages when listening to this topic.
The result which we were looking for was when Rhino1 or Rhino2 subscribed to the topic. Rhino1 and Rhino2 can see both messages. 
However, when using rostopic info, the publisher registered in the ROS master of the machine appears as the only publisher.


### ROS Time sync

In the last test, the time synchronization between two ROS masters was examined.
A node was written to publish the ROS time on a topic.
This node was spawned using namespace by Rhino1 and Rhino2 in each one.
And in the server, a second script takes the two measurements and prints in the console.
As a result, we can see that the ROS Time is synchronized in the seconds level, but not at the nanoseconds level.
This can be explained as the two scripts were not started in the same instant. Also, one script should take a little longer to call the functions and publish the message.

This is an expected result as the docker instances run in the same machine sand they clocks will be the same as the host machine.
The [ROS Clock documentation](http://wiki.ros.org/Clock) talks about running ROS on multiple machines and how important it is to synchronize time between them.
And, recommend the [chrony](http://chrony.tuxfamily.org/) tool for time synchronization.
This tool syncs the system clock and can use one of the LAN machines as a reference clock allowing accuracy in tens of microseconds.
