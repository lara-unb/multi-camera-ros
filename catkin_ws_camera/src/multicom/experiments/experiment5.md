# Experiment 5

## Overview

This experiment aims to test what happens to the server when the robot is turned off and when the robot is disconnected from the network in docker environment.

---

## Running

Two docker machines were prepared for this experiment to simulate one Rhinos and one server.
They share the same configuration as the machines in experiment 1.


## Running
- The first test is to shoot down the container using the command below.       
``
docker stop rhino_1
``

- The first test is to shoot down the container using the command below.       
``
docker network connect docker1 rhino_1
``



---

## Results

The results for both commands were the same. 

After 50 seconds, the server takes rhino_1 as offline. After 300 seconds the server removes rhino_1 from the list of active robots.

This data comes from the heartbeat_hz and remove_after parameters present in the master_discovery.launch file. These parameters can be changed.

When the rhino is disconnected and then connected, the rhino will only connect when its heartbeat happens.


