ED Sensor integration [![Build Status](https://travis-ci.org/tue-robotics/ed_sensor_integration.svg?branch=master)](https://travis-ci.org/tue-robotics/ed_sensor_integration)
======

Plugins for integrating sensor data into ED, e.g. Lidar, RGBD

## Installation

Depends on:
- https://github.com/tue-robotics/ed.git

Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src
    git clone https://github.com/tue-robotics/ed_sensor_integration.git
    
    cd <your_catkin_workspace>/src
    wget -4 dlib.net/files/dlib-19.16.tar.bz2
    tar -xf dlib-19.16.tar.bz2 

And compile

    cd <your_catkin_workspace>:
    catkin_make
    
## Tutorial - Convex Hull Segmentation

All ED tutorials can be found in the ed_tutorials package: https://github.com/tue-robotics/ed_tutorials

## Tutorial - Geometric Tracking using a LRF

**General idea**: By fitting rectangles and circles and updating the properties (position, (rotational) velocity and the dimensions) over time with a Kalman-Filter, non-semantic tracking of objects is achieved

**Conceptual working principles**:
    1. Assuming the robot being localized, sensor-readings related to the static map are filtered out
    2. Readings not related to the environement are grouped into segments. A gap between sensor readings is an indication for the maximum size of a segment.
    3. Now, for each segment is determined which readings should be associated to entities being modelled before.  A closest distance criterion is used. If there is proof that a single entity actually belongs to multiple objects, segments are splitted. This is for example the case when an elevator door opens.
    4. For each segment associated to an entity, its geometric properties (position and radius for a circle, position, depth and width for a rectangle) are determined. Now, the properties of the model of the object are updated using a Kalman filter. For estimating the position and velocity, a constant-velocity model is used. Corrections are made for differences in dimensions between the model and the current measurement.
