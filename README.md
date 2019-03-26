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

  1.  Assuming the robot being localized, sensor-readings related to the static map are filtered out
  2. Readings not related to the environement are grouped into segments. A gap between sensor readings is an indication for the maximum size of a segment.
  3. Now, for each segment is determined which readings should be associated to entities being modelled before.  A closest distance criterion is used. If there is proof that a single entity actually belongs to multiple objects, segments are splitted. This is for example the case when an elevator door opens.
  4. For each segment associated to an entity, its geometric properties (position and radius for a circle, position, depth and width for a rectangle) are determined. Now, the properties of the model of the object are updated using a Kalman filter. For estimating the position and velocity, a constant-velocity model is used. Corrections are made for differences in dimensions between the model and the current measurement.
  5. In order to discriminate between a circle and a rectangle, a probability is determined using the fitting error. As fitting a circle with an infinite radius approximates a straight line, the probability is scaled with a typical corridor width. Over time, this probability is updated using a probability mass function. In order to allow for recovery, the maximum probabilty of an entity being a circle or rectangle is set to 0.9
  
  **Tunable Parameters**:
  Below, an example is given for your ED-configration file:
  <pre>
  plugins:
  - name: laser_integration
    lib: libed_laser_plugin_tracking.so
    frequency: 30
    parameters:
        laser_topic: /ropod/laser/scan
        min_segment_size_pixels: 6
        world_association_distance: 0.2
        segment_depth_threshold: 0.08
        min_cluster_size: 0.05
        max_cluster_size: 10.0
        max_gap_size: 1
        min_gap_size_for_split: 5
        nominal_corridor_width: 0.5
        correctRot: 1
        correctTrans: 0
        dist_for_object_split: 0.4
  - name: entity_clearer
    lib: libed_clearer_plugin.so
    enabled: 1
    parameters:
        entity_timeout: 3.0
</pre>

The entity-clearer removes objects which have not been seen for "entity_timeout"-seconds. With the enable-parameters you can turn it on or off.

The meaning of the parameters for the tracker is as follows:

  * _laser_topic_: ros-topic on on which the relevant LRF-data are publised
  * _min_segment_size_pixels_: Minimum number of pixels required to consider a segment
  * _world_association_distance_: if the distance [m] between the sensor reading and the expected sensor reading is less than this parameters, than the reading is assumed to be associated to the static environment.
  * _segment_depth_threshold_: if the distance [m] between 2 sensor readings is above this threshold, than it is considered as a gap.
  * _min_cluster_size_: minimum size [m] of a segment
  * _max_cluster_size_: maximum size [m] of a segment
  * _max_gap_size_: Size [number of pixels] of a gap to split a segment
  * _nominal_corridor_width_: typical corridor with in order to scale the probability of an entity being a circle or rectangle.
  * _correctRot_: For reading being associated to the environment, straight lines are used to correct the orientation during the association phase. This prevents false positives.
  * _correctTrans_: Idem, but for translation based on corners. Does not work robustly!
  * _min_gap_size_for_split_: When points have been associated and there are at least "min_gap_size_for_split"-pixels for which there is a reading which is significantly larger, the segment is splitted in 2 segments.
  * _dist_for_object_split_: When points have been associated and there are at least dist_for_object_split-pixels[m] for which there is a reading which is significantly larger, the segment is splitted in 2 segments.
