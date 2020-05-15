#ifndef ED_SENSOR_INTEGRATION_VISUALIATION_FUNCTIONS_H_
#define ED_SENSOR_INTEGRATION_VISUALIATION_FUNCTIONS_H_

#include <visualization_msgs/MarkerArray.h>

// ROS
//#include <ros/subscriber.h>
//#include <ros/callback_queue.h>
#include "ros/ros.h"

#include "featureProperties.h" 

visualization_msgs::Marker getMarker ( tracking::FeatureProperties& featureProp, int ID); // TODO move to ed_rviz_plugins?

void pubPoints ( visualization_msgs::MarkerArray *markerArray, std::vector<geo::Vec2f> points, unsigned int *ID );

visualization_msgs::Marker visualizePoints( std::vector< geo::Vec2f> points, std::string frame_id, std::string nameSpace, ros::Time timeStamp, float height, int colorID);

#endif