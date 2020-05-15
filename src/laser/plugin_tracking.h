#ifndef ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_

#include <ed/plugin.h>
#include "feature_functions.h"
#include "featureProperties_info.h"

#include "plugin_support_functions.h"
#include "visualization_functions.h"
#include "generic_functions.h"

// Tracking
#include "problib/conversions.h"
#include "wiredData.h"

// Generic
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <algorithm>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

// TF
#include <tf/transform_listener.h>

// Messages
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <ed_sensor_integration/doorDetection.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define ASSOCIATION_DISTANCE 0.5                        // [m] For the entities which already exist in the WM, determine the relevant properties in order to determine which entities _might_ associate to which clusters Association distance 
#define MIN_ASSOCIATION_DISTANCE 0.3                    // [m] Maximum distance to reassociate consecutive segments
#define MIN_ASSOCIATION_DISTANCE_SEGMENTS 0.1           // [m] 
#define ANGLE_MARGIN_FITTING 25*M_PI/180                // [rad]

// For corrections based on reading being associated to the environment
#define MAX_DEVIATION_ANGLE_CORRECTION 20*M_PI/180      // [rad]
#define MIN_DISTANCE_CORNER_DETECTION   0.10            // [m]
#define MIN_DISTANCE_CORNER_DETECTION_LARGE   2*MIN_DISTANCE_CORNER_DETECTION       // [m]
#define MAX_DISTANCE_POS_CORRECTION2 std::pow(0.3, 2.0) // [m]

#define DEFAULT_MEASUREMENT_UNCERTAINTY 0.1 //[-] TODO TEMP, in order to check which dimensions are measured.

#define DELAY_AFTER_INIT ros::Duration(1.0)             // [s]

#define INF 10000

#define DEBUG false // TEMP
#define DEBUG_SF false // TEMP

// ----------------------------------------------------------------------------------------------------

class LaserPluginTracking : public ed::Plugin
{

public:

    LaserPluginTracking();

    virtual ~LaserPluginTracking();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:
    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;
    
    ros::Publisher door_pub_;
    
    ros::Publisher ObjectMarkers_pub_; // ############################## TEMP ############################
    
    ros::Publisher pose_updated_pub_; // ############################## TEMP ############################
   
    ros::Publisher points_modelled_pub_; // ############################## TEMP ############################
   
    ros::Publisher points_measured_pub_; // ############################## TEMP ############################

    ros::Publisher associatedPoints_pub_; // ############################## TEMP ############################

    ros::Subscriber amclPose_sub_;
    
    ros::Subscriber initializedPose_sub_;

    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;
    
    std::queue<geometry_msgs::PoseWithCovarianceStamped::ConstPtr> pose_buffer_;
    
    std::queue<geometry_msgs::PoseWithCovarianceStamped::ConstPtr> pose_buffer_init_;

    tf::TransformListener* tf_listener_;

    geo::LaserRangeFinder lrf_model_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    void PoseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    
    void PoseWithCovarianceStampedInitCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan, geo::Pose3D& sensor_pose, ed::UpdateRequest& req);

    // PARAMETERS
    int min_segment_size_pixels_;
    float world_association_distance_;
    float segment_depth_threshold_;
    double min_cluster_size_;
    double max_cluster_size_;
    bool fit_entities_;
//     bool check_door_status_;
    float nominal_corridor_width_;
    bool correctXYpos_;

    int max_gap_size_;
    int min_gap_size_for_split_; 
    float dist_for_object_split_;
    std::map<ed::UUID,geo::Pose3D> pose_cache;

    // 'Feature' property key
    ed::PropertyKey<tracking::FeatureProperties> featureProperties_; 
};

#endif
