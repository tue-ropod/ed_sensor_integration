#ifndef ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_

#include <ed/plugin.h>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

// TF
#include <tf/transform_listener.h>

#include <geolib/sensors/LaserRangeFinder.h>

// Messages
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <ed_sensor_integration/doorDetection.h>

// Properties
#include "ed/convex_hull.h"
#include "ed/convex_hull_calc.h"

#define MAX_CORRIDOR_WIDTH 3 // [m]
#define ADD_ASSOCIATION_DISTANCE 0.5 // [m]
#define MIN_ASSOCIATION_DISTANCE 0.3 // [m] TODO reduce
#define COORDINATE_OUTSIDE_MAP 1000.0 // [m]
#define CONFIDENCE_REGION_HIGH_COVARIANCE 0.5 // [m] If the distance is smaller than this distance, we are not certain about the size of an object
#define POINTS_TO_CHECK_CONFIDENCE 5 // [-]
#define EPSILON 1e-2 // [m]

// ----------------------------------------------------------------------------------------------------

class LaserPlugin : public ed::Plugin
{

public:

    LaserPlugin();

    virtual ~LaserPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    //

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;
    
    ros::Publisher door_pub_;
    
//     ros::Publisher ObjectMarkers_pub_;

    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

    tf::TransformListener* tf_listener_;

    geo::LaserRangeFinder lrf_model_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                const geo::Pose3D& sensor_pose, ed::UpdateRequest& req);



    // PARAMETERS

    int min_segment_size_pixels_;
    float world_association_distance_;
    float segment_depth_threshold_;
    double min_cluster_size_;
    double max_cluster_size_;
    bool fit_entities_;
    bool check_door_status_;

    int max_gap_size_;
    float entity_timeout_;
    std::map<ed::UUID,geo::Pose3D> pose_cache;
    
    // 'Feature' property key
    ed::PropertyKey<ed::tracking::FeatureProperties> featureProperties_; // TODO double defined now for publishing-purposes in rviz

};


#endif
