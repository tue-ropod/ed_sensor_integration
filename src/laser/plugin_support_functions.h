#ifndef ED_SENSOR_INTEGRATION_PLUGIN_SUPPORT_FUNCTIONS_H_
#define ED_SENSOR_INTEGRATION_PLUGIN_SUPPORT_FUNCTIONS_H_

#include <geolib/sensors/LaserRangeFinder.h>
#include "feature_functions.h"
#include "visualization_functions.h" 

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>
#include <geolib/datatypes.h>

// Tracking
#include "wire_msgs/WorldEvidence.h"
//#include "wire_msgs/ObjectEvidence.h"

bool sortBySegmentSize(const tracking::ScanSegment &lhs, const tracking::ScanSegment &rhs);

//ros::Time tLatestPoseInit;
//tLatestPoseInit =  0.0 ;

struct EntityUpdate
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

struct EntityProperty
{
    geo::Vec2f entity_min;
    geo::Vec2f entity_max;
};

struct measuredPropertyInfo
{
       tracking::FeatureProperties featureProperty;
       bool propertiesDescribed;
       bool confidenceCircle;
       bool confidenceRectangleWidth;// confidence of entire side
       bool confidenceRectangleWidthLow; // confidence about cornerpoint
       bool confidenceRectangleWidthHigh; // confidence about cornerpoint
       bool confidenceRectangleDepth; 
       bool confidenceRectangleDepthLow;
       bool confidenceRectangleDepthHigh;
       tracking::FITTINGMETHOD methodRectangle; //
       float fittingErrorCircle;
       float fittingErrorRectangle;
       std::vector<geo::Vec2f> measuredCorners;
};

void renderWorld(const geo::Pose3D& sensor_pose, std::vector<double>& model_ranges, const ed::WorldModel& world, geo::LaserRangeFinder lrf_model);

bool pointIsPresent(double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges);

bool pointIsPresent(const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges);

geo::Pose3D getPoseFromCache(const ed::Entity& e, std::map<ed::UUID,geo::Pose3D>& pose_cache);

geo::Pose3D fitEntity(const ed::Entity& e, const geo::Pose3D& sensor_pose, const geo::LaserRangeFinder& lrf,
                      const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                      float x_window, float x_step, float y_window, float y_step, float yaw_min, float yaw_plus, 
                      float yaw_step, std::map<ed::UUID,geo::Pose3D>& pose_cache);

void addEvidenceWIRE(wire_msgs::WorldEvidence& world_evidence, tracking::FeatureProperties measuredProperty );

double getFittingError ( const ed::Entity& e, const geo::LaserRangeFinder& lrf, const geo::Pose3D& rel_pose,
                         const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                         int& num_model_points );

unsigned int determineClosestObject(geo::Vec2f point, float *shortestDistance, float timeStampScan,
                        std::vector< int > possibleSegmentEntityAssociations, 
                        std::vector<ed::WorldModel::const_iterator> it_laserEntities, 
                        ed::PropertyKey<tracking::FeatureProperties> featurePropertiesKey);

#endif