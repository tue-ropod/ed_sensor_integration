#ifndef ED_FEATURE_FUNCTIONS_H_
#define ED_FEATURE_FUNCTIONS_H_

#include "ros/ros.h" // used for the msg timestamps, need to be removed if markers are not communicated via ros anymore TODO
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <numeric>

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include <ed/plugin.h>

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "ed/termcolor.hpp"

#include <sensor_msgs/LaserScan.h>
#include <geolib/sensors/LaserRangeFinder.h>
#include <geolib/Shape.h>

#include <ed/entity.h>
#include <ed/world_model.h>

#include "featureProperties.h" // TODO chosen properly?
#include "generic_functions.h"

// TODO: make many of variables below configurable/tunable in ED model descriptions?
#define ARBITRARY_HEIGHT                0.03            // [m]
#define ARBITRARY_DEPTH                 ARBITRARY_HEIGHT
#define POINTS_TO_CHECK_CONFIDENCE      3               // [-]
#define EPSILON                         1e-4            // [m]
#define LASER_ACCURACY                  0.05            // [m]

#define N_POINTS_MARGIN_FOR_BEING_CONSECUTIVE 3    // [-] points must be consecutive for splitting if there is a proven gap. This is the margin for points being considered as consecutive

namespace tracking
{

enum FITTINGMETHOD {
    NONE = 1,
    LINE =   2,
    CIRCLE =   4,
    RECTANGLE =   8
};

struct laserSegments
{
  std::vector<geo::Vec2f>::iterator begin;
  std::vector<geo::Vec2f>::iterator end;
};

typedef std::vector<unsigned int> ScanSegment;

struct PointsInfo
{
    std::vector<geo::Vec2f> points;
    ScanSegment laserIDs;
};

float fitCircle ( std::vector<geo::Vec2f>& points, tracking::Circle* cirlce, const geo::Pose3D& pose );

float fitRectangle ( std::vector<geo::Vec2f>& points, tracking::Rectangle* rectangle, const geo::Pose3D& pose , unsigned int cornerIndex, unsigned int minPointsLine );

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *IDs, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end, float minDistCornerDetection, unsigned int minPointsLine );

bool findPossibleCorners ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *cornerIndices, float minDistCornerDetection, unsigned int minPointsLine );

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end ) ;//, unsigned int& index);

float fitLineLeastSq ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end );

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose, unsigned int minPointsLine );

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose,   unsigned int minPointsLine );

float fitObject ( std::vector<geo::Vec2f>& points, int FITTINGMETHOD, unsigned int* cornerIndex, tracking::Rectangle* rectangle, tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine);

bool determineCornerConfidence(const sensor_msgs::LaserScan::ConstPtr& scan, unsigned int element, bool checkElementLow);

geo::Vec2f avg ( std::vector<geo::Vec2f>& points, std::vector<geo::Vec2f>::const_iterator it_start, std::vector<geo::Vec2f>::const_iterator it_end );

geo::Vec2f projectPointOnLine(geo::Vec2f p1Line, geo::Vec2f p2Line, geo::Vec2f point2Project);

double getFittingError(const ed::Entity& e, const geo::LaserRangeFinder& lrf, const geo::Pose3D& rel_pose,
                       const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                       int& num_model_points);

bool splitSegmentsWhenGapDetected( std::vector< PointsInfo >& associatedPointsInfo, int min_gap_size_for_split,
                                   int min_segment_size_pixels, float dist_for_object_split, std::vector<float>& sensor_ranges, 
                                   const sensor_msgs::LaserScan::ConstPtr& scan);

std::vector<tracking::ScanSegment> determineSegments(std::vector<float> sensor_ranges, int maxGapSize, int minSegmentSize, 
                                                float segmentdepthThreshold, geo::LaserRangeFinder lrf_model, 
                                                double minClusterSize, double maxClusterSize, bool checkMinSizeCriteria);
}

#endif
