#ifndef ED_FEATURE_FUNCTIONS_H_
#define ED_FEATURE_FUNCTIONS_H_

#include "ros/ros.h" // used for the msg timestamps, need to be removed if markers are not communicated via ros anymore TODO
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <numeric>

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "ed/termcolor.hpp"

#include <sensor_msgs/LaserScan.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include "featureProperties.h"

// TODO: make many of variables below configurable/tunable in ED model descriptions?
#define ARBITRARY_HEIGHT                0.03            // [m]
#define ARBITRARY_DEPTH                 ARBITRARY_HEIGHT
#define POINTS_TO_CHECK_CONFIDENCE      3               // [-]
#define EPSILON                         1e-4            // [m]
#define LASER_ACCURACY                  0.05            // [m]

namespace tracking
{

enum FITTINGMETHOD {
    NONE = 1,
    LINE =   2,
    CIRCLE =   4,
    RECTANGLE =   8
};

class Point
{

public:
    float x, y;

    Point ( double x_in = 0.0, double y_in = 0.0 ) {
        x = x_in;
        y = y_in;
    }

};

struct greater
{
    template<class T>
    bool operator()(T const &a, T const &b) const { return a < b; }
};

struct laserSegments
{
  std::vector<geo::Vec2f>::iterator begin;
  std::vector<geo::Vec2f>::iterator end;
};

float fitCircle ( std::vector<geo::Vec2f>& points, tracking::Circle* cirlce, const geo::Pose3D& pose );

void determineIAV(std::vector<float> ranges, float* mean, float* standardDeviation, geo::LaserRangeFinder lrf_model, unsigned int firstElement, unsigned int finalElement );

int maxCrossCorrelation(std::vector<float>& measuredRanges, std::vector<unsigned int>::iterator measuredRangesStartElement,  std::vector<unsigned int>::iterator measuredRangesFinalElement,
                        std::vector<float>& modelledRanges, std::vector<unsigned int>::iterator modelledRangesStartElement,  std::vector<unsigned int>::iterator modelledRangesFinalElement);

float fitRectangle ( std::vector<geo::Vec2f>& points, tracking::Rectangle* rectangle, const geo::Pose3D& pose , unsigned int cornerIndex, unsigned int minPointsLine );

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *IDs, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end, float minDistCornerDetection, unsigned int minPointsLine );

bool findPossibleCorners ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *cornerIndices, float minDistCornerDetection, unsigned int minPointsLine );

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end ) ;//, unsigned int& index);

float fitLineLeastSq ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end );

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose, unsigned int minPointsLine );

template<typename T>
void wrap2Interval ( T* alpha, T lowerBound, T upperBound )
{
    T delta = upperBound - lowerBound;

    if ( *alpha < lowerBound )
    {
        while ( *alpha < lowerBound )
        {
            *alpha += delta;
        }
    }
    else if ( *alpha >= upperBound )
    {
        while ( *alpha >= upperBound )
        {
            *alpha -= delta;
        }
    }
}

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose,   unsigned int minPointsLine );

float fitObject ( std::vector<geo::Vec2f>& points, int FITTINGMETHOD, unsigned int* cornerIndex, tracking::Rectangle* rectangle, tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine);

bool determineCornerConfidence(const sensor_msgs::LaserScan::ConstPtr& scan, unsigned int element, bool checkElementLow);

geo::Vec2f avg ( std::vector<geo::Vec2f>& points, std::vector<geo::Vec2f>::const_iterator it_start, std::vector<geo::Vec2f>::const_iterator it_end );

geo::Vec2f projectPointOnLine(geo::Vec2f p1Line, geo::Vec2f p2Line, geo::Vec2f point2Project);

}


#endif
