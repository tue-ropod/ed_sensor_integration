#ifndef ED_FEATURE_FUNCTIONS_H_
#define ED_FEATURE_FUNCTIONS_H_

#include "ros/ros.h" // used for the msg timestamps, need to be removed if markers are not communicated via ros anymore TODO
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <numeric>
#include "tf/transform_datatypes.h"

#include <vector>
#include <algorithm>

// #include <eigen3/Eigen/Dense>
#include <Eigen/Dense>

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "ed/termcolor.hpp"

#include <sensor_msgs/LaserScan.h>
#include <geolib/sensors/LaserRangeFinder.h>


// TODO: make many of variables below configurable/tunable in ED model descriptions?
#define TIMEOUT_TIME                    0.5             // [s]
//#define MAX_LINE_ERROR                  0.05            // [m]  
// #define MIN_POINTS_LINEFIT              5               // [-]
#define ARBITRARY_HEIGHT                0.03            // [m]
#define ARBITRARY_DEPTH                 ARBITRARY_HEIGHT
#define MARGIN_RECTANGLE_INTERCHANGE    30*M_PI/180     // [rad]
#define POINTS_TO_CHECK_CONFIDENCE      3               // [-]
#define EPSILON                         1e-4            // [m]
#define LASER_ACCURACY                  0.05            // [m]
#define MIN_PROB_OBJECT			0.05		// [-]


namespace ed
{


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

class Circle
{
    float x_, y_, z_, roll_, pitch_, yaw_, xVel_, yVel_, radius_; //xAccel_;//, yAccel_, radius_; // x, y, z-positions, roll, pitch, yaw and radius of circle
    Eigen::MatrixXf P_, Pdim_; // estimated covariance for state = [x, y, xVel, yVel] ^T and the radius
    
  public:
    Circle();
    
    void setProperties ( float x, float y, float z, float roll, float pitch, float yaw, float radius );
    
    float get_x()                                { return x_; } ;
    float get_y()                                { return y_; } ;
    float get_z()                                { return z_; } ;
    float get_roll()                             { return roll_; } ;
    float get_pitch()                            { return pitch_; } ;
    float get_yaw()                              { return yaw_; } ;
    float get_xVel()                             { return xVel_; } ;
    float get_yVel()                             { return yVel_; } ;
  // float get_xAccel()                           { return xAccel_; } ;
  //  float get_yAccel( )                          { return yAccel_; } ;
    float get_radius()                           { return radius_; } ;
    Eigen::MatrixXf get_P()                      { return P_; } ;
    Eigen::MatrixXf get_Pdim()                   { return Pdim_; } ;
    
    void set_x          ( float x )              { x_     = x; } ;
    void set_y          ( float y )              { y_     = y; } ;
    void set_z          ( float z )              { z_     = z; } ;
    void set_roll       ( float roll )           { roll_  = roll; } ;
    void set_pitch      ( float pitch )          { pitch_ = pitch; } ;
    void set_yaw        ( float yaw )            { yaw_   = yaw; } ;
    void set_xVel       ( float xVel )           { xVel_  = xVel; } ;
    void set_yVel       ( float yVel )           { yVel_  = yVel; } ;
//    void set_xAccel     ( float xAccel )         { xAccel_  = xAccel; } ;
//    void set_yAccel     ( float yAccel )          { yAccel_  = yAccel; } ;
    void set_radius     ( float radius )         { radius_ = radius; } ;
    void set_P          ( Eigen::MatrixXf P )    { P_ = P; } ;
    void set_Pdim       ( Eigen::MatrixXf Pdim ) { Pdim_ = Pdim; } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID );
    
    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color );
    
    void setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID );
    
    geo::Pose3D getPose() {geo::Pose3D pose(x_, y_, z_, roll_, pitch_,yaw_); return pose; };
    
    std::vector< geo::Vec2f > convexHullPoints(unsigned int nPoints);

    float predictX( float dt );
    
    float predictY( float dt );
    
    void predictPos( float* predictedX, float* predictedY, float dt );
    
    void predictAndUpdatePos( float dt );
    
    void printProperties();
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

float fitCircle ( std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, const geo::Pose3D& pose );

class Rectangle
{
        float x_, y_, z_, w_, d_, h_, roll_, pitch_, yaw_, xVel_, yVel_, yawVel_; // x, y of center, width, height and rotation of rectangle
        Eigen::MatrixXf P_, Pdim_;
  public:
    Rectangle();
    
    void setValues ( float x, float y, float z, float w, float d, float h, float roll, float pitch, float yaw );
    
    float get_x()                       { return x_; } ;
    float get_y()                       { return y_; } ;
    float get_z()                       { return z_; } ;
    float get_w()                       { return w_; } ;
    float get_d()                       { return d_; } ;
    float get_h()                       { return h_; } ;
    float get_roll()                    { return roll_; } ;
    float get_pitch()                   { return pitch_; } ;
    float get_yaw()                     { return yaw_; } ;
    float get_xVel()                    { return xVel_; } ;
    float get_yVel()                    { return yVel_; } ;
    float get_yawVel()                  { return yawVel_; } ;
    Eigen::MatrixXf get_P()             { return P_; } ;
    Eigen::MatrixXf get_Pdim()          { return Pdim_; } ;
    
    geo::Pose3D getPose() {geo::Pose3D pose(x_, y_, z_, roll_, pitch_,yaw_); return pose; };
    
    void set_x          ( float x )             { x_     = x; } ;
    void set_y          ( float y )             { y_     = y; } ;
    void set_z          ( float z )             { z_     = z; } ;
    void set_w          ( float w )             { w_     = w; } ;
    void set_d          ( float d )             { d_     = d; } ;
    void set_h          ( float h )             { h_     = h; } ;
    void set_roll       ( float roll )          { roll_  = roll; } ;
    void set_pitch      ( float pitch )         { pitch_ = pitch; } ;
    void set_yaw        ( float yaw )           { yaw_   = yaw; } ;
    void set_xVel       ( float xVel )          { xVel_  = xVel; } ;
    void set_yVel       ( float yVel )          { yVel_  = yVel; } ;
    void set_yawVel     ( float yawVel )        { yawVel_  = yawVel; } ;
    void set_P          ( Eigen::MatrixXf P )   { P_     = P; } ;
    void set_Pdim       ( Eigen::MatrixXf Pdim ){ Pdim_     = Pdim; } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID );
    
    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color);
    
    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color, std::string ns );
    
    void setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID );
    
    void setRotationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID );
    
    std::vector<geo::Vec2f> determineCorners( float associationDistance);
    
    std::vector<geo::Vec2f> determineCenterpointsOfEdges ( );
    
//     std::vector<geo::Vec2f> determinePointsOfSquare ( float associationDistance, float rotation );
    
    float predictX( float dt );
    
    float predictY( float dt );
    
    float predictYaw( float dt );
    
    void predictPos( float* predictedX, float* predictedY, float* predictedYaw, float dt );
    
    void predictAndUpdatePos( float dt );
    
    bool switchDimensions( float measuredYaw);
    
    void interchangeRectangleFeatures();
    
    Eigen::VectorXf setState( float posX, float posY, float posYaw, float xVel, float yVel, float yawVel, float width, float depth );

    void printProperties();
};

template <typename T> 
int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> 
void unwrap (T *angleMeasured, T angleReference, T increment)
{
        // Rectangle is symmetric over pi-radians, so unwrap to pi
        T diff = angleReference - *angleMeasured;
        
        int d = diff / (increment);
        *angleMeasured += d*increment;
        
        T r = angleReference - *angleMeasured;
        
        if( fabs(r) > (0.5*increment) )
        {
                *angleMeasured += sgn(r)*increment;
        }
}

void determineIAV(std::vector<float> ranges, float* mean, float* standardDeviation, geo::LaserRangeFinder lrf_model, unsigned int firstElement, unsigned int finalElement );

int maxCrossCorrelation(std::vector<float>& measuredRanges, std::vector<unsigned int>::iterator measuredRangesStartElement,  std::vector<unsigned int>::iterator measuredRangesFinalElement,
                        std::vector<float>& modelledRanges, std::vector<unsigned int>::iterator modelledRangesStartElement,  std::vector<unsigned int>::iterator modelledRangesFinalElement);

float fitRectangle ( std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, const geo::Pose3D& pose , unsigned int cornerIndex, unsigned int minPointsLine );

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *IDs, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end, float minDistCornerDetection, unsigned int minPointsLine );

bool findPossibleCorners ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *cornerIndices, float minDistCornerDetection, unsigned int minPointsLine );

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end ) ;//, unsigned int& index);

float fitLineLeastSq ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end );

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, ed::tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose, unsigned int minPointsLine );

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

float fitObject ( std::vector<geo::Vec2f>& points, int FITTINGMETHOD, unsigned int* cornerIndex, ed::tracking::Rectangle* rectangle, ed::tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine);

bool determineCornerConfidence(const sensor_msgs::LaserScan::ConstPtr& scan, unsigned int element, bool checkElementLow);

// bool determineSegmentConfidence ( const sensor_msgs::LaserScan::ConstPtr& scan, unsigned int elementLow, unsigned int elementHigh );

geo::Vec2f avg ( std::vector<geo::Vec2f>& points, std::vector<geo::Vec2f>::const_iterator it_start, std::vector<geo::Vec2f>::const_iterator it_end );

geo::Vec2f projectPointOnLine(geo::Vec2f p1Line, geo::Vec2f p2Line, geo::Vec2f point2Project);

Eigen::MatrixXf kalmanUpdate(Eigen::MatrixXf F, Eigen::MatrixXf H, Eigen::MatrixXf *P, Eigen::MatrixXf x_k_1_k_1, Eigen::MatrixXf z_k, Eigen::MatrixXf Q, Eigen::MatrixXf R);

// Probabilities
class FeatureProbabilities
{
  public:
    std::shared_ptr<pbl::PMF> pmf_;

FeatureProbabilities()
{
        pmf_ = std::make_shared<pbl::PMF>();
//    FeatureProbabilities ( void ) { // Initialize with 50/50 probabilities
        pmf_->setDomainSize ( 2 );
        pmf_->setProbability ( "Rectangle", 0.5 );
        pmf_->setProbability ( "Circle", 0.5 );
    };

    void setProbabilities ( float pRectangle_in, float pCircle_in ) {
        pmf_->setProbability ( "Rectangle", pRectangle_in );
        pmf_->setProbability ( "Circle", pCircle_in );
    };

    float get_pRectangle() const {
            double out = pmf_->getProbability ( "Rectangle" );
        return ( float ) out;
    } ;

    float get_pCircle() const {
//            std::cout << "Feature prob: ptr = " << this << "\t";
            double out = pmf_->getProbability ( "Circle" );
        return ( float ) out;
    } ;

    int getDomainSize (){
            return pmf_->getDomainSize();
            }
    bool setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleDiameter, float typicalCorridorWidth );

    void update ( float pRectangle_measured, float pCircle_measured );
    
    void update ( FeatureProbabilities& featureProbabilities_in );
    
};

//FeatureProbabilities::FeatureProbabilities (  ) { // Initialize with 50/50 probabilities
//        pmf_.setDomainSize ( 2 );
//        pmf_.setProbability ( "Rectangle", 0.5 );
//        pmf_.setProbability ( "Circle", 0.5 );
//    };

class FeatureProperties
{
  public:
    FeatureProbabilities featureProbabilities_; // Probabilities of the features. Is there a need to reset these when there is a switch? Or only when the probability of a feature was low?

    Circle circle_;

    Rectangle rectangle_;
    
    int nMeasurements_;
    
    FeatureProperties ( ) { // Initialize with 50/50 probabilities unless otherwise indicated
      featureProbabilities_.setProbabilities ( 0.5, 0.5 );
      nMeasurements_ = 0;
   };
   
   ~FeatureProperties ( ) { // Initialize with 50/50 probabilities unless otherwise indicated
    //  delete featureProbabilities_;
   };

    FeatureProperties ( const FeatureProperties* other ) {  
//             std::cout << "other = " << other << std::endl;
       featureProbabilities_ = other->featureProbabilities_;
       circle_ = other->circle_;
       rectangle_ = other->rectangle_;
       nMeasurements_ = other->nMeasurements_;
    };

    FeatureProbabilities getFeatureProbabilities() const {
        return featureProbabilities_;
    };

    void setFeatureProbabilities ( float pRectangle_in, float pCircle_in ) {
        featureProbabilities_.setProbabilities ( pRectangle_in, pCircle_in );
    };

    void setFeatureProbabilities ( FeatureProbabilities featureProbabilities_in ) {
        featureProbabilities_ = featureProbabilities_in;
    };

    void updateProbabilities ( FeatureProbabilities featureProbabilities_in ) {
        featureProbabilities_.update ( featureProbabilities_in );
    };

    void setCircle ( Circle circle_in ) {
        circle_ = circle_in;
    };

    Circle getCircle() const {
        return circle_;
    };

    Rectangle getRectangle() const {
        return rectangle_;
    };

    void setRectangle ( Rectangle rectangle_in ) {
        rectangle_ = rectangle_in;
    };
    
    int getNMeasurements() const{
            return nMeasurements_;
    };
    
    void setNMeasurements( int nMeasurements ){
            nMeasurements_ = nMeasurements;
    };
    
    void updateCircleSize(float Q_k, float R_k, float z_k); // z = observation // TODO improve! -> Determine proper covariances

    void updateRectangleSize(Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::VectorXf z_k); // TODO improve! h-> Determine proper covariances
    
    void updatePosition();
    
    void updateCircleFeatures(Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::MatrixXf z_k, float dt);
    
    void updateRectangleFeatures(Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::VectorXf z_k, float dt, const geo::Pose3D& sensor_pose);
    
    void correctForDimensions( float deltaWidth, float deltaDepth, float* xMeasured, float* yMeasured, float measuredPosX, float measuredPosY, float modelledPosX, float modelledPosY,  float dt );
    
    void correctPosForDimDiff(float deltaWidth, float deltaDepth, float *deltaX, float *deltaY, float dt);
    
    void printProperties();
};


}

}


#endif
