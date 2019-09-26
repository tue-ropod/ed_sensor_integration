#include "plugin_tracking.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>
#include <numeric>
#include <cmath>
#include <iterator>
#include <boost/graph/graph_concepts.hpp>

// 
namespace
{

typedef std::vector<unsigned int> ScanSegment;

bool sortBySegmentSize(const ScanSegment &lhs, const ScanSegment &rhs) { return lhs.size() > rhs.size(); }

ros::Time tLatestPoseInit( 0.0 );

struct PointsInfo
{
    std::vector<geo::Vec2f> points;
    ScanSegment laserIDs;
};

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
       ed::tracking::FeatureProperties featureProperty;
       bool propertiesDescribed;
       bool confidenceCircle;
       bool confidenceRectangleWidth;// confidence of entire side
       bool confidenceRectangleWidthLow; // confidence about cornerpoint
       bool confidenceRectangleWidthHigh; // confidence about cornerpoint
       bool confidenceRectangleDepth; 
       bool confidenceRectangleDepthLow;
       bool confidenceRectangleDepthHigh;
       ed::tracking::FITTINGMETHOD methodRectangle; //
       float fittingErrorCircle;
       float fittingErrorRectangle;
       std::vector<geo::Vec2f> measuredCorners;
};

visualization_msgs::Marker getMarker ( ed::tracking::FeatureProperties& featureProp, int ID) // TODO move to ed_rviz_plugins?
// ############################## TEMP ############################
{
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;

        color.r = 0;
        color.g = 255;
        color.b = 0;
        color.a = ( float ) 0.5;

        if ( featureProp.getFeatureProbabilities().get_pCircle() > featureProp.getFeatureProbabilities().get_pRectangle() )
        {
            ed::tracking::Circle circle = featureProp.getCircle();
            circle.setMarker ( marker , ID, color );
        }
        else
        {
            ed::tracking::Rectangle rectangle = featureProp.getRectangle();
            rectangle.setMarker ( marker , ID, color );
        }
        
    return marker;
}

template <typename T>
void append(std::vector<T>& a, const std::vector<T>& b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
}

template <typename T>
void append(std::vector<T>& a, const std::vector<T>& b, int bStart, int bEnd)
{
    a.reserve(a.size() + bEnd - bStart );
    a.insert(a.end(), b.begin() + bStart, b.begin() + bEnd);
}

float COLORS[27][3] = { { 1.0, 0.0, 0.0},// ############################## TEMP ############################
                        { 0.0, 1.0, 0.0},
                        { 0.0, 0.0, 1.0},
                        { 1.0, 0.0, 1.0},
                        { 0.0, 1.0, 1.0},
                        { 1.0, 1.0, 1.0},
                        { 1.0, 0.0, 0.0},
                        { 0.0, 0.0, 0.0}
                      };


void pubPoints ( visualization_msgs::MarkerArray *markerArray, std::vector<geo::Vec2f> points, unsigned int *ID )
// ############################## TEMP ############################
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.id = ( *ID ) ++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.3;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( 0.0, 0.0, 0.0 );
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    int i_color = *ID % 8;
    marker.color.r = COLORS[i_color][0];
    marker.color.g = COLORS[i_color][1];
    marker.color.b = COLORS[i_color][2];
    marker.color.a = ( float ) 1.0;

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );

    for ( unsigned int ii = 0; ii < points.size(); ii++ )
    {

        geometry_msgs::Point p;
        p.x = points[ii].x;
        p.y = points[ii].y;
        p.z = 0.0;
        marker.points.push_back ( p );
    }
    
    markerArray->markers.push_back ( marker );
}

void renderWorld(const geo::Pose3D& sensor_pose, std::vector<double>& model_ranges, const ed::WorldModel& world, geo::LaserRangeFinder lrf_model)
{
     geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        
        if (e->shape() && e->has_pose() && !(e->hasType("left_door") || e->hasType("door_left") || e->hasType("right_door") || e->hasType("door_right" ) || e->hasFlag("non-localizable")))
        {
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            geo::Mesh testMesh = e->shape()->getMesh();
            geo::Pose3D poseTest = sensor_pose_inv ;
            geo::Pose3D poseTest2 = e->pose();

            opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());

            geo::LaserRangeFinder::RenderResult res(model_ranges);
            lrf_model.render(opt, res);
        }        
    }       
}
 
// ----------------------------------------------------------------------------------------------------

double getFittingError(const ed::Entity& e, const geo::LaserRangeFinder& lrf, const geo::Pose3D& rel_pose,
                       const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                       int& num_model_points)
{
    std::vector<double> test_model_ranges = model_ranges;

    // Render the entity with the given relative pose
    geo::LaserRangeFinder::RenderOptions opt;
    opt.setMesh(e.shape()->getMesh(), rel_pose);

    geo::LaserRangeFinder::RenderResult res(test_model_ranges);
    lrf.render(opt, res);

    int n = 0;
    num_model_points = 0;
    double total_error = 0;
    for(unsigned int i = 0; i < test_model_ranges.size(); ++i)
    {
        double ds = sensor_ranges[i];
        double dm = test_model_ranges[i];

        if (ds <= 0)
            continue;

        ++n;

        if (dm <= 0)
        {
            total_error += 0.1;
            continue;
        }

        double diff = std::abs(ds - dm);
        if (diff < 0.1)
            total_error += diff;
        else
        {
            if (ds > dm)
                total_error += 1;
            else
                total_error += 0.1;
        }

        ++num_model_points;
    }

    return total_error / (n+1); // to be sure to never divide by zero.
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D getPoseFromCache(const ed::Entity& e, std::map<ed::UUID,geo::Pose3D>& pose_cache)
{
    const ed::UUID ID = e.id();
    geo::Pose3D old_pose = e.pose();
    if (pose_cache.find(ID) == pose_cache.end())
    {
        pose_cache[ID] = old_pose;
    }
    else
    {
        old_pose = pose_cache[ID];
    }
    return old_pose;
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D fitEntity(const ed::Entity& e, const geo::Pose3D& sensor_pose, const geo::LaserRangeFinder& lrf,
                      const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                      float x_window, float x_step, float y_window, float y_step, float yaw_min, float yaw_plus, float yaw_step, std::map<ed::UUID,geo::Pose3D>& pose_cache)
{
    const geo::Pose3D& old_pose = getPoseFromCache(e, pose_cache);

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();


    double min_error = 1e6;
    geo::Pose3D best_pose = e.pose();


    for(float dyaw = yaw_min; dyaw <= yaw_plus; dyaw += yaw_step)
    {
        geo::Mat3 rot;
        rot.setRPY(0, 0, dyaw);
        geo::Pose3D test_pose = old_pose;
        test_pose.R = old_pose.R * rot;

        for(float dx = -x_window; dx <= x_window; dx += x_step)
        {
            test_pose.t.x = old_pose.t.x + dx;
            for(float dy = -y_window; dy <= y_window; dy += y_step)
            {
                test_pose.t.y = old_pose.t.y + dy;

                int num_model_points;
                double error = getFittingError(e, lrf, sensor_pose_inv * test_pose, sensor_ranges, model_ranges, num_model_points);

//                ROS_ERROR_STREAM("yaw = " << dyaw << ", error = " << error << ", minerror= " << min_error << ", num_model_points = " << num_model_points);

                if (error < min_error && num_model_points >= 3)
                {
                    best_pose = test_pose;
                    min_error = error;
                }
            }
        }
    }
    return best_pose;
}

void splitSegmentsWhenGapDetected( std::vector< PointsInfo >& associatedPointsInfo, int min_gap_size_for_split,int min_segment_size_pixels, float dist_for_object_split, 
                                   std::vector<float>& sensor_ranges, const sensor_msgs::LaserScan::ConstPtr& scan)
{
        for ( unsigned int iList = 0; iList < associatedPointsInfo.size(); iList++ )
        {           
            std::vector<unsigned int> IDs = associatedPointsInfo[iList].laserIDs;
      
            if( IDs.size() == 0 )
                    continue;
            
            for(unsigned int iIDs = 1; iIDs < IDs.size(); iIDs++)
            {
                    unsigned int idLow = iIDs - 1;
                    unsigned int gapSize = IDs[iIDs] - IDs[iIDs - 1];
                    
                    if( gapSize >= min_gap_size_for_split )
                    {
                        // check ranges in gap
                        // if there is a set of consecutive ranges (min_gap_size_for_split) which is significantly larger than ranges before gap and after gap, this implies that there is free space
                        // instead of an object which blocks the view on the object. The entities are splitted and treated as 2 separate ones.
                            
                            unsigned int nLowElements, nHighElements;
                            if (idLow < min_gap_size_for_split)
                            {
                                    if ((2 * idLow) > IDs.size())
                                        nLowElements = IDs.size() - idLow;
                                    else
                                        nLowElements = idLow;
                            }
                            else
                            {
                                    if ((idLow + min_gap_size_for_split) > IDs.size())
                                        nLowElements = IDs.size() - idLow;
                                    else
                                        nLowElements = min_gap_size_for_split;
                            }                           
                            
                            if (iIDs > (IDs.size() - (unsigned int)  min_gap_size_for_split))
                            {
                                    nHighElements = IDs.size() - iIDs;
                            }
                            else
                            {
                                    nHighElements = min_gap_size_for_split;
                            }
                            
                            float avgRangeLow = 0.0, avgRangeHigh = 0.0;
                            for(unsigned int iAvgLow = 0; iAvgLow < nLowElements; iAvgLow++)
                            {
                                    float range = sensor_ranges[IDs[idLow + iAvgLow]];
                                    if (range == 0.0 ) // ranges were set to zero if associated to world
                                            range = scan->range_max;
                                    
                                    avgRangeLow += range;
                            }
                            avgRangeLow /= nLowElements;
                            
                            // TEMP FOR TEST
//                             bool check = iIDs > (IDs.size() - min_gap_size_for_split);
//                             std::cout << "Bla: " << IDs.size() - min_gap_size_for_split << std::endl;
//                             std::cout << "Bla2: " << IDs.size() - (unsigned int) min_gap_size_for_split << std::endl;
                        //    ROS_WARN("Potential Problem in ED tracking plugin: iIDs + iAvgHigh >= IDs.size(). iIDs = %u  iAvgHigh = %u IDs.size() = %lu  nHighElements = %u  min_gap_size_for_split = %u, check = %i", iIDs, iAvgHigh, IDs.size(), nHighElements, min_gap_size_for_split, check );
                            // 
                                            
                                            
                            
                            for(unsigned int iAvgHigh = 0; iAvgHigh < nHighElements; iAvgHigh++)
                            {
                                    // Some checks just in case
                                    if( iIDs + iAvgHigh >= IDs.size() )
                                    {
                                            bool check = iIDs > (IDs.size() - min_gap_size_for_split);
                                            std::cout << "Test1: " << IDs.size() - min_gap_size_for_split << std::endl;
                                            std::cout << "Test2: " << (unsigned int) IDs.size() - (unsigned int) min_gap_size_for_split << std::endl;
                                            ROS_WARN("Potential Problem in ED tracking plugin: iIDs + iAvgHigh >= IDs.size(). iIDs = %u  iAvgHigh = %u IDs.size() = %lu  nHighElements = %u  min_gap_size_for_split = %u, check = %i", iIDs, iAvgHigh, IDs.size(), nHighElements, min_gap_size_for_split, check );
                                            nHighElements = iAvgHigh - 1; // safety measure!
                                            continue;
                                            
                                    }
                                    
                                    // Potential Problem in ED tracking plugin: iIDs + iAvgHigh >= IDs.size(). iIDs = 1  iAvgHigh = 1 IDs.size() = 2  nHighElements = 5  min_gap_size_for_split = 5
                                    //[ WARN] [1559656004.524065310][/ed]: Potential Problem in ED tracking plugin: IDs[iIDs + iAvgHigh] >= sensor_ranges.size(). iIDs = 1 iAvgHigh = 1 IDs.size() = 2 IDs[iIDs + iAvgHigh] = 1111917452 nHighElements = 5 min_gap_size_for_split = 5 sensor_ranges.size() = 726

                                    if( IDs[iIDs + iAvgHigh] >= sensor_ranges.size() )
                                    {
                                            ROS_WARN("Potential Problem in ED tracking plugin: IDs[iIDs + iAvgHigh] >= sensor_ranges.size(). iIDs = %u iAvgHigh = %u IDs.size() = %lu IDs[iIDs + iAvgHigh] = %u nHighElements = %u min_gap_size_for_split = %u sensor_ranges.size() = %lu", iIDs, iAvgHigh, IDs.size(), IDs[iIDs + iAvgHigh], nHighElements, min_gap_size_for_split, sensor_ranges.size() );
                                    }
                                    
                                    
                                    float range = sensor_ranges[IDs[iIDs + iAvgHigh]];
                                    if (range == 0.0 ) // ranges were set to zero if associated to world
                                            range = scan->range_max;
                                    
                                    avgRangeHigh += range;
                            }
                            
                            if(nHighElements <= 0)
                            {
                                    avgRangeHigh = scan->range_max;
                            } else {
                                    avgRangeHigh /= nHighElements;
                            }
                            
                            std::cout << "avgRangeLow = " << avgRangeLow << " avgRangeHigh = " << avgRangeHigh << std::endl;

                            float maxReference = std::max(avgRangeLow, avgRangeHigh);
                            unsigned int nLargerRanges = 0;

                            for(unsigned int iGap = IDs[idLow]; iGap < IDs[iIDs]; iGap++ )
                            {       
/*                                    
                                    std::cout << "sensor_ranges[iGap] = " << sensor_ranges[iGap] << " maxReference = " << maxReference  << std::endl;
                                    if(sensor_ranges[iGap] == 0.0 )
                                    {
                                            std::cout << "Zero gap: i = " << iGap;
                                    }
                                    */

                                   if(  sensor_ranges[iGap] > maxReference ||  sensor_ranges[iGap] == 0.0 ) // as points associated to the world are set to 0
                                    {
                                            nLargerRanges++;
                                    }
                                    else
                                    {
                                            nLargerRanges = 0;
                                    }
                                    
//                                     std::cout << "nLargerRanges = " << nLargerRanges << std::endl;
                                    
                                    if(nLargerRanges >= min_gap_size_for_split)
                                    {
//                                             std::cout << "Going to split segment " << std::endl;
                                            
                                            PointsInfo splittedInfo;

                                            std::vector<geo::Vec2f> points = associatedPointsInfo[iList].points;
                                            std::copy( associatedPointsInfo[iList].laserIDs.begin() + iIDs, associatedPointsInfo[iList].laserIDs.end(), std::back_inserter(splittedInfo.laserIDs) );
                                            std::copy( associatedPointsInfo[iList].points.begin() + iIDs, associatedPointsInfo[iList].points.end(),  std::back_inserter(splittedInfo.points) );
                                            associatedPointsInfo.push_back( splittedInfo );
                                            associatedPointsInfo[iList].laserIDs.erase (associatedPointsInfo[iList].laserIDs.begin() + iIDs, associatedPointsInfo[iList].laserIDs.end() );
                                            associatedPointsInfo[iList].points.erase (associatedPointsInfo[iList].points.begin() + iIDs, associatedPointsInfo[iList].points.end() );
                                            goto endOfLoop;
                                    }
                            }
                    }
            }
            endOfLoop:;  
    }
    
    for ( unsigned int iList = 0; iList < associatedPointsInfo.size(); iList++ )
    {
            // As each point is associated to an object, it might happen that 2 objects close to each other can be seen as one. This results in large distances between consecutive points.
            // Here it is checked if this is the case. If so, objects are being splitted.
            
            ScanSegment IDs = associatedPointsInfo[iList].laserIDs;
            std::vector< geo::Vec2f > segmentPoints = associatedPointsInfo[iList].points;
            
            unsigned int nPointsForAvg;
            min_segment_size_pixels % 2 == 0 ? nPointsForAvg = min_segment_size_pixels / 2 : nPointsForAvg = (min_segment_size_pixels - 1) / 2;
            
            if( IDs.size() < 2*nPointsForAvg ) // 2 times, because we have to determine the average for the lower and upper part
            {
                    continue;
            }
            
            geo::Vec2f pointLowSum, pointHighSum;
            pointLowSum.x = 0.0;
            pointLowSum.y = 0.0;
            pointHighSum.x = 0.0;
            pointHighSum.y = 0.0;
            
            for(unsigned int iAvgLow = 0; iAvgLow < nPointsForAvg; iAvgLow++)
            {                    
                    pointLowSum += segmentPoints[iAvgLow];
            }
            
            geo::Vec2f avgPointLow = pointLowSum / nPointsForAvg;
             
            for(unsigned int iAvgHigh = nPointsForAvg; iAvgHigh < 2*nPointsForAvg; iAvgHigh++)
            {
                    pointHighSum += segmentPoints [iAvgHigh];
            }

            geo::Vec2f avgPointHigh = pointHighSum / nPointsForAvg;
            
            bool splitFound = false;
            
             if( std::fabs( IDs[(2*nPointsForAvg) - 1] - IDs[0]) <=  2*nPointsForAvg + N_POINTS_MARGIN_FOR_BEING_CONSECUTIVE &&
                 std::fabs( IDs[(2*nPointsForAvg) - 1] - IDs[0]) >=  2*nPointsForAvg - N_POINTS_MARGIN_FOR_BEING_CONSECUTIVE ) // points must be consecutive
             {                     
                if( avgPointHigh.dist( avgPointLow ) >  dist_for_object_split )
                {
                        PointsInfo splittedInfo;       
                        unsigned int position2Split = nPointsForAvg;//IDs[nPointsForAvg];
                        std::vector<geo::Vec2f> points = associatedPointsInfo[iList].points;             
                        std::copy( associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end(), std::back_inserter(splittedInfo.laserIDs) );
                        std::copy( associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end(),  std::back_inserter(splittedInfo.points) );
                        associatedPointsInfo.push_back( splittedInfo );
                        
                        associatedPointsInfo[iList].laserIDs.erase (associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end() );
                        associatedPointsInfo[iList].points.erase (associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end() );
                        splitFound = true;
                }
             }

            for(unsigned int iIDs = 0; iIDs < (IDs.size() - 2*nPointsForAvg - 1) && !splitFound && IDs.size() >= ( 2*nPointsForAvg + 1); iIDs++) // -1 because the first segment is already determined as above
            { 
                    pointLowSum -= segmentPoints[iIDs];
                    pointLowSum += segmentPoints[iIDs + nPointsForAvg];
                    
                    pointHighSum -= segmentPoints[iIDs + nPointsForAvg];
                    pointHighSum += segmentPoints[iIDs + 2*nPointsForAvg];

                    if( IDs[iIDs + 2*nPointsForAvg] - IDs[iIDs] ==  2*nPointsForAvg) // points must be consecutive
                    {
                            avgPointLow = pointLowSum / nPointsForAvg;
                            avgPointHigh = pointHighSum / nPointsForAvg;
                        
                            if( avgPointHigh.dist( avgPointLow ) >  dist_for_object_split )
                            {
                                    PointsInfo splittedInfo;       
                                    unsigned int position2Split = iIDs;//IDs[nPointsForAvg];
                                    std::vector<geo::Vec2f> points = associatedPointsInfo[iList].points;     
                
                                    for(unsigned int iPrint = 0; iPrint < associatedPointsInfo[iList].laserIDs.size(); iPrint++)
                                    {
                                            if(associatedPointsInfo[iList].laserIDs[iPrint]  > sensor_ranges.size())
                                            {
                                                    ROS_ERROR("BAD INFORMATION 0!!");
                                                    exit(0);
                                             }
                                     }
                                        
                                     std::copy( associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end(), std::back_inserter(splittedInfo.laserIDs) );
                                     std::copy( associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end(),  std::back_inserter(splittedInfo.points) );
                                     associatedPointsInfo.push_back( splittedInfo );

                                     associatedPointsInfo[iList].laserIDs.erase (associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end() );
                                     associatedPointsInfo[iList].points.erase (associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end() );
                                     splitFound = true;
                                       
                                     for(unsigned int iPrint = 0; iPrint < associatedPointsInfo[iList].laserIDs.size(); iPrint++)
                                     {       
                                             if(associatedPointsInfo[iList].laserIDs[iPrint]  > sensor_ranges.size())
                                             {
                                                     ROS_ERROR("BAD INFORMATION 1!!");
                                                     exit(0);
                                             }
                                     }   
                                        
                                     for(unsigned int iPrint = 0; iPrint <splittedInfo.laserIDs.size(); iPrint++)
                                     {       
                                             if(splittedInfo.laserIDs[iPrint]  > sensor_ranges.size())
                                             {
                                                     ROS_ERROR("BAD INFORMATION 2!!");
                                                     exit(0);
                                             }
                                      }
                             }
                    }
            }
    }
}

// ----------------------------------------------------------------------------------------------------

bool pointIsPresent(double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    int i_beam = lrf.getAngleUpperIndex(x_sensor, y_sensor);
    if (i_beam < 0 || i_beam >= sensor_ranges.size())
        return true; // or actually, we don't know

    float rs = sensor_ranges[i_beam];
    return rs == 0 || geo::Vec2(x_sensor, y_sensor).length() > rs - 0.1;
}

// ----------------------------------------------------------------------------------------------------

bool pointIsPresent(const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    return pointIsPresent(p_sensor.x, p_sensor.y, lrf, sensor_ranges);
}

}

// ----------------------------------------------------------------------------------------------------

// Strongly inspired by https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
template<typename T>
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment( T& p, T& q, T& r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}
 
 template<typename T>
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation( T& p, T& q, T& r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
 template<typename T>
// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect( T& p1, T& q1, T& p2, T& q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}
 
 template<typename T>
// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(std::vector<T> Points, T& p)
{
     int n = Points.size();   
        
    // There must be at least 3 vertices in polygon[]
    if (n < 3)  return false;
    
    // Create a point for line segment from p to infinite
    T extreme;
    extreme.x = INF;
    extreme.y = p.y;
 
    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
 
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(Points[i], Points[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(Points[i], p, Points[next]) == 0)
               return onSegment(Points[i], p, Points[next]);
            count++;
        }
        i = next;
    } while (i != 0);
    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}

 std::vector<ScanSegment> determineSegments(std::vector<float> sensor_ranges, int maxGapSize, int minSegmentSize, float segmentdepthThreshold, geo::LaserRangeFinder lrf_model, double minClusterSize, double maxClusterSize)
{
        unsigned int num_beams = sensor_ranges.size();
        std::vector<ScanSegment> segments;
 
        // Find first valid value
        ScanSegment current_segment;
        for ( unsigned int i = 0; i < num_beams - 1; ++i )
        {

                if ( sensor_ranges[i] > 0 )
                {
                        current_segment.push_back(i);
                        break;
                }
        }
    
        if ( current_segment.empty() )
        {
                return segments;
        }

        int gap_size = 0;
        std::vector<float> gapRanges;

        for(unsigned int i = current_segment.front(); i < num_beams; ++i)
        {
                float rs = sensor_ranges[i];

                if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segmentdepthThreshold || i == num_beams - 1)
                {
            
                        // Found a gap
                        ++gap_size;
                        gapRanges.push_back ( rs );

                        if (gap_size >= maxGapSize || i == num_beams - 1)
                        {
                                i = current_segment.back() + 1;

                                if (current_segment.size()  >= minSegmentSize )
                                {
                                        // calculate bounding box
                                        geo::Vec2 seg_min, seg_max;
                                        for(unsigned int k = 0; k <  current_segment.size(); ++k)
                                        {
                                                geo::Vector3 p = lrf_model.rayDirections()[ current_segment[k]] * sensor_ranges[current_segment[k]];

                                                if (k == 0)
                                                {
                                                seg_min = geo::Vec2(p.x, p.y);
                                                seg_max = geo::Vec2(p.x, p.y);
                                                }
                                                else
                                                {
                                                seg_min.x = std::min(p.x, seg_min.x);
                                                seg_min.y = std::min(p.y, seg_min.y);
                                                seg_max.x = std::max(p.x, seg_max.x);
                                                seg_max.y = std::max(p.y, seg_max.y);
                                                }
                                        }

                                        geo::Vec2 bb = seg_max - seg_min;
                                        if ( ( bb .x > minClusterSize || bb.y > minClusterSize ) && bb.x < maxClusterSize && bb.y < maxClusterSize )
                                        {
                                                segments.push_back ( current_segment );       
                                        }
                                }

                                current_segment.clear();
                                gapRanges.clear();

                                // Find next good value
                                while (i < num_beams &&  sensor_ranges[i] == 0)
                                {
                                ++i; // check for confidence left
                                }

                                int nPointsToCheck = POINTS_TO_CHECK_CONFIDENCE;
                                if ( i < nPointsToCheck )
                                {
                                nPointsToCheck = i;
                                }

                                current_segment.push_back ( i );
                        }
                }
                else
                {
                gap_size = 0;
                gapRanges.clear();
                current_segment.push_back ( i );
                }
        }
    
    return segments;
}            

void addEvidenceWIRE(wire_msgs::WorldEvidence& world_evidence, measuredPropertyInfo measuredPropertyInformation)
{
         wire_msgs::ObjectEvidence obj_evidence;
         
        // Set the position property
        wire_msgs::Property posProp;
        posProp.attribute = "position";
        
        // Set position (x,y,z), set the covariance matrix as 0.005*identity_matrix
        double x, y, z;
        
        if(measuredPropertyInformation.featureProperty.getFeatureProbabilities().get_pCircle() > measuredPropertyInformation.featureProperty.getFeatureProbabilities().get_pRectangle() )
        {
                x = measuredPropertyInformation.featureProperty.getCircle().get_x();
                y = measuredPropertyInformation.featureProperty.getCircle().get_y();
                z = measuredPropertyInformation.featureProperty.getCircle().get_z();
        }
        else
        {
                x = measuredPropertyInformation.featureProperty.getRectangle().get_x();
                y = measuredPropertyInformation.featureProperty.getRectangle().get_y();
                z = measuredPropertyInformation.featureProperty.getRectangle().get_z();
        }

        // TODO line below gives problems when publishing info
        pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.0005, 0.0005, 0.0005)), posProp.pdf);
        obj_evidence.properties.push_back(posProp);
         
         // Set the discrete class label property
        wire_msgs::Property classProp;
        classProp.attribute = "class_label";
        pbl::PMF classPMF;
        
        // Probability of the class label
        classPMF.setDomainSize(100);
        classPMF.setProbability("Circle", measuredPropertyInformation.featureProperty.getFeatureProbabilities().get_pCircle() );
        classPMF.setProbability("Rectangle", measuredPropertyInformation.featureProperty.getFeatureProbabilities().get_pRectangle() );
        pbl::PDFtoMsg(classPMF, classProp.pdf);
        obj_evidence.properties.push_back(classProp);

        world_evidence.object_evidence.push_back(obj_evidence);
        // TODO delete objects??
         
        return;
}

// ----------------------------------------------------------------------------------------------------

LaserPluginTracking::LaserPluginTracking() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LaserPluginTracking::~LaserPluginTracking()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    std::string laser_topic;
    config.value("laser_topic", laser_topic);
    config.value("world_association_distance", world_association_distance_);
    config.value("min_segment_size_pixels", min_segment_size_pixels_);
    config.value("segment_depth_threshold", segment_depth_threshold_);
    config.value("min_cluster_size", min_cluster_size_);
    config.value("max_cluster_size", max_cluster_size_);
    config.value("max_gap_size", max_gap_size_);
    config.value("min_gap_size_for_split", min_gap_size_for_split_);
    config.value("nominal_corridor_width", nominal_corridor_width_);    
    config.value("dist_for_object_split", dist_for_object_split_);   
    
    int i_fit_entities = 0;
    config.value("fit_entities", i_fit_entities, tue::OPTIONAL);
    fit_entities_ = (i_fit_entities != 0);
     
    int i_correct_x_yPos = 0;
    config.value("correctXYpos", i_correct_x_yPos, tue::OPTIONAL);
    correctXYpos_ = (i_correct_x_yPos != 0);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);
    
    //std::cout << "Init plugin tracking: laser_topic = " << laser_topic << std::endl;

    // Communication
    unsigned int bufferSize = 1; // TODO increase to 3(?) in order to make it possible to process more laser data in 1 iteration. Set low for testing purposes now.
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, bufferSize, &LaserPluginTracking::scanCallback, this);
    door_pub_ = nh.advertise<ed_sensor_integration::doorDetection>("door", 3);
    ObjectMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/measuredObjects", 3 ); // TEMP
    pose_updated_pub_ = nh.advertise<geometry_msgs::PoseStamped> ( "PoseTest", 3 ); // TEMP
    points_measured_pub_ = nh.advertise<visualization_msgs::Marker> ( "MeasuredPoints", 3 ); // TEMP;
    points_modelled_pub_ = nh.advertise<visualization_msgs::Marker> ( "ModelledPoints", 3 ); // TEMP;
    points_modelled_all_pub_ = nh.advertise<visualization_msgs::Marker> ( "ModelledPointsAll", 3 ); // TEMP;
    points_measured_all_pub_ = nh.advertise<visualization_msgs::Marker> ( "MeasuredPointsAll", 3 ); // TEMP;
    cornerPointModelled_pub_ = nh.advertise<visualization_msgs::Marker> ( "cornerPointModelled", 3 ); // TEMP
    cornerPointMeasured_pub_ = nh.advertise<visualization_msgs::Marker> ( "cornerPointMeasured", 3 ); // TEMP
    associatedPoints_pub_ = nh.advertise<sensor_msgs::LaserScan> ("ed/associatedPoints", 3); // TEMP
  //  world_evidence_publisher_ = nh.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);

    tf_listener_ = new tf::TransformListener;

    init.properties.registerProperty ( "Feature", featureProperties_, new FeaturPropertiesInfo ); 
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{    
    cb_queue_.callAvailable();

    while(!scan_buffer_.empty())
    {
        sensor_msgs::LaserScan::ConstPtr scan = scan_buffer_.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF

        try
        {
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, t_sensor_pose);
            scan_buffer_.pop();
            geo::Pose3D sensor_pose;
            geo::convert(t_sensor_pose, sensor_pose);
            update(world, scan, sensor_pose, req);
        }
        catch(tf::ExtrapolationException& ex)
        {
            ROS_WARN_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << ex.what());
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform("map", scan->header.frame_id, ros::Time(0), latest_transform);

                if (scan_buffer_.front()->header.stamp > latest_transform.stamp_)
                {
                    // Scan is too new
                    break;
                }
                else
                {
                    // Otherwise it has to be too old (pop it because we cannot use it anymore)
                    scan_buffer_.pop();
                }
            }
            catch(tf::TransformException& exc)
            {
                scan_buffer_.pop();
            }
        }
        catch(tf::TransformException& exc)
        {
            ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << exc.what());
            scan_buffer_.pop();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                         geo::Pose3D& sensor_pose, ed::UpdateRequest& req)
{    
    tue::Timer t_total;
    t_total.start();
    double current_time = ros::Time::now().toSec();
    
//  struct timeval now;
//  gettimeofday(&now, NULL);
// 
//  std::cout << "Start of plugin at t = " << now.tv_sec << "." << now.tv_usec << std::endl;
  //std::cout << "Start of tracking plugin" << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges(scan->ranges.size());
    for(unsigned int i = 0; i < scan->ranges.size(); ++i)
    {
        float r = scan->ranges[i];
        if (r > scan->range_max)
            sensor_ranges[i] = r;
        else if (r == r && r > scan->range_min)
            sensor_ranges[i] = r;
        else
            sensor_ranges[i] = 0;
    }

    unsigned int num_beams = sensor_ranges.size();

    if (lrf_model_.getNumBeams() != num_beams)
    {
        lrf_model_.setNumBeams(num_beams);
        lrf_model_.setAngleLimits(scan->angle_min, scan->angle_max);
        lrf_model_.setRangeLimits(scan->range_min, scan->range_max);
    }
    
    // - - - - - - - - - - - - - - - - - -
    // Filter laser data (get rid of ghost points)

    for(unsigned int i = 1; i < num_beams - 1; ++i)
    {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if (std::abs(rs - sensor_ranges[i - 1]) > 0.1 && std::abs(rs - sensor_ranges[i + 1]) > 0.1)  // TODO: magic number
        {
            sensor_ranges[i] = 0;//sensor_ranges[i - 1]; // Do not use the point if there are problems
        }
    }
    
    std::vector<double> model_ranges(num_beams, 0);
//     std::cout << "tracking, before renderWorld: sensor_pose = " << sensor_pose << std::endl;
    
    renderWorld(sensor_pose, model_ranges, world, lrf_model_);

    // - - - - - - - - - - - - - - - - - -
    // Fit the doors

    if (fit_entities_)
    {
        for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;

            if (!e->shape() || !e->has_pose())
                continue;

            geo::Pose3D sensor_pose_inv = sensor_pose.inverse();
            geo::Pose3D e_pose_SENSOR = sensor_pose_inv * e->pose();

            // If not in sensor view, continue
            if (e_pose_SENSOR.t.length2() > 5.0 * 5.0 || e_pose_SENSOR.t.x < 0)
                continue;

            if (e->hasType("left_door") || e->hasType("door_left"))
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity(*e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache);
                req.setPose(e->id(), new_pose);
                //std::cout << "left_door" << std::endl;

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * new_pose);

                geo::LaserRangeFinder::RenderResult res(model_ranges);
                lrf_model_.render(opt, res);
            }
            else if (e->hasType("right_door") || e->hasType("door_right"))
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity(*e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache);
                req.setPose(e->id(), new_pose);
                //std::cout << "right_door" << std::endl;

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * new_pose);

                geo::LaserRangeFinder::RenderResult res(model_ranges);
                lrf_model_.render(opt, res);
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    std::vector<float> modelRangesAssociatedRanges2StaticWorld(sensor_ranges.size(), 0.0 );
    
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];
//         model_rangesOut.push_back(rm);
        
//         std::cout << "Sensor ranges = " << sensor_ranges[i] << ", model_ranges = " << model_ranges[i]<< std::endl;

        if (   std::abs(rm - rs) < world_association_distance_)  // If the sensor point is behind the world model, skip it
        {
                modelRangesAssociatedRanges2StaticWorld[i] = model_ranges[i];
        }
    }

    float segmentDepthThresholdSmall = segment_depth_threshold_;
    std::vector<ScanSegment> staticSegments = determineSegments(modelRangesAssociatedRanges2StaticWorld, max_gap_size_, min_segment_size_pixels_, segmentDepthThresholdSmall, lrf_model_, min_cluster_size_, max_cluster_size_ );    
    std::sort(staticSegments.begin(), staticSegments.end(), sortBySegmentSize);

    geo::Vec2f cornerPointMeasured, cornerPointModelled;
    std::vector < unsigned int > possibleCorners, possibleCornersModel;
    std::vector< geo::Vec2f> points, modelledPoints ;
    geo::Vec2f xyDiff;
    
    bool cornerPointsFound = false, angleCorrectionFound = false;
    int shift, measuredCornerElement;
    float diffAngle;
    unsigned int sensorElementOfCornerModelled ;
    
    tf::Quaternion q ( sensor_pose.getQuaternion().x, sensor_pose.getQuaternion().y, sensor_pose.getQuaternion().z, sensor_pose.getQuaternion().w );
    tf::Matrix3x3 matrix ( q );
    double rollSensor, pitchSensor, yawSensor;
    matrix.getRPY ( rollSensor, pitchSensor, yawSensor );
     
    for(unsigned int iSegment = 0; iSegment < staticSegments.size(); iSegment++)
    {
           ScanSegment staticSegment = staticSegments[iSegment];

           if( staticSegment.size() < min_segment_size_pixels_ )
           {
                   break;
           }
           
           points.clear(); modelledPoints.clear() ;
           
           for( unsigned int iSeg2Point = 0; iSeg2Point < staticSegment.size(); iSeg2Point++)
           {
                   unsigned int j = staticSegment[iSeg2Point];
                   geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];
                   geo::Vector3 p_model = lrf_model_.rayDirections() [j] * modelRangesAssociatedRanges2StaticWorld[j];
                   
                   // Transform to world frame
                   geo::Vector3 p = sensor_pose * p_sensor;
                   geo::Vector3 p_modelled = sensor_pose * p_model;
                   
                   if ( (p - p_modelled).length() > 0.2 )
                   {
                        //   std::cout << termcolor::yellow << "p = " << p << ", p_modelled = " << p_modelled << "j = " << j << "sensor_ranges = " << sensor_ranges[j];
                         //  std::cout << "modelRangesAssociatedRanges2StaticWorld = " << modelRangesAssociatedRanges2StaticWorld[j] << "\t" << termcolor::reset;
                   }
                   
                   
                   // Add to cv array
                   points.push_back( geo::Vec2f ( p.x, p.y ) );
                   modelledPoints.push_back( geo::Vec2f ( p_modelled.x, p_modelled.y ) );   
           }
           
           unsigned int segmentLength = staticSegment.size();       
           unsigned int startElement = 0;// segmentLength / SEGMENT_DIVISION_FOR_FITTING;
           unsigned int finalElement = segmentLength;// segmentLength * (SEGMENT_DIVISION_FOR_FITTING - 1)/SEGMENT_DIVISION_FOR_FITTING;

           std::vector<geo::Vec2f>::iterator it_start = points.begin(); std::advance(it_start, startElement);
           std::vector<geo::Vec2f>::iterator it_end = points.begin(); std::advance( it_end, finalElement );
           
           possibleCorners.clear();
           bool cornersFoundMeasured = ed::tracking::findPossibleCorners (points, &possibleCorners, MIN_DISTANCE_CORNER_DETECTION_LARGE, min_segment_size_pixels_  );
          
           if( !cornersFoundMeasured && ! angleCorrectionFound) // beause we want to correct based on a straight line.
           {
                // straight line detected. Take a subset (2 middle quarters) of this line and do a fit for line as well as the corresponding fit for the ranges expected in the WM

                unsigned int segmentLength = staticSegment.size();       
                unsigned int startElement = 0; //segmentLength / SEGMENT_DIVISION_FOR_FITTING;
                unsigned int finalElement = segmentLength; //segmentLength * (SEGMENT_DIVISION_FOR_FITTING - 1)/SEGMENT_DIVISION_FOR_FITTING;

                std::vector<geo::Vec2f> measuredPoints(finalElement - startElement), modelledPoints(finalElement - startElement);
                unsigned int counter = 0;
                
                for(unsigned int iLineFit = startElement; iLineFit < finalElement; iLineFit ++)       
                {
                        unsigned int element = staticSegment[iLineFit];
                        float angle = yawSensor + lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*element;
                        measuredPoints[counter].x = sensor_pose.getOrigin().getX() + sensor_ranges[element]*cos( angle );
                        measuredPoints[counter].y = sensor_pose.getOrigin().getY() + sensor_ranges[element]*sin( angle );
                        
                        modelledPoints[counter].x = sensor_pose.getOrigin().getX() + modelRangesAssociatedRanges2StaticWorld[element]*cos( angle );
                        modelledPoints[counter].y = sensor_pose.getOrigin().getY() + modelRangesAssociatedRanges2StaticWorld[element]*sin( angle );
                        counter++;
                }

                visualization_msgs::Marker pointsMeasured, pointsModelled;

                pointsModelled.header.frame_id = "/map";
                pointsModelled.header.stamp = scan->header.stamp;
                pointsModelled.ns = "modelledPoints";
                pointsModelled.id = 1;
                pointsModelled.type = visualization_msgs::Marker::POINTS;
                pointsModelled.action = visualization_msgs::Marker::ADD;
        //      pointsModelled.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollSensor, pitchSensor, yawSensor );
                pointsModelled.scale.x = 0.1;
                pointsModelled.scale.y = 0.1;
                pointsModelled.scale.z = 0.1;
                pointsModelled.color.r = 0.0;
                pointsModelled.color.g = 0.0;
                pointsModelled.color.b = 1.0;
                pointsModelled.color.a = 1.0; 
                pointsModelled.lifetime = ros::Duration( TIMEOUT_TIME );
                        
                for(int iPoints = 0; iPoints < modelledPoints.size(); iPoints++)
                {
                        geometry_msgs::Point p;
                        
                        p.x = modelledPoints[iPoints].x;
                        p.y = modelledPoints[iPoints].y;
                        p.z = sensor_pose.getOrigin().getZ();
                        
                        pointsModelled.points.push_back(p);
                }
                points_modelled_pub_.publish( pointsModelled );
                
               pointsMeasured.header.frame_id = "/map";
               pointsMeasured.header.stamp = scan->header.stamp;
               pointsMeasured.ns = "measuredPoints";
               pointsMeasured.id = 1;
               pointsMeasured.type = visualization_msgs::Marker::POINTS;
               pointsMeasured.action = visualization_msgs::Marker::ADD;
        //                     pointsMeasured.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollSensor, pitchSensor, yawSensor );
               pointsMeasured.scale.x = 0.1;
               pointsMeasured.scale.y = 0.1;
               pointsMeasured.scale.z = 0.1;
               pointsMeasured.color.r = 0.0;
               pointsMeasured.color.g = 1.0;
               pointsMeasured.color.b = 0.0;
               pointsMeasured.color.a = 1.0; 
               pointsMeasured.lifetime = ros::Duration( TIMEOUT_TIME );
                        
                for(int iPoints = 0; iPoints < measuredPoints.size(); iPoints++)
                {
                        geometry_msgs::Point p;
                        
                        p.x = measuredPoints[iPoints].x;
                        p.y = measuredPoints[iPoints].y;
                        p.z = sensor_pose.getOrigin().getZ();
                        pointsMeasured.points.push_back(p);
                }
                points_measured_pub_.publish( pointsMeasured );
                
                Eigen::VectorXf lineFitParamsMeasured( 2 ), lineFitParamsModdelled( 2 );
                std::vector<geo::Vec2f>::iterator it_start = measuredPoints.begin();
                std::vector<geo::Vec2f>::iterator it_end = measuredPoints.end();
                float fitErrorMeasured =  ed::tracking::fitLine ( measuredPoints, lineFitParamsMeasured, &it_start , &it_end );
                
                it_start = modelledPoints.begin();
                it_end = modelledPoints.end();
                float fitErrorModelled =  ed::tracking::fitLine ( modelledPoints, lineFitParamsModdelled, &it_start , &it_end );
        
                double measuredAngle = atan2 ( lineFitParamsMeasured ( 1 ), 1 );
                double modelledAngle = atan2 ( lineFitParamsModdelled ( 1 ), 1 );
                
                ed::tracking::unwrap(&modelledAngle, measuredAngle, 2*M_PI);
                diffAngle = measuredAngle - modelledAngle;
                
                if(diffAngle < MAX_DEVIATION_ANGLE_CORRECTION ) // Only correct if localization partially leads to problems
                {
                        angleCorrectionFound = true;
                }
                else
                {
                        ROS_WARN("Big angle correction required. Localisation correct?");
                }
                
           }
           
           if( angleCorrectionFound )        
           {
                   break;
           }
            
           
    }
    
    geo::Pose3D sensor_poseCorrected;
    if( angleCorrectionFound )
    {
            yawSensor -= diffAngle;
            ed::tracking::wrap2Interval(&yawSensor, (double) 0.0, (double) 2*M_PI);
            
            sensor_poseCorrected = sensor_pose;
            sensor_poseCorrected.setRPY(rollSensor, pitchSensor, yawSensor );
                        
            // Rotation updated, so this influences the delta in x,y. Recompute the cornerPointModelled                        
             /*
                geo::Vector3 p_model = lrf_model_.rayDirections() [sensorElementOfCornerModelled] * modelRangesAssociatedRanges2StaticWorld[sensorElementOfCornerModelled];
                geo::Vector3 p_modelled = sensor_poseCorrected * p_model;
                cornerPointModelled = geo::Vec2f ( p_modelled.x, p_modelled.y );
              */
    }
            /*
      if( cornerPointsFound && correctXYpos_)
      {
//              std::cout << "Diff init " << std::endl;       
//              geo::Vec2f diff = cornerPointMeasured - cornerPointModelled;
                geo::Vec2f diff = xyDiff;
                    
                geo::Vec3T< geo::real > updatedPos(sensor_poseCorrected.getOrigin().getX() - diff.x, sensor_poseCorrected.getOrigin().getY() - diff.y, sensor_poseCorrected.getOrigin().getZ() );
                sensor_poseCorrected.setOrigin( updatedPos );
                    
            }
            */
            
     // TEMP TO TEST
     geometry_msgs::PoseStamped updatedSensorPose;
     updatedSensorPose.header = scan->header;
     updatedSensorPose.header.frame_id = "map";
     updatedSensorPose.pose.position.x = sensor_pose.getOrigin().getX();
     updatedSensorPose.pose.position.y = sensor_pose.getOrigin().getY();
     updatedSensorPose.pose.position.z = sensor_pose.getOrigin().getZ();
     updatedSensorPose.pose.orientation.x = sensor_pose.getQuaternion().getX();
     updatedSensorPose.pose.orientation.y = sensor_pose.getQuaternion().getY();
     updatedSensorPose.pose.orientation.z = sensor_pose.getQuaternion().getZ();
     updatedSensorPose.pose.orientation.w = sensor_pose.getQuaternion().getW();
                
     pose_updated_pub_.publish( updatedSensorPose );                             
        
     renderWorld(sensor_poseCorrected, model_ranges, world, lrf_model_);
    
     for(unsigned int i = 0; i < num_beams; ++i)
     {
             float rs = sensor_ranges[i];
             float rm = model_ranges[i];

             if (   rs <= 0
                 || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                 || ((std::abs(rm - rs) < world_association_distance_) && rm != 0))
                {
//                         bool check1 = (rs <= 0);
//                         bool check2 = (rm > 0 && rs > rm);
//                         bool check3 = (std::abs(rm - rs) < world_association_distance_);
//                         std::cout << "i = " << i << ", checks = " << check1 << check2 << check3 << "rm, rs = " << rm << ", " << rs;
                        sensor_ranges[i] = 0;
                }
    }   
    
//     std::cout << "\n";
    
     std::vector<ScanSegment> segments = determineSegments(sensor_ranges, max_gap_size_, min_segment_size_pixels_, segment_depth_threshold_, lrf_model_, min_cluster_size_, max_cluster_size_ );         
    
    // Try to associate remaining laser points to specific entities
    std::vector<ed::WorldModel::const_iterator> it_laserEntities;
    std::vector< EntityProperty > EntityPropertiesForAssociation;
    
    // Check which entities might associate for tracking based on their latest location in order to prevent to check all the entities 
    for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it )
    {
        const ed::EntityConstPtr& e = *e_it;
        std::string laserID = "-laserTracking";
        
        if ( e->id().str().length() < laserID.length() )
        {
            continue;
        }
        
        if ( e->id().str().substr ( e->id().str().length() - laserID.size() ) == laserID )  // entity described by laser before
        {
            it_laserEntities.push_back ( e_it );

            ed::tracking::FeatureProperties featureProperties;


	if( !e->property( featureProperties_) )
	{
                req.removeEntity ( e->id() );
                continue;
	}


        featureProperties = e->property ( featureProperties_ );
        if(featureProperties.getFeatureProbabilities().getDomainSize() != 2 ) // TODO ugly! What causes that the domainsize !=2 ?
        {
                req.removeEntity ( e->id() );
                        continue;

        }

        EntityProperty currentProperty;
        float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();

        // For the entities which already exist in the WM, determine the relevant properties in order to determine which entities _might_ associate to which clusters
        if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() )
        {
                ed::tracking::Circle circle = featureProperties.getCircle();
                circle.predictAndUpdatePos(dt);
             
                currentProperty.entity_min.x = circle.get_x() - ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_max.x = circle.get_x() + ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_min.y = circle.get_y() - ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_max.y = circle.get_y() + ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
        }
        else
        {
                ed::tracking::Rectangle rectangle = featureProperties.getRectangle();
                rectangle.predictAndUpdatePos(dt);
                
                std::vector<geo::Vec2f> corners = rectangle.determineCorners ( ASSOCIATION_DISTANCE );
                currentProperty.entity_min = corners[0];
                currentProperty.entity_max = corners[0];
                for ( unsigned int i_corner = 1; i_corner < corners.size(); i_corner++ )
                {
                    currentProperty.entity_min.x = std::min ( corners[i_corner].x, currentProperty.entity_min.x );
                    currentProperty.entity_min.y = std::min ( corners[i_corner].y, currentProperty.entity_min.y );
                    currentProperty.entity_max.x = std::max ( corners[i_corner].x, currentProperty.entity_max.x );
                    currentProperty.entity_max.y = std::max ( corners[i_corner].y, currentProperty.entity_max.y );
                }
            }
            
            EntityPropertiesForAssociation.push_back ( currentProperty );
        }
    }
    
     std::vector< PointsInfo > associatedPointsInfo( it_laserEntities.size() );
     sensor_msgs::LaserScan test;
     test = *scan;

     for(unsigned int iTestLength = 0; iTestLength < test.ranges.size(); iTestLength++)
     {
             test.ranges[iTestLength] = 0.0;
     }
    
    for ( unsigned int iSegments = 0; iSegments < segments.size(); ++iSegments )
    {
        // First, determine the properties of each segment
        ScanSegment& segment = segments[iSegments];
        unsigned int segment_size = segment.size();

        std::vector<geo::Vec2f> points ( segment_size );
        std::vector<unsigned int> segmentIDs ( segment_size );
        geo::Vec2f seg_min, seg_max;

        for ( unsigned int iSegment = 0; iSegment < segment_size; ++iSegment )
        {
            unsigned int j = segment[iSegment];
            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];
            test.ranges[j] = sensor_ranges[j];
            
            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[iSegment] = geo::Vec2f ( p.x, p.y );
            segmentIDs[iSegment] = j;
            
            if ( iSegment == 0 )
            {
                seg_min = points[iSegment];
                seg_max = points[iSegment];
            }
            else
            {
                seg_min.x = std::min ( points[iSegment].x, seg_min.x );
                seg_min.y = std::min ( points[iSegment].y, seg_min.y );
                seg_max.x = std::max ( points[iSegment].x, seg_max.x );
                seg_max.y = std::max ( points[iSegment].y, seg_max.y );
            }
        }
    
        // After the properties of each segment are determined, check which clusters and entities might associate
        std::vector< int > possibleSegmentEntityAssociations;
        
        for ( unsigned int jj = 0; jj < it_laserEntities.size(); ++jj )
        {
            // check if 1 of the extrema of the segment might be related to the exploded entity
            bool check1 =  seg_min.x > EntityPropertiesForAssociation[jj].entity_min.x && seg_min.x < EntityPropertiesForAssociation[jj].entity_max.x ;
            bool check2 =  seg_max.x > EntityPropertiesForAssociation[jj].entity_min.x && seg_max.x < EntityPropertiesForAssociation[jj].entity_max.x ;
            bool check3 =  seg_min.y > EntityPropertiesForAssociation[jj].entity_min.y && seg_min.y < EntityPropertiesForAssociation[jj].entity_max.y ;
            bool check4 =  seg_max.y > EntityPropertiesForAssociation[jj].entity_min.y && seg_max.y < EntityPropertiesForAssociation[jj].entity_max.y ;

            if ( check1 && check3 || check2 && check4 )
            {
                possibleSegmentEntityAssociations.push_back ( jj );
                const ed::EntityConstPtr& e = *it_laserEntities[jj];
            }           
        }

        // If a cluster could be associated to a (set of) entities, determine for each point to which entitiy it belongs based on a shortest distance criterion. 
        // If the distance is too large, initiate a new entity
        PointsInfo pointsNotAssociated;
        std::vector<float> distances ( points.size() );
        std::vector<unsigned int> IDs ( points.size() ); // IDs of the entity which is closest to that point
        
        for ( unsigned int i_points = 0; i_points < points.size(); ++i_points )  // Determine closest object and distance to this object. If distance too large, relate to new object
        {
            geo::Vec2f p = points[i_points];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = std::numeric_limits< unsigned int >::max();

            for ( unsigned int jj = 0; jj < possibleSegmentEntityAssociations.size(); jj++ )  // relevant entities only
            {
                const ed::EntityConstPtr& e = *it_laserEntities[ possibleSegmentEntityAssociations[jj] ];

                if( !e-> property ( featureProperties_) )
                {
                        req.removeEntity ( e->id() );
                        continue;
                }
                
                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );

                float dist;
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                
		if(featureProperties.getFeatureProbabilities().getDomainSize() != 2 )// TODO ugly!
		{
			req.removeEntity ( e->id() );
                        continue;
		}	

                double prob1 = featureProperties.getFeatureProbabilities().get_pCircle();
                double prob2 = featureProperties.getFeatureProbabilities().get_pRectangle();
                bool test = featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle();
                
                if(prob1 == -1.0 || prob2 == -1.0 ) // TODO ugly!
                {
                        req.removeEntity ( e->id() );
                        continue;
                }

                if ( prob1 > prob2 )  // entity is considered to be a circle
                {
                    ed::tracking::Circle circle = featureProperties.getCircle();  
                    circle.predictAndUpdatePos( dt ); // TODO Do this update once at initialisation
                    dist = std::abs ( std::sqrt ( std::pow ( p.x - circle.get_x(), 2.0 ) + std::pow ( p.y - circle.get_y(), 2.0 ) ) - circle.get_radius() ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                }
                else     // entity is considered to be a rectangle. Check if point is inside the rectangle
                {
                    ed::tracking::Rectangle rectangle = featureProperties.getRectangle();
                    rectangle.predictAndUpdatePos( dt );// TODO Do this update once at initialisation

                    std::vector<geo::Vec2f> corners = rectangle.determineCorners ( 0.0 );

                    geo::Vec2f vx = corners[1] - corners[0];
                    geo::Vec2f vy = corners[3] - corners[0];

                    geo::Vec2f pCorrected = p - corners[0];

                    // Test if point is inside rectangle https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
                    geo::Vec2f OP = p - corners[0]; // Distance from origin to point which is tested
                    geo::Vec2f OC1 = corners[1] - corners[0];
                    geo::Vec2f OC3 = corners[3] - corners[0];

                    float OP_OC1   = OP.dot ( OC1 ); // elementwise summation
                    float OC1_OC1B = OC1.dot ( OC1 );
                    float OP_OC3   = OP.dot ( OC3 );
                    float OC3_OC3  = OC3.dot ( OC3 );

                    float minDistance = std::numeric_limits< float >::infinity();
                    
                    if ( OP_OC1 > 0 && OC1_OC1B > OP_OC1 && OP_OC3 > 0 && OC3_OC3 > OP_OC3 )   // point is inside the rectangle
                    {
                        std::vector<geo::Vec2f> p1Check = corners;

                        std::vector<geo::Vec2f> p2Check = corners; // place last element at begin
                        p2Check.insert ( p2Check.begin(), p2Check.back() );
                        p2Check.erase ( p2Check.end() );

                        for ( unsigned int ii_dist = 0; ii_dist < p1Check.size(); ii_dist++ )
                        {

                            float x1 = p1Check[ii_dist].x;
                            float x2 = p2Check[ii_dist].x;

                            float y1 = p1Check[ii_dist].y;
                            float y2 = p2Check[ii_dist].y;

                            float distance = std::abs ( ( y2 - y1 ) *p.x - ( x2 - x1 ) *p.y + x2*y1 -y2*x1 ) /std::sqrt ( std::pow ( y2-y1, 2.0 ) + std::pow ( x2-x1, 2.0 ) );
                            
                            if ( distance < minDistance )
                            {
                                minDistance = distance;
                            }
                        }
                    }
                    else     // point is outside the rectangle, https://stackoverflow.com/questions/44824512/how-to-find-the-closest-point-on-a-right-rectangular-prism-3d-rectangle/44824522#44824522
                    {
                        float tx = pCorrected.dot ( vx ) / ( vx.dot ( vx ) );
                        float ty = pCorrected.dot ( vy ) / ( vy.dot ( vy ) );

                        tx = tx < 0 ? 0 : tx > 1 ? 1 : tx;
                        ty = ty < 0 ? 0 : ty > 1 ? 1 : ty;

                        geo::Vec2f closestPoint = tx*vx + ty*vy + corners[0];

                        geo::Vec2f vector2Point = p - closestPoint;
                        minDistance = std::sqrt ( vector2Point.dot ( vector2Point ) );
                    }

                    dist = minDistance;
                }
                
                if ( dist < shortestDistance )
                {
                        shortestDistance = dist;
                        id_shortestEntity = jj;
                }
                
            }
            
            distances[i_points] = shortestDistance;
            IDs[i_points] = id_shortestEntity;
        }
        
        unsigned int IDtoCheck = IDs[0];
        unsigned int firstElement = 0;
        bool previousSegmentAssociated = false;
   
        // check groups of segments associated to a specific entity
        for ( unsigned int iDistances = 1; iDistances < distances.size(); iDistances++ )
        {
            if ( IDs[iDistances] == IDtoCheck && iDistances != distances.size() - 1 ) // ID similar and not at final reading, check if next element is associated to same entity or not
            {
                continue;
            }

            unsigned int length = iDistances - firstElement;

            if ( length >= min_segment_size_pixels_ )
            {
                float minDistance = distances[firstElement];

                for ( unsigned int iiDistances = 1; iiDistances < iDistances; iiDistances++ )
                {
                    if ( distances[iiDistances] < minDistance )
                    {
                        minDistance = distances[iiDistances];
                    }
                }
                
                bool associated = minDistance < MIN_ASSOCIATION_DISTANCE; // TODO Sufficient? Make it dependent on the covariance/time as long as we did not see it? 
                
                if( associated && !previousSegmentAssociated && firstElement > 0 ) // check for possibility to reassociate previous section
                {

                         geo::Vec2f lastPointPreviousSegment = points[firstElement - 1];
                        geo::Vec2f firstPointCurrentSegment = points[firstElement];

                        float interSegmentDistance = std::sqrt( std::pow(lastPointPreviousSegment.x-firstPointCurrentSegment.x, 2.0) + std::pow(lastPointPreviousSegment.y-firstPointCurrentSegment.y, 2.0) );
                        
                        if( interSegmentDistance < MIN_ASSOCIATION_DISTANCE_SEGMENTS)  // reassociate previous section
                        {
                                previousSegmentAssociated = true;
                                unsigned int previousID = IDs[iDistances - 1];
                                
                                const ed::EntityConstPtr& e1 = *it_laserEntities[ possibleSegmentEntityAssociations[IDtoCheck] ];
                                const ed::EntityConstPtr& e2 = *it_laserEntities[ possibleSegmentEntityAssociations[previousID] ];                                                                
                                
                                append(associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points, associatedPointsInfo.back().points);
                                append(associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs, associatedPointsInfo.back().laserIDs);

                                
                                if( associatedPointsInfo.size() > it_laserEntities.size() ) // Unassociated points were added
                                {
                                        associatedPointsInfo.erase ( associatedPointsInfo.end() );
                                }
                                else // keep the possibility to associate points to a specific entity
                                {
                                        associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points.clear();
                                        associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs.clear();
                                }
                                
                        }                             
                }

            
                if ( !associated && previousSegmentAssociated) 
                {
                        // boundaries for distance and that those boundaries are associated to a object                                
                        geo::Vec2f lastPointPreviousSegment = points[firstElement - 1];
                        geo::Vec2f firstPointCurrentSegment = points[firstElement];
                        
                        float interSegmentDistance = std::sqrt( std::pow(lastPointPreviousSegment.x-firstPointCurrentSegment.x, 2.0) + std::pow(lastPointPreviousSegment.y-firstPointCurrentSegment.y, 2.0) );
                        
                        if( interSegmentDistance < MIN_ASSOCIATION_DISTANCE_SEGMENTS)
                        {
                                associated = true;
                                unsigned int previousID = IDs[iDistances - 1];
                                IDtoCheck = previousID;
                        }           
                }

                if ( associated )  // add all points to associated entity
                {
                        previousSegmentAssociated = true;
//                    const ed::EntityConstPtr& entityToTest = *it_laserEntities[ possibleSegmentEntityAssociations[IDtoCheck] ];
                        append( associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points, points, firstElement, iDistances );
                        append( associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs, segmentIDs, firstElement, iDistances );
                }
                else
                {
                        previousSegmentAssociated = false;
                        pointsNotAssociated = PointsInfo();
                
                        for ( unsigned int i_points = firstElement; i_points < iDistances; ++i_points )
                        {
                                pointsNotAssociated.points.push_back ( points[i_points] );
                                pointsNotAssociated.laserIDs.push_back ( segmentIDs[i_points] );
                        }
                        associatedPointsInfo.push_back ( pointsNotAssociated );
                }
            }
            
            firstElement = iDistances;
            IDtoCheck = IDs[iDistances];
        }

    }
    
    associatedPoints_pub_.publish(test);
   
    splitSegmentsWhenGapDetected( associatedPointsInfo, min_gap_size_for_split_, min_segment_size_pixels_, dist_for_object_split_, sensor_ranges, scan);
    std::vector<measuredPropertyInfo> measuredProperties ( associatedPointsInfo.size() ); // The first sequence in this vector (with the length of laser entitities) consits of the properties corresponding to existing entities
    
   for ( unsigned int iList = 0; iList < associatedPointsInfo.size(); iList++ )
   {
           measuredProperties[iList].propertiesDescribed = false;
           std::vector<geo::Vec2f> points  = associatedPointsInfo[iList].points ;

           if( points.size() < (unsigned int ) min_segment_size_pixels_ )
                   continue;
       
           std::vector<unsigned int> cornerIndices;
           std::vector<geo::Vec2f>::iterator it_start = points.begin();
           std::vector<geo::Vec2f>::iterator it_end = points.end();
           unsigned int cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
          
          if( ed::tracking::findPossibleCorner ( points, &cornerIndices, &it_start, &it_end, MIN_DISTANCE_CORNER_DETECTION, (unsigned int) min_segment_size_pixels_ ) )
          {
                  cornerIndex = cornerIndices[0];
          }

          for ( std::vector<unsigned int>::const_iterator it_in = cornerIndices.begin(); it_in != cornerIndices.end(); ++it_in )
          {
                const unsigned int& index = *it_in;
          }

          ed::tracking::Circle circle;   
          ed::tracking::Rectangle rectangle;    
          std::vector<geo::Vec2f>::iterator it_low, it_high;
        
          ed::tracking::FITTINGMETHOD method = ed::tracking::CIRCLE;
          float errorCircle = ed::tracking::fitObject ( points, method, &cornerIndex, &rectangle, &circle, &it_low, &it_high, sensor_pose, (unsigned int) min_segment_size_pixels_ );
          unsigned int elementLow = associatedPointsInfo[iList].laserIDs[0];
          unsigned int elementHigh = associatedPointsInfo[iList].laserIDs.back();

          method = ed::tracking::determineCase ( points, &cornerIndex, &it_low, &it_high, sensor_pose, (unsigned int) min_segment_size_pixels_); // chose to fit a single line or a rectangle (2 lines)
          float errorRectangle = ed::tracking::fitObject ( points, method,  &cornerIndex, &rectangle, &circle, &it_low, &it_high,  sensor_pose, (unsigned int) min_segment_size_pixels_ );

          measuredProperties[iList].confidenceRectangleWidth = false;
          measuredProperties[iList].confidenceRectangleWidthLow = false;
          measuredProperties[iList].confidenceRectangleWidthHigh = false;
          measuredProperties[iList].confidenceRectangleDepth = false;
          measuredProperties[iList].confidenceRectangleDepthLow = false;
          measuredProperties[iList].confidenceRectangleDepthHigh = false;
          measuredProperties[iList].methodRectangle = method;
          measuredProperties[iList].fittingErrorCircle = errorCircle;
          measuredProperties[iList].fittingErrorRectangle = errorRectangle;
        
          geo::Quaternion sensorQuaternion  = sensor_pose.getQuaternion();
          tf::Quaternion q ( sensorQuaternion.getX(), sensorQuaternion.getY(), sensorQuaternion.getZ(), sensorQuaternion.getW() );
          tf::Matrix3x3 m ( q );
          double LRF_roll, LRF_pitch, LRF_yaw;
          m.getRPY ( LRF_roll, LRF_pitch, LRF_yaw );
        
          unsigned int IDLowWidth = 0, IDHighWidth = 0, IDLowDepth = 0, IDHighDepth = 0;
          unsigned int PointIDLowWidth = 0, PointIDHighWidth = 0, PointIDLowDepth = 0, PointIDHighDepth = 0;
        
          if( method == ed::tracking::LINE )
          {       
                  IDLowWidth = associatedPointsInfo[iList].laserIDs[0];
                  IDHighWidth = associatedPointsInfo[iList].laserIDs.back();     
                  
                  PointIDLowWidth = 0;
                  PointIDHighWidth = associatedPointsInfo[iList].laserIDs.size() - 1;
          }
          else if ( method == ed::tracking::RECTANGLE )
          {
                  // For width-information
                  IDLowWidth = associatedPointsInfo[iList].laserIDs[0];
                  IDHighWidth = associatedPointsInfo[iList].laserIDs[0] + cornerIndex;
                
                  PointIDLowWidth = 0;
                  PointIDHighWidth = cornerIndex;
                
                  // For depth information
                  IDLowDepth = IDHighWidth; // cause corner belongs to both points. TODO what if the corner is occluded? Does this automatically lead to low confidences? -> TEST
                  IDHighDepth = associatedPointsInfo[iList].laserIDs.back();
                
                  PointIDLowDepth = cornerIndex;
                  PointIDHighDepth = associatedPointsInfo[iList].laserIDs.size() - 1;
        }

        for(unsigned int iPrint = 0; iPrint < associatedPointsInfo[iList].laserIDs.size(); iPrint++)
        {
                if(associatedPointsInfo[iList].laserIDs[iPrint]  > sensor_ranges.size())
                {
                        ROS_ERROR("BAD INFORMATION!!");
                         exit(0);
                }
        }
        
        for(unsigned int iConfidence = 0; iConfidence < 2; iConfidence++) // Determine for both with and depth
        {
                bool determineWidthConfidence = false, determineDepthConfidence = false; 
                
                if(iConfidence == 0)
                {
                        determineWidthConfidence = true;
                        elementLow = IDLowWidth;
                        elementHigh = IDHighWidth;
                } 
                
                if(iConfidence == 1)
                {
                        determineDepthConfidence = true;
                        elementLow = IDLowDepth; // cause corner belongs to both points. TODO what if the corner is occluded? Does this automatically lead to low confidences? -> TEST
                        elementHigh = IDHighDepth;
                }
                
                if(elementLow == 0 && elementHigh == 0) // skip if no relevant information can be obtained
                        continue;
                
                 float avgAngle = 0.0;
                 for(unsigned int iiAssPoints = elementLow; iiAssPoints < elementHigh; iiAssPoints++)
                 {
                         avgAngle +=  LRF_yaw + lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*iiAssPoints;
                 }         
                 avgAngle /= (elementHigh - elementLow);
                 
                 if(determineWidthConfidence)
                 {
                         ed::tracking::unwrap (&avgAngle, rectangle.get_yaw(), (float) M_PI);
                 }
                 else if (determineDepthConfidence)
                 {
                          ed::tracking::unwrap (&avgAngle, rectangle.get_yaw() + (float) M_PI_2, (float) M_PI);
                 }
                 
                 if ( (std::fabs(rectangle.get_yaw() - avgAngle) < ANGLE_MARGIN_FITTING && determineWidthConfidence) || (std::fabs(rectangle.get_yaw() + M_PI_2 - avgAngle) < ANGLE_MARGIN_FITTING && determineDepthConfidence) )
                 {
                         // Make sure you have a good view on the relevant side. If you are measuring almost parallel, the distance between points might be larger than the criterion, meaning you 
                         // measure a shorter length. In this case, you are uncertain about the dimension.
                         
                         if(determineWidthConfidence)
                         {
                                 measuredProperties[iList].confidenceRectangleWidth = false;
                                 measuredProperties[iList].confidenceRectangleWidthLow = false;
                                 measuredProperties[iList].confidenceRectangleWidthHigh = ed::tracking::determineCornerConfidence ( scan, elementHigh, false);
                                 
                                 if( measuredProperties[iList].confidenceRectangleWidthHigh )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDHighWidth] );  
                                 }
                                 
                                 PointIDLowWidth = 0;
                         }
                                 
                         if (determineDepthConfidence)
                         {
                                 measuredProperties[iList].confidenceRectangleDepth = false;
                                 measuredProperties[iList].confidenceRectangleDepthLow = ed::tracking::determineCornerConfidence ( scan, elementLow, true); 
                                 measuredProperties[iList].confidenceRectangleDepthHigh = false;
                                 
                                 if( measuredProperties[iList].confidenceRectangleDepthLow )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDLowDepth] );  
                                 }
                         }
                                 
                 }
                 else
                 {
                         if(determineWidthConfidence)
                         {
                                 measuredProperties[iList].confidenceRectangleWidthLow = ed::tracking::determineCornerConfidence ( scan, elementLow, true);
                                 measuredProperties[iList].confidenceRectangleWidthHigh = ed::tracking::determineCornerConfidence ( scan, elementHigh, false); 
                                 measuredProperties[iList].confidenceRectangleWidth = (measuredProperties[iList].confidenceRectangleWidthLow && measuredProperties[iList].confidenceRectangleWidthHigh );
                                 
                                 if( measuredProperties[iList].confidenceRectangleWidthLow )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDLowWidth] );  
                                 }
                                 
                                 if(  measuredProperties[iList].confidenceRectangleWidthHigh )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDHighWidth] );                                    
                                 }
                                 
                         }
                         
                          if (determineDepthConfidence)
                          {
                                 measuredProperties[iList].confidenceRectangleDepthLow = ed::tracking::determineCornerConfidence ( scan, elementLow, true) ;
                                 measuredProperties[iList].confidenceRectangleDepthHigh = ed::tracking::determineCornerConfidence ( scan, elementHigh, false) ; 
                                 measuredProperties[iList].confidenceRectangleDepth = (measuredProperties[iList].confidenceRectangleDepthLow && measuredProperties[iList].confidenceRectangleDepthHigh );
                                 
                                 if( measuredProperties[iList].confidenceRectangleDepthLow )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDLowDepth] );  
//                                          std::cout << "Point added 3 = " << associatedPointsInfo[iList].points[PointIDLowDepth] << std::endl;
                                 }
                                 
                                 if(  measuredProperties[iList].confidenceRectangleDepthHigh )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDHighDepth] );  
//                                          std::cout << "Point added 4 = " << associatedPointsInfo[iList].points[PointIDHighDepth] << std::endl;
                                 }
                          }
                  }    
        }
         
        ed::tracking::FeatureProbabilities prob;
        if ( prob.setMeasurementProbabilities ( errorRectangle, errorCircle, 2*circle.get_radius() , nominal_corridor_width_ ) )
        {

            ed::tracking::FeatureProperties properties;
            properties.setFeatureProbabilities ( prob );
            properties.setCircle ( circle );
            properties.setRectangle ( rectangle );

            if ( rectangle.get_x() != rectangle.get_x() )
            {
                ROS_WARN ( "Rectangle: invalid properties set" );
                std::cout << "Prob = " << prob.get_pCircle() << ", " << prob.get_pRectangle() << std::endl;
                std::cout << "Method = " << method << std::endl;
                std::cout << "Errors = " << errorRectangle << ", " << errorCircle << std::endl;
            }

            measuredProperties[iList].featureProperty =  properties ;
            measuredProperties[iList].propertiesDescribed = true;
            
            if( iList < it_laserEntities.size() )
            {
                const ed::EntityConstPtr& e = *it_laserEntities[ iList ];
            }
            else
            {
//                 std::cout << "Measured properties of new entity equals" << std::endl;
            }
        }
        
    }  

    
    unsigned int marker_ID = 0; // To Do: After tracking, the right ID's should be created. The ID's are used to have multiple markers.

    ed::tracking::FeatureProperties measuredProperty, entityProperties; // Measured properties and updated properties
    ed::UUID id;
    
    visualization_msgs::MarkerArray markers;
    unsigned int ID = 100;
    
    wire_msgs::WorldEvidence world_evidence;
    
    for ( unsigned int iProperties = 0; iProperties < measuredProperties.size(); iProperties++ ) // Update associated entities
    {
            if(!measuredProperties[iProperties].propertiesDescribed ) 
                    continue;
            
        measuredProperty = measuredProperties[iProperties].featureProperty;
        
                    // TEMP publish measured properties to test with WIRE
      //      addEvidenceWIRE(world_evidence, measuredProperties[iProperties] );
        
        // temporary: pub measured properties in order to visualize the properties which are added
        markers.markers.push_back( getMarker(measuredProperty, ID++) ); // TEMP

        double existenceProbability;
        if ( iProperties < it_laserEntities.size() )
        {
            const ed::EntityConstPtr& e = * ( it_laserEntities[iProperties] );

            // check if new properties are measured.
            bool check1 = measuredProperty.getCircle().get_radius() != measuredProperty.getCircle().get_radius();
            bool check2 = measuredProperty.getRectangle().get_w() != measuredProperty.getRectangle().get_w();
            bool check3 = measuredProperty.getRectangle().get_d() != measuredProperty.getRectangle().get_d();

            if ( check1 || ( check2 && check3 ) )
            {
                // Clear unassociated entities in view
                const geo::Pose3D& pose = e->pose();
                // Transform to sensor frame
                geo::Vector3 p = sensor_pose.inverse() * pose.t;

                if ( !pointIsPresent ( p, lrf_model_, sensor_ranges ) )
                {
                    double p_exist = e->existenceProbability();
                    req.setExistenceProbability ( e->id(), std::max ( 0.0, p_exist - 0.1 ) ); // TODO: very ugly prob update
                }
                continue;
            }

            if ( !e->hasFlag ( "locked" ) )
            {
                entityProperties = e->property ( featureProperties_ );
                ed::tracking::Rectangle entityRectangle = entityProperties.getRectangle();
                ed::tracking::Circle entityCircle = entityProperties.getCircle();
                
                float Q = 0.4; // Measurement noise covariance. TODO: let it depend on if an object is partially occluded. Now, objects are assumed to be completely visible
                float R = 0.1; // Process noise covariance
                float RVariable = 1000000*R*pow(measuredProperties[iProperties].fittingErrorRectangle, 2.0);
                float largeCovariance = 100000.0;
                float mediumDimensionCovariance = 2.0;
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                
                // update rectangular properties
                Eigen::MatrixXf QmRectangle = Eigen::MatrixXf::Zero( 8, 8 );
                Eigen::MatrixXf RmRectangle = Eigen::MatrixXf::Zero( 5, 5 );
                
                QmRectangle.diagonal() << Q, Q, Q, 20*Q, 20*Q, 20*Q, Q, Q; // Covariance on state = [x, y, rot, x vel, y Vel, rot Vel, width, depth]; Q increases, more emphasis on measurements     
                RmRectangle.diagonal() << R, R, RVariable, R, R; // R decreases, more emphasis on measurements, info = [x, y, orient, width, depth]
                      
                // TODO what to do with position information if there is low confidence in with and depth? Position information should be updated with respect to an anchor point!
                // This is the problem of fusing multiple sensors

                if( !measuredProperties[iProperties].confidenceRectangleWidth && !measuredProperties[iProperties].confidenceRectangleDepth )
                {
                        // if there is uncertainty about the positions based on the dimensional check, first check if there are relevant features (corners!) which can give you a better estimation
                         bool checkCornerConfidence;
                        if( measuredProperties[iProperties].measuredCorners.size() > 0 )
                        {
                                checkCornerConfidence = true;
                        }
                        else
                        {
                                checkCornerConfidence = false;
                        }
                
                        if( checkCornerConfidence ) // we did not observe the entire entity, but apparently we have reliable features which can be used to update the position
                        {
                                // check which measured corners are reliable -> what are their corresponding coordinates?
                                // determine delta's for each reliable corner
                                // take the average delta
                                
                                std::vector<geo::Vec2f> cornersMeasured = measuredProperties[iProperties].measuredCorners; // are those properties determined in the same order for the model as well as the measurement
                                std::vector<geo::Vec2f> cornersModelled = entityProperties.getRectangle().determineCorners ( 0.0 );
                                
                                float deltaX = 0.0, deltaY = 0.0;
                                for(unsigned int iMeasured = 0; iMeasured < cornersMeasured.size(); iMeasured++)
                                {
                                        // for each corner which is measured, find the closest corner modelled
                                        float smallestDistance2 = std::numeric_limits<float>::infinity();
                                        unsigned int closestElement;
                                        
                                        for(unsigned int iModelled = 0; iModelled < cornersModelled.size(); iModelled++)
                                        {
                                                // find the clostes corner modelled
                                                float dist2 = pow( cornersMeasured[iMeasured].x - cornersModelled[iModelled].x, 2.0) + pow( cornersMeasured[iMeasured].y - cornersModelled[iModelled].y, 2.0);
                                                
                                                if( dist2 < smallestDistance2 )
                                                {
                                                        smallestDistance2 = dist2;
                                                        closestElement = iModelled;
                                                }
                                        }
                                        
                                        deltaX += cornersMeasured[iMeasured].x - cornersModelled[closestElement].x;
                                        deltaY += cornersMeasured[iMeasured].y - cornersModelled[closestElement].y;
                                }
                                
                                deltaX /= cornersMeasured.size();
                                deltaY /= cornersMeasured.size();
                                
                                ed::tracking::Rectangle updatedRectangle = measuredProperty.getRectangle(); // TODO correct? Should we take measured dimensions into account? TEST
                                
                                float updatedX = updatedRectangle.get_x() + deltaX;
                                float updatedY = updatedRectangle.get_y() + deltaY;
                                
                                updatedRectangle.set_x( updatedX );
                                updatedRectangle.set_y( updatedY );
                                
                                measuredProperty.setRectangle( updatedRectangle );                                
                                std::vector< geo::Vec2f> updatedRectangleCorners = updatedRectangle.determineCorners( 0.0 );
                        }

                        if( !checkCornerConfidence ) // No corner information obtained, but from one or 2 sides of the entity relevant position information is obtained. Set the corresponding covariances
                        {
                                // TODO DEBUG
                                // Measuring one side, but not the corners, gives accurate position information in one direction (the lateral direction of the 
                                // width is always measured, as that is the first part in the measured properties)
                                
                                float largePositionCovariance = 25.0; // TODO small covariance when the entity described does not reflect a part which is detected
                                float smallPositionCovariance = Q;
                                Eigen::MatrixXf C = Eigen::MatrixXf::Zero( 2, 2 ); // Covariance Matrix in the entity-coordinates
                                C.diagonal() << largePositionCovariance, largePositionCovariance;
                                
                                float rot = 0.0;
                                if( entityProperties.getRectangle().get_yaw() == entityProperties.getRectangle().get_yaw() )
                                {
                                        if( measuredProperty.getRectangle().get_yaw() != measuredProperty.getRectangle().get_yaw() )
                                        {
                                                rot = entityProperties.getRectangle().get_yaw();
                                        }
                                        else
                                        {
                                                rot = measuredProperty.getRectangle().get_yaw();
                                        }
                                }
                                
                                float angle = entityProperties.getRectangle().get_yaw(); //std::cout << "angle = " << angle << std::endl;
                                float ct = cos( angle );
                                float st = sin( angle );
                                
                                C( 1, 1) = smallPositionCovariance; // TODO right dim set??
                                        
                                if( measuredProperties[iProperties].methodRectangle == ed::tracking::RECTANGLE )
                                {
                                        C( 0, 0) = smallPositionCovariance;
                                }
                                
                                // Give certainties in terms of the world frame
                                Eigen::MatrixXf Rot = Eigen::MatrixXf::Zero( 2, 2 );
                                Rot << std::cos( rot ), -std::sin( rot ), std::sin( rot ),  std::cos( rot);
                        
                                RmRectangle.block< 2, 2 >( 0, 0 ) = Rot*C*Rot.transpose();
                        }
                }
                
                float modelledWidth, modelledDepth;
                bool switchedDimensions = entityProperties.getRectangle().switchDimensions( measuredProperty.getRectangle().get_yaw() ); // switching happens during update, this is just to compare the correct values
                
                if ( !switchedDimensions)
                {
                        modelledWidth = entityProperties.getRectangle().get_w();
                        modelledDepth = entityProperties.getRectangle().get_d();
                }
                else
                {
                        modelledWidth = entityProperties.getRectangle().get_d();
                        modelledDepth = entityProperties.getRectangle().get_w();
                }
                
                // set confidence for the dimensions
                if( measuredProperties[iProperties].confidenceRectangleWidth == false ) // when we do an update, we always measured in the width direction
                {
                        if( measuredProperty.getRectangle().get_w() > modelledWidth )
                        {
                                RmRectangle( 3, 3 ) = mediumDimensionCovariance; // as it it safer to estimate the dimensions too high than too low
                        }
                        else
                        {
                                RmRectangle( 3, 3 ) = largeCovariance;
                        }
                }
                        
                if( measuredProperties[iProperties].confidenceRectangleDepth == false )
                {                        
                          if( measuredProperty.getRectangle().get_d() > modelledDepth )
                          {
                                  RmRectangle( 4, 4 ) = mediumDimensionCovariance;
                          }
                          else
                          {
                                  RmRectangle( 4, 4 ) = largeCovariance;
                          }
                } 
               
                Eigen::VectorXf zmRectangle( 5 );
                zmRectangle <<  
                measuredProperty.getRectangle().get_x(),
                measuredProperty.getRectangle().get_y(), 
                measuredProperty.getRectangle().get_yaw(),  
                measuredProperty.getRectangle().get_w(), 
                measuredProperty.getRectangle().get_d();
                
                if (measuredProperty.getRectangle().get_yaw() != measuredProperty.getRectangle().get_yaw())
                {
                        // In case it is hard to determine the yaw-angle, which might be when the object is almost in line with the sensor, a NaN can be produced
                        zmRectangle(2) = entityRectangle.get_yaw();
                        QmRectangle(2, 2) = largeCovariance;
                }
                
                entityProperties.updateRectangleFeatures(QmRectangle, RmRectangle, zmRectangle, dt, sensor_pose);
               
                // update circular properties
                Eigen::MatrixXf QmCircle = Eigen::MatrixXf::Zero( 5, 5 );
                Eigen::MatrixXf RmCircle = Eigen::MatrixXf::Zero( 3, 3 );
                
                 QmCircle.diagonal() << Q, Q, Q, Q, Q; // xPos, yPos, xVel, yVel, radius
                RmCircle.diagonal() << R, R, R;
                
                // TODO what to do with position information if there is low confidence in with and depth? Position information should be updated with respect to an anchor point!
                if( measuredProperties[iProperties].confidenceRectangleDepth == false )
                {
                        QmCircle( 4, 4 ) = largeCovariance;
                } 

                Eigen::VectorXf zmCircle( 3 );
                zmCircle <<
                measuredProperty.getCircle().get_x(),
                measuredProperty.getCircle().get_y(),
                measuredProperty.getCircle().get_radius();

                ed::tracking::FeatureProbabilities measuredProb = measuredProperty.getFeatureProbabilities(); 

                entityProperties.updateCircleFeatures(QmCircle, RmCircle, zmCircle, dt);
                entityProperties.updateProbabilities ( measuredProb );
            }

            // Update existence probability
            double p_exist = e->existenceProbability();
            existenceProbability = std::min ( 1.0, p_exist + 0.1 ) ;// TODO: very ugly prob update
            id = e->id();

        }
        else
        {
            // create a new entity
                
            // Generate unique ID
            id = ed::Entity::generateID().str() + "-laserTracking";
            
            // Update existence probability
            existenceProbability = 1.0; // TODO magic number
            entityProperties = measuredProperty;
                     
            std::vector< bool > checks;
            checks.push_back( measuredProperty.getRectangle().get_x() != measuredProperty.getRectangle().get_x() );
            checks.push_back( measuredProperty.getRectangle().get_y() != measuredProperty.getRectangle().get_y() );
            checks.push_back( measuredProperty.getRectangle().get_z() != measuredProperty.getRectangle().get_z() );
            checks.push_back( measuredProperty.getFeatureProbabilities().get_pCircle() != measuredProperty.getFeatureProbabilities().get_pCircle() );
            checks.push_back( measuredProperty.getFeatureProbabilities().get_pRectangle() != measuredProperty.getFeatureProbabilities().get_pRectangle() );

//             std::cout << "checks = " << checks[0] << checks[1] << checks[2] << checks[3] << checks[4] << std::endl;
            
            bool all = false;
            std::vector<bool>::iterator it = checks.begin();
            
            while ( !all && it != checks.end() ) // Stops early if it hits a false
            {
                all &= *it;
                ++it;
            }

            if ( all )
            {
                ROS_FATAL  ( "Unvalid entity pub." );
                exit (EXIT_FAILURE);
            }

        }
        geo::Pose3D new_pose;

        if ( entityProperties.getFeatureProbabilities().get_pCircle() < entityProperties.getFeatureProbabilities().get_pRectangle() )
        {
            // determine corners
            ed::tracking::Rectangle rectangle = entityProperties.getRectangle();
            new_pose = rectangle.getPose();
        }
        else
        {
            // determine cilinder-properties
            ed::tracking::Circle circle = entityProperties.getCircle();
            new_pose = circle.getPose();
        }

        bool check = true;
        
        if ( new_pose.t.getX() != new_pose.t.getX() ||
                new_pose.t.getZ() != new_pose.t.getZ() ||
                new_pose.t.getY() != new_pose.t.getY() ||
                entityProperties.getFeatureProbabilities().get_pCircle() != entityProperties.getFeatureProbabilities().get_pCircle() ||
                entityProperties.getFeatureProbabilities().get_pRectangle() != entityProperties.getFeatureProbabilities().get_pRectangle()
           )
        {
            check = false;
        }

        // Set feature properties en publish geometries
        if ( check )
        {
                int nMeasurements = entityProperties.getNMeasurements() + 1;
                entityProperties.setNMeasurements(  nMeasurements );
                req.setProperty ( id, featureProperties_, entityProperties );
                req.setPose ( id, new_pose );
                
                // Set timestamp               
                req.setLastUpdateTimestamp ( id, scan->header.stamp.toSec() );
                req.setExistenceProbability ( id, existenceProbability );
        }
    }
           
    world_evidence.header.stamp =  scan->header.stamp;
    world_evidence.header.frame_id = "/map";
    
   // world_evidence_publisher_.publish( world_evidence );
    ObjectMarkers_pub_.publish(markers);

// - - - - - - - - - - - - - - - - -

//     std::cout << "tracking plugin: Total took " << t_total.getElapsedTimeInMilliSec() << " ms. \n\n\n" << std::endl;
    
//     std::cout << "End of tracking plugin" << std::endl;
}

// ----------------------------------------------------------------------------------------------------


void LaserPluginTracking::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    scan_buffer_.push(msg);
}

/*
void LaserPluginTracking::PoseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    pose_buffer_.push(msg);
}

void LaserPluginTracking::PoseWithCovarianceStampedInitCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    pose_buffer_init_.push(msg);
}
*/


ED_REGISTER_PLUGIN(LaserPluginTracking)
