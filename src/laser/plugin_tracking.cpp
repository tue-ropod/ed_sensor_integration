#include "plugin_tracking.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
//#include <ed/helpers/depth_data_processing.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ed/convex_hull_calc.h"

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>
#include <numeric>
#include <cmath>
#include <iterator>

#define DEBUG false // TEMP

namespace
{

typedef std::vector<unsigned int> ScanSegment;

// struct ScanSegmentInfo
// {
//     ScanSegment segmentRanges;
//     bool confidenceLeft;
//     bool confidenceRight;
// };

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
       std::vector<geo::Vec2f> measuredCorners;
};

visualization_msgs::Marker getMarker ( ed::tracking::FeatureProperties& featureProp, int ID) // TODO move to ed_rviz_plugins?
// ############################## TEMP ############################
{
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;

        color.r = 1;
        color.g = 140/255;
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
        
//         marker.header.stamp = ros::Time();

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




//        if (dm <= 0 && model_ranges[i]>0)
//        {
//            total_error += 0.3;
//            continue;
//        }

//        if (dm < model_ranges[i])
//            ++num_model_points;
//        else if(model_ranges[i]<=0 && dm>0 && dm<=7) //when there is no world model behind the door, output of world render without door is zero.
//        {                                              //only taking points with no world behind it, into account when nearby.
//            ++num_model_points;
//        }                                              // giving problems with fitting when door is not good in view

//        double diff = std::abs(ds - dm);
//        if (diff < 0.3)
//            total_error += diff;
//        else
//        {
//            if (ds > dm)
//                total_error += 1;
//            else
//                total_error += 0.3;
//        }
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
    //    if (best_pose != e.pose())
    //        std::cout<<"new_pose"<<std::endl;
    return best_pose;
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
    config.value("nominal_corridor_width", nominal_corridor_width_);
    

    int i_fit_entities = 0;
    config.value("fit_entities", i_fit_entities, tue::OPTIONAL);
    fit_entities_ = (i_fit_entities != 0);
    
    int i_check_door_status = 0;
    config.value("check_door_status", i_check_door_status, tue::OPTIONAL);
    check_door_status_ = (i_check_door_status != 0);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    unsigned int bufferSize = 1; // TODO increase to 3(?) in order to make it possible to process more laser data in 1 iteration. Set low for testing purposes now.
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, bufferSize, &LaserPluginTracking::scanCallback, this);
    door_pub_ = nh.advertise<ed_sensor_integration::doorDetection>("door", 3);
//     ObjectMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/objectMarkers2", 3 ); // TEMP
//     PointMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/pointMarkers", 3 ); // TEMP
//     processedLaserPoints_pub_ = nh.advertise<sensor_msgs::LaserScan> ( "processedLaserScan", 3 ); // TEMP

    tf_listener_ = new tf::TransformListener;
    
    // example given in ED/ed/examples/custom_properties. Update the probabilities using an update-request
    // TODO defined in multiple places now
    init.properties.registerProperty ( "Feature", featureProperties_, new FeaturPropertiesInfo ); 
    

    //pose_cache.clear();
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    cb_queue_.callAvailable();
    
//     std::cout << " Process started with buffer size of" << scan_buffer_.size() << std::endl;

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
                         const geo::Pose3D& sensor_pose, ed::UpdateRequest& req)
{
    if( DEBUG )
            std::cout << "Debug 1 \t";
    
    tue::Timer t_total;
    t_total.start();
    double current_time = ros::Time::now().toSec();
    
//      struct timeval now;
//         gettimeofday(&now, NULL);
// 
//          std::cout << "Start of plugin at t = " << now.tv_sec << "." << now.tv_usec << std::endl;
         
          std::cout << "Start of plugin" << std::endl;


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
    
//     sensor_msgs::LaserScan processedLRFScan = *scan;
//     processedLRFScan.ranges = sensor_ranges;
//     processedLaserPoints_pub_.publish( processedLRFScan );

    
//     if( DEBUG )
//             std::cout << "Debug 2 \t";

    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    std::vector<double> model_ranges(num_beams, 0);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
	
        if (e->shape() && e->has_pose() && !(e->hasType("left_door") || e->hasType("door_left") || e->hasType("right_door") || e->hasType("door_right" ) || e->hasFlag("non-localizable")))
        {
	//  std::cout << "Shape after = " << e->shape() << std::endl;
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());

            geo::LaserRangeFinder::RenderResult res(model_ranges);
            lrf_model_.render(opt, res);
        }        
    }

    // - - - - - - - - - - - - - - - - - -
    // Fit the doors
//     if( DEBUG )
//             std::cout << "Debug 3 \t";
    if (fit_entities_)
    {

        for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;

            if (!e->shape() || !e->has_pose())
                continue;

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
    
//     if( DEBUG )
//             std::cout << "Debug 4 \t";

//        // ############################## TEMP ############################
// 
//  for ( unsigned int iRanges = 0; iRanges < sensor_ranges.size(); ++iRanges )
//         {
//             unsigned int j = iRanges;
//             geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];
// 
//             // Transform to world frame
//             geo::Vector3 p = sensor_pose * p_sensor;
//             
//             std::cout <<  geo::Vec2f ( p.x, p.y ) << std::endl;
//         }
//             
//          // ############################## END TEMP ############################

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    std::vector<float> sensor_rangesOriginal = sensor_ranges;
//     std::vector<float> model_rangesOut; // TEMP
    
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];
//         model_rangesOut.push_back(rm);

        if (rs <= 0
                || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                || (std::abs(rm - rs) < world_association_distance_))
            sensor_ranges[i] = 0;
    }   
    
    // ############################## TEMP ############################
//     std::cout << "Sensor ranges = " << std::endl;
//     for(unsigned int ii = 0; ii < sensor_ranges.size(); ii++)
//     {
//             std::cout << sensor_ranges[ii] << "\t";
//     }
//     std::cout << "\n";
    // ############################## TEMP END ############################
    
    // - - - - - - - - - - - - - - - - - -
    // Segment the remaining points into clusters

//     if( DEBUG )
//             std::cout << "Debug 5 \t";
    
    std::vector<ScanSegment> segments;

    // Find first valid value
    ScanSegment current_segment;
//     ScanSegmentInfo currentSegmentInfo; // TODO remove for initial segments
//     bool confidenceLeft; // check if the object might have been covered by an object on both sides to determine the confidence of the measurement
//     bool confidenceRight;
    
    for ( unsigned int i = 0; i < num_beams - 1; ++i )
    {
        if ( sensor_ranges[i] > 0 )
        {
            current_segment.push_back(i);
            break;
        }
    }
    
//     if( DEBUG )
//             std::cout << "Debug 6 \t";

//     if ( currentSegmentInfo.segmentRanges.empty() )
    if ( current_segment.empty() )
    {
        return;
    }

//     if( DEBUG )
            std::cout << "Debug 7 \t";
    int gap_size = 0;
    std::vector<float> gapRanges;

//     for(unsigned int i = currentSegmentInfo.segmentRanges.front(); i < num_beams; ++i)
    for(unsigned int i = current_segment.front(); i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];

        if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segment_depth_threshold_ || i == num_beams - 1)
        {
            // Found a gap
            ++gap_size;
            gapRanges.push_back ( rs );

            if (gap_size >= max_gap_size_ || i == num_beams - 1)
            {
                 i = current_segment.back() + 1;

                if (current_segment.size()  >= min_segment_size_pixels_)
                {
                    // calculate bounding box
                    geo::Vec2 seg_min, seg_max;
                    for(unsigned int k = 0; k <  current_segment.size(); ++k)
                    {
                        geo::Vector3 p = lrf_model_.rayDirections()[ current_segment[k]] * sensor_ranges[current_segment[k]];

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
                    if ( ( bb .x > min_cluster_size_ || bb.y > min_cluster_size_ ) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_ )
                    {
                        segments.push_back ( current_segment );
                        
//                         std::cout << "New segment added. Ranges = "<< std::endl;
//                         for(unsigned int ii = 0; ii < current_segment.size(); ii++)
//                         {
//                                 std::cout << current_segment[ii] << "\t";
//                         }
//                         std::cout << "\n";
                        
                    }   
                }

                current_segment.clear();
                gapRanges.clear();

                // Find next good value
                while ( sensor_ranges[i] == 0 && i < num_beams )
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
    
//     if( DEBUG )
            std::cout << "Debug 8 \t";
    
    // Try to associate remaining laser points to specific entities
    std::vector<ed::WorldModel::const_iterator> it_laserEntities;
    std::vector< EntityProperty > EntityPropertiesForAssociation;

    // Check which entities might associate for tracking based on their latest location in order to prevent to check all the entities 
    for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it )
    {
        const ed::EntityConstPtr& e = *e_it;
//                      std::cout << "Going to check entity with id = " << e->id() << std::endl;
        std::string laserID = "-laserTracking";

        if ( e->id().str().length() < laserID.length() )
        {
            continue;
        }
        if ( e->id().str().substr ( e->id().str().length() - laserID.size() ) == laserID )  // entity described by laser before
        {
                /* // TODO where?
            bool check1 = e->existenceProbability() < 0.3 ;
            bool check2 = current_time - e->lastUpdateTimestamp() > entity_timeout_;

            if ( e->existenceProbability() < 0.3 || current_time - e->lastUpdateTimestamp() > entity_timeout_ ) // Criterea to remove entity which were created from this plugin
            {
                if ( !e->hasFlag ( "locked" ) )
                {
                    req.removeEntity ( e->id() );
                    continue;
                }
            }
            */

            it_laserEntities.push_back ( e_it );

            ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
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
    
//     std::cout << "laserEntities = " << std::endl;
//     for(unsigned int iTest = 0; iTest < it_laserEntities.size(); iTest++)
//     {
//              const ed::EntityConstPtr& e = *it_laserEntities[iTest];
//             std::cout << e->id() << ", " << EntityPropertiesForAssociation[iTest].entity_min.x  << ", " <<
//             EntityPropertiesForAssociation[iTest].entity_min.y  << ", " << 
//             EntityPropertiesForAssociation[iTest].entity_max.x  << ", " << 
//             EntityPropertiesForAssociation[iTest].entity_max.y  << ", " << std::endl;
//     }
    
//     if( DEBUG )
            std::cout << "Debug 9 \t";
    
    
//      std::vector< std::vector< geo::Vec2f > > associatedPointsList ( it_laserEntities.size() );
     std::vector< PointsInfo > associatedPointsInfo( it_laserEntities.size() );
//       std::vector< std::vector< int > > segmentsIDs ( segments.size() ); // IDs of the points in the laser-array
     
//      std::cout << "segments.size() = " << segments.size() << std::endl;
//     visualization_msgs::MarkerArray markerArrayPoints;
//     unsigned int IDPoints = 0;
    
    for ( unsigned int iSegments = 0; iSegments < segments.size(); ++iSegments )
    {
        // First, determine the properties of each segment
//         ScanSegment& segment = segments[iSegments].segmentRanges;
        ScanSegment& segment = segments[iSegments];
        unsigned int segment_size = segment.size();

        std::vector<geo::Vec2f> points ( segment_size );
        std::vector<unsigned int> segmentIDs ( segment_size );
        geo::Vec2f seg_min, seg_max;

//         if( DEBUG )
//                 std::cout << "Debug 10 \t";
        
//          std::cout << "Ranges in segment = "<< std::endl;
//                         for(unsigned int ii = 0; ii < segment_size; ii++)
//                         {
//                                 std::cout << segment[ii] << "\t";
//                         }
//                         std::cout << "\n";
        
//         std::cout << "Points in segment " << iSegments << " = " << std::endl;
        for ( unsigned int iSegment = 0; iSegment < segment_size; ++iSegment )
        {
            unsigned int j = segment[iSegment];
            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[iSegment] = geo::Vec2f ( p.x, p.y );
            segmentIDs[iSegment] = j;
            
//             std::cout << points[iSegment] << "\t ";
            
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
//         segmentsIDs[iSegments] = segmentIDs;
        
//         pubPoints(&markerArrayPoints, points, &IDPoints);
//         IDPoints++;
        
//         std::cout << "Segm min/mx = " <<   seg_min.x << ", " << 
//         seg_min.y << ", " << 
//         seg_max.x << ", " << 
//         seg_max.y << std::endl;

// if( DEBUG )
        std::cout << "Debug 11 \t";
    
        // After the properties of each segment are determined, check which clusters and entities might associate
        std::vector< int > possibleSegmentEntityAssociations;
//         std::cout << "it_laserEntities.size() = " << it_laserEntities.size() << std::endl;
        for ( unsigned int jj = 0; jj < it_laserEntities.size(); ++jj )
        {
//                  const ed::EntityConstPtr& e = *it_laserEntities[jj];
//                 std::cout << "Entity min/mx = " <<  EntityPropertiesForAssociation[jj].entity_min.x << ", " << 
//                 EntityPropertiesForAssociation[jj].entity_max.x << ", " <<
//                 EntityPropertiesForAssociation[jj].entity_min.y << ", " <<
//                 EntityPropertiesForAssociation[jj].entity_max.y << std::endl;
           
//    if( DEBUG )
           std::cout << "Debug 12 \t";
            // check if 1 of the extrema of the segment might be related to the exploded entity
            bool check1 =  seg_min.x > EntityPropertiesForAssociation[jj].entity_min.x && seg_min.x < EntityPropertiesForAssociation[jj].entity_max.x ;
            bool check2 =  seg_max.x > EntityPropertiesForAssociation[jj].entity_min.x && seg_max.x < EntityPropertiesForAssociation[jj].entity_max.x ;
            bool check3 =  seg_min.y > EntityPropertiesForAssociation[jj].entity_min.y && seg_min.y < EntityPropertiesForAssociation[jj].entity_max.y ;
            bool check4 =  seg_max.y > EntityPropertiesForAssociation[jj].entity_min.y && seg_max.y < EntityPropertiesForAssociation[jj].entity_max.y ;

            if ( check1 && check3 || check2 && check4 )
            {
                possibleSegmentEntityAssociations.push_back ( jj );
                const ed::EntityConstPtr& e = *it_laserEntities[jj];
//                  std::cout << "Possible association with entity " << e->id() << std::endl;
            }           
        }

//     if( DEBUG )
            std::cout << "Debug 13 \t";
        // If a cluster could be associated to a (set of) entities, determine for each point to which entitiy it belongs based on a shortest distance criterion. 
        // If the distance is too large, initiate a new entity
//         std::vector<geo::Vec2f> pointsNotAssociated;
        PointsInfo pointsNotAssociated;
        std::vector<float> distances ( points.size() );
        std::vector<unsigned int> IDs ( points.size() ); // IDs of the entity which is closest to that point

//             if( DEBUG )
            std::cout << "Debug 13.1 \t";
        
        for ( unsigned int i_points = 0; i_points < points.size(); ++i_points )  // Determine closest object and distance to this object. If distance too large, relate to new object
        {
//                     if( DEBUG )
//             std::cout << "Debug 13.2 \t";
            geo::Vec2f p = points[i_points];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = std::numeric_limits< unsigned int >::max();
//                 if( DEBUG )
//             std::cout << "Debug 13.3 \t";
            
            std::cout << "possibleSegmentEntityAssociations.size() = " << possibleSegmentEntityAssociations.size() << "\t";

            for ( unsigned int jj = 0; jj < possibleSegmentEntityAssociations.size(); ++jj )  // relevant entities only
            {
                        if( DEBUG )
            std::cout << "jj = " << jj << " \t";
                const ed::EntityConstPtr& e = *it_laserEntities[ possibleSegmentEntityAssociations[jj] ];
                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
                float dist;
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                    if( DEBUG )
            std::cout << "Debug 13.5 \t";
                if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() )  // entity is considered to be a circle
                {
                            if( DEBUG )
            std::cout << "Debug 13.6 \t";
                    ed::tracking::Circle circle = featureProperties.getCircle();  
                    circle.predictAndUpdatePos( dt ); // TODO Do this update once at initialisation
                    dist = std::abs ( std::sqrt ( std::pow ( p.x - circle.get_x(), 2.0 ) + std::pow ( p.y - circle.get_y(), 2.0 ) ) - circle.get_radius() ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                }
                else     // entity is considered to be a rectangle. Check if point is inside the rectangle
                {
                            if( DEBUG )
            std::cout << "Debug 13.7 \t";
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
    if( DEBUG )
            std::cout << "Debug 13.8 \t";
                    if ( OP_OC1 > 0 && OC1_OC1B > OP_OC1 && OP_OC3 > 0 && OC3_OC3 > OP_OC3 )   // point is inside the rectangle
                    {
                                if( DEBUG )
            std::cout << "Debug 13.9 \t";
                        std::vector<geo::Vec2f> p1Check = corners;

                        std::vector<geo::Vec2f> p2Check = corners; // place last element at begin
                        p2Check.insert ( p2Check.begin(), p2Check.back() );
                        p2Check.erase ( p2Check.end() );
   if( DEBUG )
            std::cout << "Debug 13.10 \t";
                        for ( unsigned int ii_dist = 0; ii_dist < p1Check.size(); ii_dist++ )
                        {

                                    if( DEBUG )
            std::cout << "Debug 13.10.1 \t";
                            float x1 = p1Check[ii_dist].x;
                            float x2 = p2Check[ii_dist].x;

                                                                if( DEBUG )
            std::cout << "Debug 13.10.2 \t";
                            float y1 = p1Check[ii_dist].y;
                            float y2 = p2Check[ii_dist].y;

                                                                if( DEBUG )
            std::cout << "Debug 13.10.3 \t";
                            float distance = std::abs ( ( y2 - y1 ) *p.x - ( x2 - x1 ) *p.y + x2*y1 -y2*x1 ) /std::sqrt ( std::pow ( y2-y1, 2.0 ) + std::pow ( x2-x1, 2.0 ) );
                            
                            if ( distance < minDistance )
                            {
                                minDistance = distance;
                            }
                        }
                    }
                    else     // point is outside the rectangle, https://stackoverflow.com/questions/44824512/how-to-find-the-closest-point-on-a-right-rectangular-prism-3d-rectangle/44824522#44824522
                    {
                                if( DEBUG )
            std::cout << "Debug 13.11 \t";
                        float tx = pCorrected.dot ( vx ) / ( vx.dot ( vx ) );
                        float ty = pCorrected.dot ( vy ) / ( vy.dot ( vy ) );

                        tx = tx < 0 ? 0 : tx > 1 ? 1 : tx;
                        ty = ty < 0 ? 0 : ty > 1 ? 1 : ty;

                        geo::Vec2f closestPoint = tx*vx + ty*vy + corners[0];

                        geo::Vec2f vector2Point = p - closestPoint;
                        minDistance = std::sqrt ( vector2Point.dot ( vector2Point ) );
                    }

                                                    if( DEBUG )
            std::cout << "Debug 13.11.1 \t";
                    dist = minDistance;
                }
                                                    if( DEBUG )
            std::cout << "Debug 13.11.2 \t";
                if ( dist < shortestDistance )
                {
                                                                            if( DEBUG )
            std::cout << "Debug 13.11.3 \t";
                    shortestDistance = dist;
                                                                        if( DEBUG )
            std::cout << "Debug 13.11.4 \t";
//             std::cout << "possibleSegmentEntityAssociations.size() = " << possibleSegmentEntityAssociations.size() << "jj = " << jj << std::endl;
//             std::cout << "id_shortestEntity = " << id_shortestEntity << std::endl;
                    id_shortestEntity = jj;
                }
            }
                                                    if( DEBUG )
            std::cout << "Debug 13.11.5 \t";
            distances[i_points] = shortestDistance;
                                                                if( DEBUG )
            std::cout << "Debug 13.11.6 \t";
            IDs[i_points] = id_shortestEntity;
        }
        
        std::cout << "End of loop debug 13.12.0 " << std::endl;
        
//         std::cout << "Shortest Distances = " ;
//         for(unsigned int i_points = 0; i_points < distances.size(); i_points++)
//         {
//                 std::cout << distances[i_points] << "\t ";
//         }
//         std::cout << "\n";
        
            if( DEBUG )
            std::cout << "Debug 13.12 \t";
        unsigned int IDtoCheck = IDs[0];
        unsigned int firstElement = 0;
        bool previousSegmentAssociated = false;
        
//         std::cout << "possibleSegmentEntityAssociations = " << std::endl;
//         for (unsigned int iTest = 0; iTest < possibleSegmentEntityAssociations.size(); iTest++)
//         {
//                 std::cout << possibleSegmentEntityAssociations[iTest] << "\t" << std::endl;
//         }
//         std::cout << "\n";

        // check groups of segments associated to a specific entity
        for ( unsigned int iDistances = 1; iDistances < distances.size(); iDistances++ )
        {
                    if( DEBUG )
            std::cout << "Debug 13.13 \t";
            if ( IDs[iDistances] == IDtoCheck && iDistances != distances.size() - 1 ) // ID similar and not at final reading, check if next element is similar or not
            {
                    if( DEBUG )
                    std::cout << "Debug 13.13.0 \t";
                continue;
            }
            
            if( DEBUG )
            std::cout << "Debug 13.13.1 \t";

            unsigned int length = iDistances - firstElement;
           // bool test = length < min_segment_size_pixels_ ;

            if ( length >= min_segment_size_pixels_ )
            {
// //                         if( DEBUG )
            std::cout << "Debug 13.14 \t";
                float minDistance = distances[firstElement];
//                 if( DEBUG )
                std::cout << "Debug 13.14.0 \t";
                for ( unsigned int iiDistances = 1; iiDistances < iDistances; iiDistances++ )
                {
                    if ( distances[iiDistances] < minDistance )
                    {
                        minDistance = distances[iiDistances];
                    }
                }
                
//                 if( DEBUG )
                std::cout << "Debug 13.14.1 \t";
                
//                 std::cout << "Debug 13.15 \t";
                bool associated = minDistance < MIN_ASSOCIATION_DISTANCE;
                
//                 std::cout << "associated, previousSegmentAssociated, firstElement" << associated << previousSegmentAssociated << firstElement<< std::endl;
                
                if( associated && !previousSegmentAssociated && firstElement > 0 ) // check for possibility to reassociate previous section
                {
                        if( DEBUG )
                        std::cout << "Debug 13.16 \t";
                         geo::Vec2f lastPointPreviousSegment = points[firstElement - 1];
                         if( DEBUG )
                         std::cout << "Debug 13.17 \t";
                        geo::Vec2f firstPointCurrentSegment = points[firstElement];
//                         if( DEBUG )
                        std::cout << "Debug 13.18 \t";
                        float interSegmentDistance = std::sqrt( std::pow(lastPointPreviousSegment.x-firstPointCurrentSegment.x, 2.0) + std::pow(lastPointPreviousSegment.y-firstPointCurrentSegment.y, 2.0) );

//                         std::cout << termcolor::cyan << "Try to reassociate previous section: interSegmentDistance = " << interSegmentDistance << termcolor::reset << std::endl;;
                        
//                         std::cout << "Points.size() = " << points.size() << " firstElement = " << firstElement << " lastPointPreviousSegment = " << lastPointPreviousSegment;
//                         std::cout << " firstPointCurrentSegment = " << firstPointCurrentSegment << " Test = " << associatedPointsInfo.back().points.back() << std::endl;
                        
//                         std::cout << "Points = " << std::endl;
//                         for(unsigned int iPointsPrint = 0; iPointsPrint < points.size(); iPointsPrint++)
//                         {
//                                 std::cout << points[iPointsPrint] << "\t";
//                         }
//                         std::cout << "\n";
                        
                        if( interSegmentDistance < MIN_ASSOCIATION_DISTANCE_SEGMENTS)  // reassociate previous section
                        {
                                
//                                 std::cout << termcolor::cyan << "reassociate previous section" << termcolor::reset << std::endl;;
                                
//                                 std::cout << "Debug 13.19 \t";
                                previousSegmentAssociated = true;
//                                 std::cout << "Debug 13.19.1 \t";
                                unsigned int previousID = IDs[iDistances - 1];
//                                 std::cout << "Debug 13.20 \t";
                                
                                const ed::EntityConstPtr& e1 = *it_laserEntities[ possibleSegmentEntityAssociations[IDtoCheck] ];
                                const ed::EntityConstPtr& e2 = *it_laserEntities[ possibleSegmentEntityAssociations[previousID] ];                                                                
//                                 std::cout << "Entity IDs = " << e1->id() << ", " << e2->id() << std::endl;
                                
                                append(associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points, associatedPointsInfo.back().points);
                                append(associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs, associatedPointsInfo.back().laserIDs);
//                                 std::cout << "Debug 13.21 \t";
                                
                                
                                if( associatedPointsInfo.size() > it_laserEntities.size() ) // Unassociated points were added
                                {
                                        associatedPointsInfo.erase ( associatedPointsInfo.end() );
//                                                                 std::cout << "Debug 13.22 \t";
//                                                                 std::cout << termcolor::magenta << "associatedPointsInfo.erase ( associatedPointsInfo.end() )" << termcolor::reset << std::endl;
                                }
                                else // keep the possibility to associate points to a specific entity
                                {
                                        associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points.clear();
                                        associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs.clear();
                                }
                                
                        }        
//                          std::cout << "Debug 13.23 \t";
                     
                }
                
            std::cout << "Debug 13.18.1 \t";
//             std::cout << "2: associated, previousSegmentAssociated" << associated << previousSegmentAssociated << std::endl;
            
                if ( !associated && previousSegmentAssociated) 
                {
//                         std::cout << "Debug 13.22 \t";
                        // boundaries for distance and that those boundaries are associated to a object                                
                        geo::Vec2f lastPointPreviousSegment = points[firstElement - 1];
//                         std::cout << "Debug 13.23 \t";
                        geo::Vec2f firstPointCurrentSegment = points[firstElement];
//                         std::cout << "Debug 13.24 \t";
                        
                        float interSegmentDistance = std::sqrt( std::pow(lastPointPreviousSegment.x-firstPointCurrentSegment.x, 2.0) + std::pow(lastPointPreviousSegment.y-firstPointCurrentSegment.y, 2.0) );
                        
//                         std::cout << "!associated && previousSegmentAssociated: interSegmentDistance = " << interSegmentDistance << std::endl;
                        
//                         std::cout << "Debug 13.25 \t";
                        if( interSegmentDistance < MIN_ASSOCIATION_DISTANCE_SEGMENTS)
                        {
//                                 std::cout << "Debug 13.26 \t";
                                
//                                 std::cout << "associated set to true in --!associated && previousSegmentAssociated-- -section"  << std::endl;
                                
                                associated = true;
                                unsigned int previousID = IDs[iDistances - 1];
//                                 std::cout << "Debug 13.27 \t";
                                IDtoCheck = previousID;
                        }           
                }

                std::cout << "Debug 13.18.2 \t";
//                 std::cout << "3: associated, previousSegmentAssociated" << associated << previousSegmentAssociated << std::endl;
                if ( associated )  // add all points to associated entity
                {
                        previousSegmentAssociated = true;
//                             if( DEBUG )
            std::cout << "Debug 13.15 \t";
                    const ed::EntityConstPtr& entityToTest = *it_laserEntities[ possibleSegmentEntityAssociations[IDtoCheck] ];
            std::cout << "iDistances = " << iDistances << " points.size() = " << points.size() << " segmentIDs.size() = " << segmentIDs.size() << std::endl;
            std::cout << "IDtoCheck = " << IDtoCheck << " firstElement = " << firstElement << " iDistances = " << iDistances << std::endl;
            std::cout << "associatedPointsInfo.size() = " << associatedPointsInfo.size() << ", possibleSegmentEntityAssociations.size() = " << possibleSegmentEntityAssociations.size() << std::endl;
            std::cout << "possibleSegmentEntityAssociations[IDtoCheck] = " << possibleSegmentEntityAssociations[IDtoCheck] << std::endl;
            
            append( associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points, points, firstElement, iDistances );
            std::cout << "Debug 13.15.0 \t";
            append( associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs, segmentIDs, firstElement, iDistances );
            
//              std::cout << "Debug 13.15.1 \t";
//                     for ( unsigned int i_points = firstElement; i_points <= iDistances; ++i_points )
//                     {
// //                             std::cout << "points[i_points] = " << points[i_points] << " segmentIDs[i_points] = " << segmentIDs[i_points] << std::endl;
// //                         associatedPointsList.at ( possibleSegmentEntityAssociations[IDtoCheck] ).push_back ( points[i_points] );
//                         associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points.push_back ( points[i_points] );
//                         associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).laserIDs.push_back ( segmentIDs[i_points] );
//                     }
            
            std::cout << "Debug 13.15.1 \t";
                }
                else
                {
//                             if( DEBUG )
            std::cout << "Debug 13.16 \t";
//                     pointsNotAssociated.clear();
            
            previousSegmentAssociated = false;
            pointsNotAssociated = PointsInfo();
            
                    for ( unsigned int i_points = firstElement; i_points < iDistances; ++i_points )
                    {
                        pointsNotAssociated.points.push_back ( points[i_points] );
                        pointsNotAssociated.laserIDs.push_back ( segmentIDs[i_points] );
                    }
//                     associatedPointsList.push_back ( pointsNotAssociated );
                        associatedPointsInfo.push_back ( pointsNotAssociated );
                        
//                         std::cout << termcolor::magenta << "AssociatedPointsInfo extended" << termcolor::reset << std::endl;
                }
                std::cout << "Debug 13.16.-1 \t";
            }
            
            firstElement = iDistances;
            IDtoCheck = IDs[iDistances];
        }

    }
    
    
     // ############################## TEMP ############################
    
//     unsigned int IDPoints = 0;
    
//     Print all points associated to a specific entity
//     for(unsigned int iTest = 0; iTest < associatedPointsInfo.size(); iTest++)
//     {
//             
//             if( iTest < it_laserEntities.size() )
//             {
//             const ed::EntityConstPtr& e = *it_laserEntities[ iTest ];
//             std::cout << "Points associated to entity " << termcolor::blue << e->id() << termcolor::reset << " are \n";
//             }
//             else
//             {
//                     std::cout << termcolor::red << "Points associated to new entity = \n "  << termcolor::reset;
//             }
//             std::cout << "associatedPointsList.size() = " << associatedPointsInfo.size() << std::endl;
            
//             std::vector<geo::Vec2f> pointsToPrint  = associatedPointsInfo[iTest].points ;
//             pubPoints(&markerArrayPoints, pointsToPrint, &IDPoints);
//             IDPoints++;
            
//             for( unsigned int iTest1 = 0; iTest1 < pointsToPrint.size(); iTest1++)
//             {
//                     std::cout << pointsToPrint[iTest1] << "\t";
//             }
            
//             std::vector<unsigned int> laserIDsToPrint = associatedPointsInfo[iTest].laserIDs;

//             std::cout << "LaserIds = " << std::endl;
//             for( unsigned int iTest1 = 0; iTest1 < laserIDsToPrint.size(); iTest1++)
//             {
//                     std::cout << laserIDsToPrint[iTest1] << "\t";
//             }
//             
//              std::cout << "LaserRanges = " << std::endl;
//             for( unsigned int iTest1 = 0; iTest1 < laserIDsToPrint.size(); iTest1++)
//             {
//                     std::cout << sensor_ranges[laserIDsToPrint[iTest1]] << "\t";
//             }
            
//             std::cout << "\n\n ";
//     }
    
 // ############################## TEMP END ############################
    
 
    
    
//     if( DEBUG )
            std::cout << "Debug 14 \t";
    
    // TODO check at which point the segment should be splitted
    
//    std::vector<ed::tracking::FeatureProperties> measuredProperties;
//     std::vector<ed::tracking::FeatureProperties> measuredProperties ( associatedPointsInfo.size() ); // The first sequence in this vector (with the length of laser entitities) are the properties corresponding to existing entities
    std::vector<measuredPropertyInfo> measuredProperties ( associatedPointsInfo.size() ); // The first sequence in this vector (with the length of laser entitities) are the properties corresponding to existing entities
    
//     std::cout << "associatedPointsList.size() = " << associatedPointsList.size() << std::endl;
//     visualization_msgs::MarkerArray markerArrayPoints;
    
//     std::vector<bool> propertiesDescribed( associatedPointsInfo.size(), false );
//     unsigned int IDPoints = 0;
   for ( unsigned int iList = 0; iList < associatedPointsInfo.size(); iList++ )
//     for ( std::vector<ScanSegmentInfo>::const_iterator it = segments.begin(); it != segments.end(); ++it )
    {
            measuredProperties[iList].propertiesDescribed = false;
//         const ScanSegmentInfo& segment = *it; // TODO make it a scansegment with the info of confidence high and low
        //std::vector<geo::Vec2f> points ( segment.segmentRanges.size() );
       std::vector<geo::Vec2f> points  = associatedPointsInfo[iList].points ;
       
//        std::cout << "points.size() = " << points.size() <<  std::endl;
       
       if( points.size() == 0 )
               continue;
       
//        pubPoints(&markerArrayPoints, points, &IDPoints);
//         IDPoints++;
       
//         for ( unsigned int i = 0; i < points.size(); ++i ) // TODO points computed again?
//         {
//             unsigned int j = segment.segmentRanges[i];
//             geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];
// 
//             // Transform to world frame
//             geo::Vector3 p = sensor_pose * p_sensor;
// 
//             // Add to cv array
//             points[i] = geo::Vec2f ( p.x, p.y );            
//         }
       
      
        
//        if( DEBUG )
               std::cout << "Debug 15 \t";
       
        std::vector<unsigned int> cornerIndices;
        std::vector<geo::Vec2f>::iterator it_start = points.begin();
        std::vector<geo::Vec2f>::iterator it_end = points.end();
        unsigned int cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
          
        if( ed::tracking::findPossibleCorner ( points, &cornerIndices, &it_start, &it_end ) )
        {
                cornerIndex = cornerIndices[0];
        }
        
        for ( std::vector<unsigned int>::const_iterator it_in = cornerIndices.begin(); it_in != cornerIndices.end(); ++it_in )
        {
             const unsigned int& index = *it_in;
        }
        
//         if( DEBUG )
                std::cout << "Debug 16 \t";

        ed::tracking::Circle circle;   
        ed::tracking::Rectangle rectangle;    
        std::vector<geo::Vec2f>::iterator it_low, it_high;
        
        ed::tracking::FITTINGMETHOD method = ed::tracking::CIRCLE;
        float error_circle2 = ed::tracking::fitObject ( points, method, &cornerIndex, &rectangle, &circle, &it_low, &it_high, sensor_pose );
        unsigned int elementLow = associatedPointsInfo[iList].laserIDs[0];
        unsigned int elementHigh = associatedPointsInfo[iList].laserIDs.back();
//         measuredProperties[iList].confidenceCircle = ed::tracking::determineSegmentConfidence ( scan, elementLow, elementHigh);

        method = ed::tracking::determineCase ( points, &cornerIndex, &it_low, &it_high, sensor_pose ); // chose to fit a single line or a rectangle (2 lines)        
        float error_rectangle2 = ed::tracking::fitObject ( points, method,  &cornerIndex, &rectangle, &circle, &it_low, &it_high,  sensor_pose );

        measuredProperties[iList].confidenceRectangleWidth = false;
        measuredProperties[iList].confidenceRectangleWidthLow = false;
        measuredProperties[iList].confidenceRectangleWidthHigh = false;
        measuredProperties[iList].confidenceRectangleDepth = false;
        measuredProperties[iList].confidenceRectangleDepthLow = false;
        measuredProperties[iList].confidenceRectangleDepthHigh = false;
        measuredProperties[iList].methodRectangle = method;
                
//         if( iList < it_laserEntities.size() )
//         {
//              const ed::EntityConstPtr& eTest = *it_laserEntities[ iList ];
//              std::cout << "For points associated to entity " << eTest->id();
//         }
//         else
//         {
//                 std::cout << " For points associated to a NEW entity ";
//         }
        
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
        
//         std::cout << " IDLowWidth = " << IDLowWidth << " IDHighWidth = " << IDHighWidth << " IDLowDepth = " << IDLowDepth << " IDHighDepth = " << IDHighDepth << std::endl;
        
                        std::cout << "Debug 16.0 \t";
        for(unsigned int iConfidence = 0; iConfidence < 2; iConfidence++) // Determine for both with and depth
        {
                bool determineWidthConfidence = false, determineDepthConfidence = false; 
                
                if(iConfidence == 0)
                {
                        determineWidthConfidence = true;
                        elementLow = IDLowWidth;
                        elementHigh = IDHighWidth;
                        
//                         std::cout << " iConfidence = " << iConfidence << " elementLow = " << elementLow << " elementHigh = " << elementHigh << std::endl;
                } 
                
                if(iConfidence == 1)
                {
                        determineDepthConfidence = true;
                        elementLow = IDLowDepth; // cause corner belongs to both points. TODO what if the corner is occluded? Does this automatically lead to low confidences? -> TEST
                        elementHigh = IDHighDepth;
//                         std::cout << " iConfidence = " << iConfidence << " elementLow = " << elementLow << " elementHigh = " << elementHigh << std::endl;
                }
                
//                 std::cout << "elementLow = " << elementLow << ", elementHigh = " << elementHigh << ", determineWidthConfidence = " << determineWidthConfidence << ", determineDepthConfidence = " << determineDepthConfidence<< std::endl;
                
                if(elementLow == 0 && elementHigh == 0) // skip if no relevant information can be obtained
                        continue;
                
                 float avgAngle = 0.0;
                 for(unsigned int iiAssPoints = elementLow; iiAssPoints < elementHigh; iiAssPoints++)
                 {
//                          unsigned int element = associatedPointsInfo[iList].laserIDs[iiAssPoints];
                         avgAngle +=  LRF_yaw + lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*iiAssPoints;
                 }         
                 avgAngle /= (elementHigh - elementLow);
                 
//                   std::cout << " LRF_yaw, lrf_model_.getAngleMin(),  lrf_model_.getAngleIncrement(), elementHigh, elementLow = " ;
//                  std::cout << LRF_yaw << ", " << lrf_model_.getAngleMin() << ", " << lrf_model_.getAngleIncrement()<< " , " << elementHigh << ", " << elementLow << std::endl;
                 
                 if(determineWidthConfidence)
                 {
                         ed::tracking::unwrap (&avgAngle, rectangle.get_yaw(), M_PI);
                 }
                 else if (determineDepthConfidence)
                 {
                          ed::tracking::unwrap (&avgAngle, rectangle.get_yaw() + M_PI_2, M_PI);
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
//                           std::cout << "Statement = false"  << std::endl;
                         if(determineWidthConfidence)
                         {
                                 measuredProperties[iList].confidenceRectangleWidthLow = ed::tracking::determineCornerConfidence ( scan, elementLow, true);
                                 measuredProperties[iList].confidenceRectangleWidthHigh = ed::tracking::determineCornerConfidence ( scan, elementHigh, false); 
                                 measuredProperties[iList].confidenceRectangleWidth = (measuredProperties[iList].confidenceRectangleWidthLow && measuredProperties[iList].confidenceRectangleWidthHigh );
                                 
                                 if( measuredProperties[iList].confidenceRectangleWidthLow )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDLowWidth] );  
//                                          std::cout << "Point added 0 = " << associatedPointsInfo[iList].points[PointIDLowWidth] << std::endl;
                                 }
                                 
                                 if(  measuredProperties[iList].confidenceRectangleWidthHigh )
                                 {
                                         measuredProperties[iList].measuredCorners.push_back( associatedPointsInfo[iList].points[PointIDHighWidth] );  
//                                          std::cout << "Point added 1 = " << associatedPointsInfo[iList].points[PointIDHighWidth] << std::endl;
                                         
                                         std::cout << "associatedPointsInfo[iList].points.size() = " << associatedPointsInfo[iList].points.size() << ". PointIDHighWidth = " << PointIDHighWidth << " Points = " << std::endl;
                                         for(unsigned int iPointsTest = 0; iPointsTest < associatedPointsInfo[iList].points.size(); iPointsTest++)
                                         {
                                                std::cout << associatedPointsInfo[iList].points[iPointsTest] << "\t";
                                         }
                                         
                                         
                                         
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
        
                    std::cout << "Debug 16.1 \t";
         
        ed::tracking::FeatureProbabilities prob;
        if ( prob.setMeasurementProbabilities ( error_rectangle2, error_circle2, 2*circle.get_radius() , nominal_corridor_width_ ) )
        {

            ed::tracking::FeatureProperties properties;
            properties.setFeatureProbabilities ( prob );
            properties.setCircle ( circle );
            properties.setRectangle ( rectangle );

            if ( rectangle.get_x() != rectangle.get_x() )
            {
                ROS_WARN ( "Rectangle: invalid properties set" );
//                 rectangle.printProperties();
//                 std::cout << "Prob = " << prob.get_pCircle() << ", " << prob.get_pRectangle() << std::endl;
//                 std::cout << "Method = " << method << std::endl;
//                 std::cout << "Errors = " << error_rectangle2 << ", " << error_circle2 << std::endl;
            }

            measuredProperties[iList].featureProperty =  properties ;
            measuredProperties[iList].propertiesDescribed = true;
            
             if( iList < it_laserEntities.size() )
            {
                const ed::EntityConstPtr& e = *it_laserEntities[ iList ];
//                 std::cout << "New measurement of entity " << termcolor::blue << e->id()  << termcolor::reset << " is \n";
            }
            else
            {
//                 std::cout << "Measured properties of new entity equals" << std::endl;
            }
//             properties.printProperties();
        }
        
    }  
    
//    PointMarkers_pub_.publish( markerArrayPoints );
       
//    if( DEBUG )
            std::cout << "Debug 17 \t";
    
    unsigned int marker_ID = 0; // To Do: After tracking, the right ID's should be created. The ID's are used to have multiple markers.

    ed::tracking::FeatureProperties measuredProperty, entityProperties; // Measured properties and updated properties
    ed::UUID id;
//     std::vector<ed::WorldModel::const_iterator> it_laserEntities; 

//     if( DEBUG )
            std::cout << "Debug 18 \t";
    
//     visualization_msgs::MarkerArray markers;
//     unsigned int ID = 100;
    
    for ( unsigned int iProperties = 0; iProperties < measuredProperties.size(); iProperties++ ) // Update associated entities
    {
                if( DEBUG )
            std::cout << "Debug 18.1 \t";
            if(!measuredProperties[iProperties].propertiesDescribed ) 
                    continue;
            
                if( DEBUG )
            std::cout << "Debug 18.2 \t";
            
        measuredProperty = measuredProperties[iProperties].featureProperty;
        
        // temporary: pub measured properties in order to visualize the properties which are added
//         markers.markers.push_back( getMarker(measuredProperty, ID++) ); // TEMP

        
//         measuredProperty.getRectangle().printProperties();
//         measuredProperty.getCircle().printProperties();
//         std::cout << "prob = " << measuredProperty.getFeatureProbabilities().get_pCircle() << ", " << measuredProperty.getFeatureProbabilities().get_pRectangle() << std::endl;

            if( DEBUG )
            std::cout << "Debug 18.3 \t";
        double existenceProbability;
        if ( iProperties < it_laserEntities.size() )
        {
            const ed::EntityConstPtr& e = * ( it_laserEntities[iProperties] );
//             std::cout << "Going to update entity " << e->id() << std::endl;

            // check if new properties are measured.
            bool check1 = measuredProperty.getCircle().get_radius() != measuredProperty.getCircle().get_radius();
            bool check2 = measuredProperty.getRectangle().get_w() != measuredProperty.getRectangle().get_w();
            bool check3 = measuredProperty.getRectangle().get_d() != measuredProperty.getRectangle().get_d();


            if ( check1 || ( check2 && check3 ) )
            {
                //Clear unassociated entities in view
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
            
            if( DEBUG )
                    std::cout << "Debug 1 " << std::endl;

            if ( !e->hasFlag ( "locked" ) )
            {
                entityProperties = e->property ( featureProperties_ );
                ed::tracking::Rectangle entityRectangle = entityProperties.getRectangle();
                ed::tracking::Circle entityCircle = entityProperties.getCircle();
                
                std::cout << "\nFor entity " << termcolor::blue << e->id() << termcolor::reset << std::endl;
                
                float Q = 0.1; // Measurement noise covariance. TODO: let it depend on if an object is partially occluded. Now, objects are assumed to be completely visible
                float R = 0.2; // Process noise covariance
                float largeCovariance = 1000.0;
                float mediumDimensionCovariance = 2.0;
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                
                if( DEBUG )
                        std::cout << "Test 1  \t";
                
                // update rectangular properties
                Eigen::MatrixXf QmRectangle = Eigen::MatrixXf::Zero( 8, 8 );
                Eigen::MatrixXf RmRectangle = Eigen::MatrixXf::Zero( 5, 5 );
                
                if( DEBUG )
                        std::cout << "Test 2 \t";
                QmRectangle.diagonal() << Q, Q, Q, Q, Q, Q, Q, Q;
                RmRectangle.diagonal() << R, R, R, R, R;
                
//                 std::cout << "Confidence width = " << measuredProperties[iProperties].confidenceRectangleWidth << ", confidence depth = " << measuredProperties[iProperties].confidenceRectangleDepth << std::endl;
                
                
                // TODO what to do with position information if there is low confidence in with and depth? Position information should be updated with respect to an anchor point!
                
                // Determine position accuracy of the measurements
                // Check if there is confidence about certain corner-points
                
                
                // else, check if information can be obtained in local coordinates dus to a measurement of the side
                // when method is LINE or RECTANGLE, in both cases the width information is known
                
//                 Eigen::MatrixXf RmRectanglePos = Eigen::MatrixXf::Zero( 2, 2 ); // state = [x, y]^T
//                 float largePosCovariance = 2.0;
//                 
//                 RmRectanglePos( 0, 0 ) = largePosCovariance; // certainty in y, so uncertainty in x direction
//                 
//                 if( measuredProperties[iProperties].methodRectangle == ed::tracking::RECTANGLE ) //  depth as well
//                 {
//                         RmRectanglePos( 1, 1 ) = largePosCovariance; // certainty in y, so uncertainty in x direction
//                 }
//                 
                
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
                
                
                        // TODO Do we need to store all these features of the rectangles?
                        
        //                 if (!measuredProperties[iProperties].confidenceRectangleWidthLow &&
        //                     !measuredProperties[iProperties].confidenceRectangleDepthHigh && (
        //                         !measuredProperties[iProperties].confidenceRectangleWidthHigh ||
        //                         !measuredProperties[iProperties].confidenceRectangleDepthLow )
        //                 )
        //                 { // cornerpoints give are an accurate source of the position of an object, so make use of this property!
        //                         
        //                         
        //                         // TODO Neighrest neighbour association of cornerpoints -> still necessary? 
        //                 }
                        
                        std::cout << "measuredProperties = " << 
                        measuredProperties[iProperties].confidenceRectangleWidthLow <<
                        measuredProperties[iProperties].confidenceRectangleWidthHigh <<
                        measuredProperties[iProperties].confidenceRectangleDepthLow <<
                        measuredProperties[iProperties].confidenceRectangleDepthHigh << std::endl;
        //                 
        //                 std::cout << "checkCornerConfidence = " << 
        //                 checkCornerConfidence << std::endl;

                        

                        if( checkCornerConfidence ) // we did not observe the entire entity, but apparently we have reliable features which can be used to update the position
                        {
                                // check which measured corners are reliable -> what are their corresponding coordinates?
                                // determine delta's for each reliable corner
                                // take the average delta
                                
//                                 std::cout << "Update based on features -> cornerpoints" << std::endl;
                                
                                std::vector<geo::Vec2f> cornersMeasured = measuredProperties[iProperties].measuredCorners; // are those properties determined in the same order for the model as well as the measurement
                                std::vector<geo::Vec2f> cornersModelled = entityProperties.getRectangle().determineCorners ( 0.0 );
                                
//                                 std::cout << "CornersMeasured = " << std::endl;
//                                 for( unsigned int iPrint = 0; iPrint < cornersMeasured.size(); iPrint++ )
//                                 {
//                                         std::cout << cornersMeasured[iPrint] << "\t";
//                                 }
//                                 std::cout << "\n";
                                
//                                 std::cout << "cornersModelled = " << std::endl;
//                                 for( unsigned int iPrint = 0; iPrint < cornersModelled.size(); iPrint++ )
//                                 {
//                                         std::cout << cornersModelled[iPrint] << "\t";
//                                 }
//                                 std::cout << "\n";
                                
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
                                        
//                                         std::cout << "smallestDistance2 = " << smallestDistance2 << std::endl;
//                                         std::cout << "cornersMeasured[iMeasured] = " << cornersMeasured[iMeasured] << " cornersModelled[closestElement] = " << cornersModelled[closestElement] << std::endl;
                                        
                                        deltaX += cornersMeasured[iMeasured].x - cornersModelled[closestElement].x;
                                        deltaY += cornersMeasured[iMeasured].y - cornersModelled[closestElement].y;
                                        
//                                         std::cout << "In loop: deltaX, deltaY = " << deltaX << ", " << deltaY << std::endl;
                                }
                                
                                deltaX /= cornersMeasured.size();
                                deltaY /= cornersMeasured.size();
                                
//                                 std::cout << "deltaX, deltaY = " << deltaX << ", " << deltaY << std::endl;
                                
                                ed::tracking::Rectangle updatedRectangle = measuredProperty.getRectangle(); // TODO correct? Should we take measured dimensions into account? TEST
                                
//                                 std::cout << "Measured Properties = " << updatedRectangle.get_x() << ", "  << updatedRectangle.get_y() << std::endl;
//                                 std::cout << "Meas. Properties +D = " << updatedRectangle.get_x() + deltaX << ", "  << updatedRectangle.get_y() + deltaY << std::endl;
                                
                                float updatedX = updatedRectangle.get_x() + deltaX;
                                float updatedY = updatedRectangle.get_y() + deltaY;
                                
//                                 std::cout << "Meas. Properties up = " << updatedX << ", "  << updatedY << std::endl;
                                
                                updatedRectangle.set_x( updatedX );
                                updatedRectangle.set_y( updatedY );
                                
                                measuredProperty.setRectangle( updatedRectangle );
                                
//                                 std::cout << "Updated Properties = " << updatedRectangle.get_x() << ", "  << updatedRectangle.get_y() << std::endl;
                                
                                std::vector< geo::Vec2f> updatedRectangleCorners = updatedRectangle.determineCorners( 0.0 );
                                
//                                 std::cout << "updatedRectangleCorners = ";
//                                 for(unsigned int iTestCorners = 0; iTestCorners < updatedRectangleCorners.size(); iTestCorners++)
//                                 {
//                                         std::cout << updatedRectangleCorners[iTestCorners];
//                                 }
//                                 std::cout << "\n" ;
                                
                                
                        }

                        
                        
                        if( !checkCornerConfidence ) // No corner information obtained, but from one or 2 sides of the entity relevant position information is obtained. Set the corresponding covariances
                        {
                                // TODO DEBUG
                                // Measuring one side, but not the corners, gives accurate position information in one direction, namely the lateral direction of the 
                                // width is always measured, as that is the first part in the measured properties
                                
//                                 std::cout << "Update based on position of sides" << std::endl;
                                
                                float largePositionCovariance = 25.0; // TODO small covariance when the entity described does not reflect a part which is detected
                                float smallPositionCovariance = Q;
                                Eigen::MatrixXf C = Eigen::MatrixXf::Zero( 2, 2 ); // Covariance Matrix in the entity-coordinates
                                C.diagonal() << largePositionCovariance, largePositionCovariance;
                                
        //                         float xCorrected, yCorrected;
                                
                        
                                
        //                         std::vector<geo::Vec2f> centerpointsOfEdges = measuredProperty.getRectangle().determineCenterpointsOfEdges ( ); // first 2 belong to the width, 2nd 2 belong to the depth
        //                         std::vector<geo::Vec2f> corners = measuredProperty.getRectangle().determineCorners ( 0.0 );
                                
        //                         ed::tracking::Rectangle rectangleCorrected = measuredProperty.getRectangle();
                                
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
        //                         ed::tracking::wrap2Interval(&angle, -M_PI_2, M_PI_2); std::cout << "angle = " << angle << std::endl;
                                float ct = cos( angle );
                                float st = sin( angle );
                                
                                // We have certainty about the y-coordinate in the entity-frame
                                C( 1, 1) = smallPositionCovariance; // TODO right dim set??
                                        
                                if( measuredProperties[iProperties].methodRectangle == ed::tracking::RECTANGLE )
                                {
                                        C( 0, 0) = smallPositionCovariance;
                                }
                                
                                // Give certainties in terms of the world frame
                                Eigen::MatrixXf Rot = Eigen::MatrixXf::Zero( 2, 2 );
                                Rot << std::cos( rot ), -std::sin( rot ),
                                std::sin( rot ),  std::cos( rot);
                        
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
                
                if( DEBUG )
                        std::cout << "Test 3 \t";
               
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
                
                if( DEBUG )
                        std::cout << "Test 4 \t";
                
                
                 std::cout << "Before update: " << std::endl;
                 entityProperties.rectangle_.printProperties();
                std::cout << " \n Measurement: \t" ;
                measuredProperty.rectangle_.printProperties();
                
                entityProperties.updateRectangleFeatures(QmRectangle, RmRectangle, zmRectangle, dt, sensor_pose);
             
                std::cout << "\nAfter update: \t";
                entityProperties.rectangle_.printProperties();
                

                
                // update circular properties
                Eigen::MatrixXf QmCircle = Eigen::MatrixXf::Zero( 5, 5 );
                Eigen::MatrixXf RmCircle = Eigen::MatrixXf::Zero( 3, 3 );
                
                if( DEBUG )
                        std::cout << "Test 5 \t";
                QmCircle.diagonal() << Q, Q, Q, Q, Q;
                RmCircle.diagonal() << R, R, R;
                
                // TODO what to do with position information if there is low confidence in with and depth? Position information should be updated with respect to an anchor point!
                if( measuredProperties[iProperties].confidenceRectangleDepth == false )
                {
                        QmCircle( 4, 4 ) = largeCovariance;
//                         std::cout << " QmCircle = " << QmCircle << std::endl;
                } 
                
                if( DEBUG )
                        std::cout << "Test 6 \t";
                Eigen::VectorXf zmCircle( 3 );
                zmCircle <<
                measuredProperty.getCircle().get_x(),
                measuredProperty.getCircle().get_y(),
                measuredProperty.getCircle().get_radius();
                

                
                
                if( DEBUG )
                        std::cout << "Test 7 \t";
                entityProperties.updateCircleFeatures(QmCircle, RmCircle, zmCircle, dt);
                if( DEBUG )
                        std::cout << "Test 8 \t";
                entityProperties.updateProbabilities ( measuredProperty.getFeatureProbabilities() );
                
//                 std::cout << "Entity Property after = " << std::endl; entityProperties.printProperties();
                
                if( DEBUG )
                        std::cout << "Test 9 \t";
//                 float Q = 0.1; // Measurement noise covariance. 
//                 float R = 0.0; // Process noise covariance
//
//                 Eigen::MatrixXd Qm ( 2, 2 ), Rm ( 2, 2 );
//                 Eigen::VectorXd z_k ( 2, 1 );
//
//                 Qm << Q, 0, 0, Q;
//                 Rm << R, 0, 0, R;
//
//                 z_k << measuredProperty.getRectangle().get_w(), measuredProperty.getRectangle().get_d(); // How are the width and depth determined? How to ensure the depth-information will not be confused with the width-information?
//
//                 entityProperties.updateProbabilities ( measuredProperty.getFeatureProbabilities() ); // TODO: update probabilities of the features -> Do we still need to use them? Because this will be solved in the PF
//                 entityProperties.updateCircleSize ( Q, R, measuredProperty.getCircle().get_radius() );
//                 entityProperties.updateRectangleSize ( Qm, Rm, z_k );

                

          //      std::cout << "Association with entity " << e->id() << " need to update properties" << std::endl;

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
            req.setProperty ( id, featureProperties_, entityProperties );
            req.setPose ( id, new_pose );

            // Set timestamp
            req.setLastUpdateTimestamp ( id, scan->header.stamp.toSec() );
            req.setExistenceProbability ( id, existenceProbability );
        }
    }
    
//     ObjectMarkers_pub_.publish(markers);

// - - - - - - - - - - - - - - - - -

    std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms. \n\n\n" << std::endl;
}

// ----------------------------------------------------------------------------------------------------


void LaserPluginTracking::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    scan_buffer_.push(msg);
}


ED_REGISTER_PLUGIN(LaserPluginTracking)
