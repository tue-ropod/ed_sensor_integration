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

// 
namespace
{

typedef std::vector<unsigned int> ScanSegment;

bool sortBySegmentSize(const ScanSegment &lhs, const ScanSegment &rhs) { return lhs.size() > rhs.size(); }

ros::Time tLatestPoseInit( 0.0 );
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

void renderWorld(const geo::Pose3D& sensor_pose, std::vector<double>& model_ranges, const ed::WorldModel& world, geo::LaserRangeFinder lrf_model)
{
     geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    
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

 std::vector<ScanSegment> determineSegments(std::vector<float> sensor_ranges, int maxGapSize, int minSegmentSize, float segmentdepthThreshold, geo::LaserRangeFinder lrf_model, double minClusterSize, double maxClusterSize)
{
//         std::cout << "In determine Segments " << std::endl;
//         
//         std::cout << " maxGapSize = " << maxGapSize <<
//     " minSegmentSize = " << minSegmentSize <<
//     " segmentdepthThreshold = " << segmentdepthThreshold <<
//     " minClusterSize = " << minClusterSize <<
//     " maxClusterSize = " << maxClusterSize << std::endl;
        
        unsigned int num_beams = sensor_ranges.size();
         std::vector<ScanSegment> segments;
 // Find first valid value
    ScanSegment current_segment;
    for ( unsigned int i = 0; i < num_beams - 1; ++i )
    {
//             std::cout << "sensor_ranges[i] = " << sensor_ranges[i] << std::endl;
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
        return segments;
    }

//     if( DEBUG )
//             std::cout << "Debug 7 \t";
    int gap_size = 0;
    std::vector<float> gapRanges;

//     for(unsigned int i = currentSegmentInfo.segmentRanges.front(); i < num_beams; ++i)
    for(unsigned int i = current_segment.front(); i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        
//         std::cout << "rs = " << rs << std::endl;

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
//                             std::cout << "current_segment.size() = " << current_segment.size();
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
    
//     std::cout << "In determineSegments: segmenst.size() = " << segments.size();
    return segments;
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
    
    int i_check_door_status = 0;
    config.value("check_door_status", i_check_door_status, tue::OPTIONAL);
    check_door_status_ = (i_check_door_status != 0);
    
    int i_correct_x_yPos = 0;
    config.value("correctXYpos", i_correct_x_yPos, tue::OPTIONAL);
    correctXYpos_ = (i_correct_x_yPos != 0);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    unsigned int bufferSize = 1; // TODO increase to 3(?) in order to make it possible to process more laser data in 1 iteration. Set low for testing purposes now.
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, bufferSize, &LaserPluginTracking::scanCallback, this);
    door_pub_ = nh.advertise<ed_sensor_integration::doorDetection>("door", 3);
    ObjectMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/objectMarkers2", 3 ); // TEMP
//     PointMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/pointMarkers", 3 ); // TEMP
//     processedLaserPoints_pub_ = nh.advertise<sensor_msgs::LaserScan> ( "processedLaserScan", 3 ); // TEMP
    pose_updated_pub_ = nh.advertise<geometry_msgs::PoseStamped> ( "PoseTest", 3 ); // TEMP
    points_measured_pub_ = nh.advertise<visualization_msgs::Marker> ( "MeasuredPoints", 3 ); // TEMP;
    points_modelled_pub_ = nh.advertise<visualization_msgs::Marker> ( "ModelledPoints", 3 ); // TEMP;
    points_modelled_all_pub_ = nh.advertise<visualization_msgs::Marker> ( "ModelledPointsAll", 3 ); // TEMP;
    points_measured_all_pub_ = nh.advertise<visualization_msgs::Marker> ( "MeasuredPointsAll", 3 ); // TEMP;
    cornerPointModelled_pub_ = nh.advertise<visualization_msgs::Marker> ( "cornerPointModelled", 3 ); // TEMP
    cornerPointMeasured_pub_ = nh.advertise<visualization_msgs::Marker> ( "cornerPointMeasured", 3 ); // TEMP
    associatedPoints_pub_ = nh.advertise<sensor_msgs::LaserScan> ("ed/associatedPoints", 3); // TEMP
//     initializedPose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initPose", bufferSize, &LaserPluginTracking::PoseWithCovarianceStampedInitCallback, this);
//     improvedRobotPos_pub_= nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "initialpose", 3 );
//     amclPose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", bufferSize, &LaserPluginTracking::PoseWithCovarianceStampedCallback, this);

    tf_listener_ = new tf::TransformListener;
    
    // example given in ED/ed/examples/custom_properties. Update the probabilities using an update-request
    // TODO defined in multiple places now
    init.properties.registerProperty ( "Feature", featureProperties_, new FeaturPropertiesInfo ); 
    

    //pose_cache.clear();
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
//         std::cout << termcolor::red << "pose_buffer_init_.size() before = " << pose_buffer_init_.size() << termcolor::reset << std::endl; 
  //      Do not pub improved localisation on AMCL-topic -> gives problems with time-synchronisation!
        /*
    if( !pose_buffer_init_.empty() )
    {
            geometry_msgs::PoseWithCovarianceStamped::ConstPtr robotPose = pose_buffer_init_.front();
            geometry_msgs::PoseWithCovarianceStamped updatedPose;
            
            updatedPose.header = robotPose->header;
            updatedPose.pose = robotPose->pose;
            
            improvedRobotPos_pub_.publish(updatedPose);
//             pose_buffer_init_.empty();
            tLatestPoseInit = ros::Time::now();
    }
    
    if( ros::Time::now() - tLatestPoseInit < DELAY_AFTER_INIT )
    {           
            pose_buffer_.empty();
    }
    */
//     std::cout << termcolor::red << "pose_buffer_init_.size() after = " << pose_buffer_init_.size()  << termcolor::reset << std::endl;
    
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
                         geo::Pose3D& sensor_pose, ed::UpdateRequest& req)
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
   //       std::cout << termcolor::on_yellow << "Start of plugin" << termcolor::reset << std::endl;


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

    
//     geo::Pose3D sensor_pose_inv = sensor_pose.inverse();
// 
//     std::vector<double> model_ranges(num_beams, 0);
//     for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
//     {
//         const ed::EntityConstPtr& e = *it;
// 	
//         if (e->shape() && e->has_pose() && !(e->hasType("left_door") || e->hasType("door_left") || e->hasType("right_door") || e->hasType("door_right" ) || e->hasFlag("non-localizable")))
//         {
// 	//  std::cout << "Shape after = " << e->shape() << std::endl;
//             // Set render options
//             geo::LaserRangeFinder::RenderOptions opt;
//             opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());
// 
//             geo::LaserRangeFinder::RenderResult res(model_ranges);
//             lrf_model_.render(opt, res);
//         }        
//     }

std::vector<double> model_ranges(num_beams, 0);
renderWorld(sensor_pose, model_ranges, world, lrf_model_);

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

//     std::vector<float> sensor_rangesOriginal = sensor_ranges;
//     std::vector<float> model_rangesOut; // TEMP
    std::vector<float> modelRangesAssociatedRanges2StaticWorld(sensor_ranges.size(), 0.0 );
    
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];
//         model_rangesOut.push_back(rm);
        
//         std::cout << "Sensor ranges = " << sensor_ranges[i] << ", model_ranges = " << model_ranges[i]<< std::endl;

        if (   //    (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                (std::abs(rm - rs) < world_association_distance_))
        {
                modelRangesAssociatedRanges2StaticWorld[i] = model_ranges[i];
//                 sensor_ranges[i] = 0;
        }
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

    if( DEBUG )
            std::cout << "Debug 5 \t";
    
//     std::vector<ScanSegment> segments = determineSegments(sensor_ranges, max_gap_sizemodelRangesAssociatedRanges2StaticWorld_, min_segment_size_pixels_, segment_depth_threshold_, lrf_model_, min_cluster_size_, max_cluster_size_ );    

    float segmentDepthThresholdSmall = segment_depth_threshold_;
    std::vector<ScanSegment> staticSegments = determineSegments(modelRangesAssociatedRanges2StaticWorld, max_gap_size_, min_segment_size_pixels_, segmentDepthThresholdSmall, lrf_model_, min_cluster_size_, max_cluster_size_ );    
//     std::cout << "staticSegments.size() = " << staticSegments.size() << std::endl;
    std::sort(staticSegments.begin(), staticSegments.end(), sortBySegmentSize);
//     std::cout << "Size of staticSegments = ";
//     for (unsigned int iSegmentTest = 0; iSegmentTest < staticSegments.size(); iSegmentTest++)
//     {
//             std::cout << staticSegments[iSegmentTest].size() << "\t";
//     }
//     std::cout << "\n";
//     
    
//     std::cout << "cornerPointMeasured init" << std::endl;
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
//      std::cout << "cornerPointMeasured init 2" << std::endl;
//      std::cout << "staticSegments.size() = " << staticSegments.size() << std::endl;
     
    for(unsigned int iSegment = 0; iSegment < staticSegments.size(); iSegment++)
    {
//             std::cout << "Bla";
           ScanSegment staticSegment = staticSegments[iSegment];
//            std::cout << "After Bla";
//            std::cout << "For iSegment = " << iSegment << " staticSegment read. Size = " << staticSegment.size() << std::endl;
//            std::cout << "Static Segment size = " << staticSegment.size() << std::endl;
           
           if( staticSegment.size() < min_segment_size_pixels_ )
           {
                   break;
           }
           
//            std::vector< geo::Vec2f> points(  staticSegment.size() ), modelledPoints(  staticSegment.size() ) ;
           points.clear(); modelledPoints.clear() ;
//            std::cout << "Point info ";
//            std::cout << "staticSegment.size() = " << staticSegment.size();
           
           for( unsigned int iSeg2Point = 0; iSeg2Point < staticSegment.size(); iSeg2Point++)
           {
                   unsigned int j = staticSegment[iSeg2Point];
                   geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];
                   geo::Vector3 p_model = lrf_model_.rayDirections() [j] * modelRangesAssociatedRanges2StaticWorld[j];
                   
//                    std::cout << "j = " << j << ", p_sensor = " << p_sensor << ", p_model = " << p_model << "\t";
                   
                   // Transform to world frame
                   geo::Vector3 p = sensor_pose * p_sensor;
                   geo::Vector3 p_modelled = sensor_pose * p_model;
                   
                   if ( (p - p_modelled).length() > 0.2 )
                   {
                           std::cout << termcolor::yellow << "p = " << p << ", p_modelled = " << p_modelled << "j = " << j << "sensor_ranges = " << sensor_ranges[j];
                           std::cout << "modelRangesAssociatedRanges2StaticWorld = " << modelRangesAssociatedRanges2StaticWorld[j] << "\t" << termcolor::reset;
                   }
                   
                   
                   // Add to cv array
//                    points[iSeg2Point] = geo::Vec2f ( p.x, p.y );
//                    modelledPoints[iSeg2Point] = geo::Vec2f ( p_modelled.x, p_modelled.y );  
                   points.push_back( geo::Vec2f ( p.x, p.y ) );
                   modelledPoints.push_back( geo::Vec2f ( p_modelled.x, p_modelled.y ) );   
           }
           
           
//            std::cout << "points[0, final] = " << points[0] << ", " << points.back() << ". modelledPoints[0, final] = " << modelledPoints[0] << ", " << modelledPoints.back() << std::endl;
                      
//            std::cout << " modelledPoints.size() = " << modelledPoints.size() << std::endl;
           
           
//            unsigned int segmentLength = staticSegment.size();       
//            unsigned int startElement = segmentLength / SEGMENT_DIVISION_FOR_FITTING;
//            unsigned int finalElement = segmentLength * (SEGMENT_DIVISION_FOR_FITTING - 1)/SEGMENT_DIVISION_FOR_FITTING;
           
           unsigned int segmentLength = staticSegment.size();       
           unsigned int startElement = 0;// segmentLength / SEGMENT_DIVISION_FOR_FITTING;
           unsigned int finalElement = segmentLength;// segmentLength * (SEGMENT_DIVISION_FOR_FITTING - 1)/SEGMENT_DIVISION_FOR_FITTING;
//            
//            std::vector<geo::Vec2f>::iterator it_start = std::next( points.begin(), startElement );
           std::vector<geo::Vec2f>::iterator it_start = points.begin(); std::advance(it_start, startElement);
//            std::vector<geo::Vec2f>::iterator it_end = std::next( points.begin(), finalElement );
            std::vector<geo::Vec2f>::iterator it_end = points.begin(); std::advance( it_end, finalElement );
           
//            std::cout << "\nFor measured points: ";
//            bool cornerFound = ed::tracking::findPossibleCorner ( points, &possibleCorners, &it_start, &it_end, MIN_DISTANCE_CORNER_DETECTION_LARGE );
           possibleCorners.clear();
           bool cornersFoundMeasured = ed::tracking::findPossibleCorners (points, &possibleCorners, MIN_DISTANCE_CORNER_DETECTION_LARGE, min_segment_size_pixels_  );
          
           
//            std::cout << "cornerPointsFound  = " << cornerPointsFound << " cornerFound = " << cornerFound << std::endl;
           
//            std::cout << "cornersFoundMeasured = " << cornersFoundMeasured << " @ ";
           
//            for( unsigned int ipossibleCorners = 0; ipossibleCorners < possibleCorners.size(); ipossibleCorners++)
//            {
//                    std::cout << possibleCorners[ipossibleCorners] << ", \t";
//            }
           
          
           
//            std::cout << termcolor::red << "correctXYpos_ = " << correctXYpos_ << termcolor::reset << std::endl;
           
           if (!cornerPointsFound && cornersFoundMeasured && correctXYpos_)
           {
                it_start = modelledPoints.begin(); std::advance(it_start, startElement);
//              std::next( modelledPoints.begin(), startElement );
                it_end = modelledPoints.begin(); std::advance(it_end , finalElement );
//                 std::cout << "For modelled points: ";
//                 bool cornerFoundModel = ed::tracking::findPossibleCorner ( modelledPoints, &possibleCornersModel, &it_start, &it_end, MIN_DISTANCE_CORNER_DETECTION_LARGE );
                
                possibleCornersModel.clear();
                bool cornersFoundModelled = ed::tracking::findPossibleCorners (modelledPoints, &possibleCornersModel, MIN_DISTANCE_CORNER_DETECTION_LARGE, min_segment_size_pixels_  );
//                 std::cout << "cornersFoundMeasured = " << cornersFoundModelled << " @ ";
//            for( unsigned int ipossibleCorners = 0; ipossibleCorners < possibleCornersModel.size(); ipossibleCorners++)
//            {
//                    std::cout << possibleCornersModel[ipossibleCorners] << ", \t";
//            }
//                  std::cout << "cornerFoundModel  = " << cornersFoundModelled << std::endl;
                 
                if(cornersFoundMeasured && cornersFoundModelled)
                {
//                         std::cout << "CornerModelFound" << std::endl;
//                         std::cout << "cornerPointMeasured assigned "<< std::endl;
                        cornerPointsFound = true;
                        
                        
                        // do neighrest neighbour association of all corners
                        // calc average diff
                        // test correction
                        
                        measuredCornerElement = possibleCorners[0];
                        
                        
                        // TEST
                         // always make the matrix square!
                      /*  dlib::matrix<int> cost(4, 4);
                        cost =  5, 5, 3, 0,
                                6, 7, 4, 0,
                                7, 0, 5, 0,
                                1, 2, 6, 0;
                               
                               std::cout << "Cost = " << cost << std::endl;
                               std::cout << "Cost(00) = " << cost(0, 0) << std::endl;
                               std::cout << "Cost = " << cost(0, 1) << std::endl;
                               std::cout << "Cost = " << cost(2, 3) << std::endl;
                               std::cout << "Cost = " << cost(3, 2) << std::endl;
*/
//                       std::cout << "max sizes = " << possibleCorners.size() << ", " << possibleCornersModel.size() << std::endl;
                      unsigned int size = std::max(possibleCorners.size(), possibleCornersModel.size());
                      dlib::matrix<int> cost = dlib::ones_matrix<int> (size, size); // assignment can handly int's only, so convert to mm
                      cost = 0;// makes it a zero-matrix over all elements
//                       cost = std::numeric_limits< double >::infinity()*cost; 
//                       std::cout << "cost = " << cost << " size = " << size << std::endl;
                      
                      for (unsigned int iMeasuredCorners = 0; iMeasuredCorners < possibleCorners.size(); iMeasuredCorners++)
                      { 
//                               std::cout << " iMeasuredCorners = " << iMeasuredCorners;
//                               std::cout << "possibleCorners[iMeasuredCorners] = " << possibleCorners[iMeasuredCorners] << std::endl;
//                               std::cout << "lrf_model_.rayDirections() [ possibleCorners[iMeasuredCorners] ] = " << lrf_model_.rayDirections() [ possibleCorners[iMeasuredCorners] ];
//                               std::cout << "sensor_ranges[ possibleCorners[iMeasuredCorners] ] = " << sensor_ranges[ possibleCorners[iMeasuredCorners] ];
                              geo::Vector3 p_sensor = lrf_model_.rayDirections() [ possibleCorners[iMeasuredCorners] ] * sensor_ranges[ possibleCorners[iMeasuredCorners] ];
//                       std::cout << "boe " << possibleCornersModel.size() << " p_sensor = " << p_sensor << std::endl;
                      
                              for (unsigned int iModelledCorners = 0; iModelledCorners < possibleCornersModel.size(); iModelledCorners++)
                              {
//                                       std::cout << termcolor::on_red << "TADAA" << termcolor::reset << std::endl;
//                                       std::cout << " iModelledCorners = " << iModelledCorners;
                                      geo::Vector3 p_model = lrf_model_.rayDirections() [ possibleCornersModel[iModelledCorners] ] * modelRangesAssociatedRanges2StaticWorld[ possibleCornersModel[iModelledCorners] ];
//                       std::cout << " heu ";
//                       std::cout << "p_sensor = " << p_sensor << " p_model = " << p_model << std::endl;
                      
                                      float dist = std::sqrt( std::pow(p_sensor.x - p_model.x, 2.0 ) +  std::pow(p_sensor.y - p_model.y, 2.0 ) );
//                                       std::cout << "moeh dist = " << dist;
                                     int ElementCost = (int) 1000 / dist;
                                      
                                      cost(iMeasuredCorners, iModelledCorners) = ElementCost; // assignment can handle int's only, so scale wisely
//                                       std::cout << " ha cost = " << ElementCost;
                              } 
                      }
                      
                      
//                         std::cout << "cost = " << cost << " size = " << size << std::endl;
                      
                       // To find out the best assignment of people to jobs we just need to call this function.
                        std::vector<long> assignment = dlib::max_cost_assignment(cost);

                        // This prints optimal assignments:  [2, 0, 1] which indicates that we should assign
                        // the person from the first row of the cost matrix to job 2, the middle row person to
                        // job 0, and the bottom row person to job 1.
                        
//                         std::cout << "assignment.size() = " << assignment.size() << std::endl;
                        
                        for (unsigned int i = 0; i < assignment.size(); i++)
//                                 std::cout << assignment[i] << ", Element = " << i << assignment[i] << "Cost = " << cost(i, assignment[i]) << std::endl;
// 
                        // This prints optimal cost:  16.0
                        // which is correct since our optimal assignment is 6+5+5.
                        std::cout << "optimal cost: " << dlib::assignment_cost(cost, assignment) << std::endl;
                        
                        unsigned int counter = 0;
                        xyDiff.x = 0.0;
                        xyDiff.y = 0.0;
                        for (unsigned int i = 0; i < assignment.size(); i++)
                        {
                                if( i < possibleCorners.size() && assignment[i] <  possibleCornersModel.size() )
                                {
                                        counter++;
                                         geo::Vector3 p_sensor = lrf_model_.rayDirections() [ possibleCorners[i] ] * sensor_ranges[ possibleCorners[i] ];
                                         geo::Vector3 p_model = lrf_model_.rayDirections() [ possibleCornersModel[assignment[i]] ] * modelRangesAssociatedRanges2StaticWorld[ possibleCornersModel[assignment[i]] ];
                                         
                                         geo::Vector3 diff = p_sensor - p_model;
                                         xyDiff.x += diff.x;
                                         xyDiff.y += diff.y; 
                                }
                        }
                        xyDiff.x /= counter;
                        xyDiff.y /= counter;
                        
                        if ( std::pow(xyDiff.x, 2.0) + std::pow(xyDiff.x, 2.0) >  MAX_DISTANCE_POS_CORRECTION2 )
                        {
                                ROS_WARN("Large pos correction. No correction made. xyDiff.x = %f, xyDiff.y = %f", xyDiff.x, xyDiff.y);
                                xyDiff.x = 0.0;
                                xyDiff.y = 0.0;       
                        }
                      
                  /*      
                        shift = ed::tracking::maxCrossCorrelation(sensor_ranges, staticSegment.begin(), staticSegment.end(), modelRangesAssociatedRanges2StaticWorld, staticSegment.begin(), staticSegment.end());
                        
                        int diffShift = 0; // in order to correct for the fact that the shift is such that the element is outside of the range allowed
                        
                        if ( shift < 0 && -shift > measuredCornerElement )
                        {
                                diffShift = -(measuredCornerElement + shift);
                                std::cout << "diffshift option 1, measuredCornerElement - shift = " << measuredCornerElement + shift << std::endl;
                        }
                        else if( shift > 0 && measuredCornerElement + shift > modelledPoints.size() + 1 )
                        {
                                diffShift = -(measuredCornerElement + shift - modelledPoints.size() + 1 );
                                std::cout << "diffshift option 2, measuredCornerElement + shift = " << measuredCornerElement + shift << std::endl;
                        }
                        
//                         unsigned int lowerBoundShift = -measuredCornerElement;
//                         unsigned int upperBoundShift = modelledPoints.size() - measuredCornerElement;
//                         if( shift < lowerBoundShift)
//                         {
//                                 diffShift = lowerBoundShift;
//                         }
//                         else if( shift > upperBoundShift )
//                         {
//                                 diffShift = upperBoundShift;
//                         }
                        
//                         TODO and TEST: elementOfCornerMeasured must be unsigned int, such that measuredCornerElement is vald!
                         std::cout << " modelledPoints.size() = " << modelledPoints.size();
                         std::cout << "measuredCornerElement = " << measuredCornerElement << " shift = " << shift << " diffShift = " << diffShift;
                        std::cout << "measuredCornerElement = " << measuredCornerElement << std::endl;
                        unsigned int elementOfCornerMeasured = measuredCornerElement + diffShift ;
                        
                          std::cout << " elementOfCornerMeasured = " << elementOfCornerMeasured;
                        cornerPointMeasured = points[elementOfCornerMeasured];
                        
                        unsigned int staticSegmentElementOfCornerModelled = measuredCornerElement + shift + diffShift ;
                         std::cout << " elementOfCornerModelled = " << staticSegmentElementOfCornerModelled;
                        cornerPointModelled = modelledPoints[ staticSegmentElementOfCornerModelled ];
                        
                        sensorElementOfCornerModelled = staticSegment[0] + staticSegmentElementOfCornerModelled;
                      
                       
                        std::cout << "StartPoints = " << modelledPoints[0] << ", " << points[0] << std::endl;
                        std::cout << "CornerPoints = " << cornerPointModelled << ", " << cornerPointMeasured << std::endl;
                        std::cout << "ElementsOfCorner = " << staticSegmentElementOfCornerModelled << ", " << elementOfCornerMeasured << std::endl;
                     */   
//                         geo::Vector3 p_model = lrf_model_.rayDirections() [elementOfCorner] * modelRangesAssociatedRanges2StaticWorld[elementOfCorner];
                        
//                         cornerPointModelled = modelledPoints[ elementOfCorner ];
                        
//                         std::cout << "cornerPointMeasured = " << cornerPointMeasured << " cornerPointModelled = " << cornerPointModelled << std::endl;

//                    std::cout << " AngleStart = " << yawSensor + lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*staticSegment[0] << " staticSegment[0] = " << staticSegment[0];
//                    std::cout << "yawSensor = " << yawSensor << " lrf_model_.getAngleMin() = " << lrf_model_.getAngleMin()  << " lrf_model_.getAngleIncrement() = " << lrf_model_.getAngleIncrement() << std::endl;
                   
                        visualization_msgs::Marker pointsModelledAll;
                        pointsModelledAll.header.frame_id = "/map";
                        pointsModelledAll.header.stamp = scan->header.stamp;
                        pointsModelledAll.ns = "modelledPoints";
                        pointsModelledAll.id = 1;
                        pointsModelledAll.type = visualization_msgs::Marker::POINTS;
                        pointsModelledAll.action = visualization_msgs::Marker::ADD;
                        pointsModelledAll.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( 0.0, 0.0, 0.0 );
                        pointsModelledAll.scale.x = 0.03;
                        pointsModelledAll.scale.y = 0.03;
                        pointsModelledAll.scale.z = 0.03;
                        pointsModelledAll.color.r = 1.0;
                        pointsModelledAll.color.g = 0.0;
                        pointsModelledAll.color.b = 1.0;
                        pointsModelledAll.color.a = 1.0; 
                        pointsModelledAll.lifetime = ros::Duration( TIMEOUT_TIME );
                             
                        visualization_msgs::Marker pointsAll;
                        pointsAll.header.frame_id = "/map";
                        pointsAll.header.stamp = scan->header.stamp;
                        pointsAll.ns = "measuredPoints";
                        pointsAll.id = 1;
                        pointsAll.type = visualization_msgs::Marker::POINTS;
                        pointsAll.action = visualization_msgs::Marker::ADD;
                        pointsAll.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( 0.0, 0.0, 0.0 );
                        pointsAll.scale.x = 0.03;
                        pointsAll.scale.y = 0.03;
                        pointsAll.scale.z = 0.03;
                        pointsAll.color.r = 1.0;
                        pointsAll.color.g = 1.0;
                        pointsAll.color.b = 0.0;
                        pointsAll.color.a = 1.0; 
                        pointsAll.lifetime = ros::Duration( TIMEOUT_TIME );
           
                        for( unsigned int iSeg2Point = 0; iSeg2Point < staticSegment.size(); iSeg2Point++)
                        {
                //             std::cout << "Points = " << points[iSeg2Point] << modelledPoints[iSeg2Point] << "\t";
                                unsigned int element = staticSegment[iSeg2Point];
                                float angle = yawSensor + lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*element;
                                
                                modelledPoints[iSeg2Point].x = sensor_pose.getOrigin().getX() + modelRangesAssociatedRanges2StaticWorld[element]*cos( angle );
                                modelledPoints[iSeg2Point].y = sensor_pose.getOrigin().getY() + modelRangesAssociatedRanges2StaticWorld[element]*sin( angle );
                                 
                                geometry_msgs::Point point2Pub;
                                point2Pub.x = sensor_pose.getOrigin().getX() + sensor_ranges[element]*cos( angle );
                                point2Pub.y = sensor_pose.getOrigin().getY() + sensor_ranges[element]*sin( angle );
                                point2Pub.z = sensor_pose.getOrigin().getZ();        
                                pointsAll.points.push_back(point2Pub);
                        
                                geometry_msgs::Point p;
                                p.x = modelledPoints[iSeg2Point].x;
                                p.y = modelledPoints[iSeg2Point].y;
                                p.z = sensor_pose.getOrigin().getZ();
                                                
                                pointsModelledAll.points.push_back(p);       
                        }
                        
                        points_measured_all_pub_.publish(pointsAll);
                        points_modelled_all_pub_.publish(pointsModelledAll);
                        
                                        visualization_msgs::Marker cornerPointMeasuredMarker, cornerPointModelledMarker;
                    
                        cornerPointMeasuredMarker.header.frame_id = "/map";
                        cornerPointMeasuredMarker.header.stamp = scan->header.stamp;
                        cornerPointMeasuredMarker.ns = "measuredPoints";
                        cornerPointMeasuredMarker.id = 1;
                        cornerPointMeasuredMarker.type = visualization_msgs::Marker::POINTS;
                        cornerPointMeasuredMarker.action = visualization_msgs::Marker::ADD;
        //                     pointsMeasured.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollSensor, pitchSensor, yawSensor );
                        cornerPointMeasuredMarker.scale.x = 0.1;
                        cornerPointMeasuredMarker.scale.y = 0.1;
                        cornerPointMeasuredMarker.scale.z = 0.1;
                        cornerPointMeasuredMarker.color.r = 1.0;
                        cornerPointMeasuredMarker.color.g = 1.0;
                        cornerPointMeasuredMarker.color.b = 0.0;
                        cornerPointMeasuredMarker.color.a = 1.0; 
                        cornerPointMeasuredMarker.lifetime = ros::Duration( TIMEOUT_TIME );
                        
                        cornerPointModelledMarker = cornerPointMeasuredMarker;
                        
                        for(unsigned ipossibleCorners = 0; ipossibleCorners < possibleCorners.size(); ipossibleCorners++)
                        {
                                geometry_msgs::Point pos;
                                cornerPointMeasured = points[possibleCorners[ipossibleCorners]];
                              
                                pos.x = cornerPointMeasured.x;
                                pos.y = cornerPointMeasured.y;
                                pos.z = sensor_pose.getOrigin().getZ();                        
                                cornerPointMeasuredMarker.points.push_back(pos);
                        }      
                        cornerPointMeasured_pub_.publish(cornerPointMeasuredMarker); 
                        
                        cornerPointModelledMarker.color.r = 1.0;
                        cornerPointModelledMarker.color.g = 0.0;
                        cornerPointModelledMarker.color.b = 1.0;
                        cornerPointModelledMarker.color.a = 1.0;
                        for(unsigned ipossibleCornersModel = 0; ipossibleCornersModel < possibleCornersModel.size(); ipossibleCornersModel++)
                        {       
                                cornerPointModelled = modelledPoints[possibleCornersModel[ipossibleCornersModel]];
                                geometry_msgs::Point pos;
                                pos.x = cornerPointModelled.x;
                                pos.y = cornerPointModelled.y;
                                pos.z = sensor_pose.getOrigin().getZ();   
                        
                                cornerPointModelledMarker.points.push_back(pos);
                        }
                        cornerPointModelled_pub_.publish(cornerPointModelledMarker); 
                        
                        

//                         std::cout << "Shift = " << shift << std::endl;
                        
//            int maxCrossCorrelation(std::vector<float>& measuredRanges, std::vector<unsigned int>::iterator measuredRangesStartElement,  std::vector<unsigned int>::iterator measuredRangesFinalElement,
//                         std::vector<float>& modelledRanges, std::vector<unsigned int>::iterator modelledRangesStartElement,  std::vector<unsigned int>::iterator modelledRangesFinalElement);
                }
           }       

//            float mean, standardDeviation;
//            ed::tracking::determineIAV(modelRangesAssociatedRanges2StaticWorld, &mean, &standardDeviation, lrf_model_, staticSegment[0], staticSegment.back() );
//             std::cout << "Mean IAV = " << mean  << " abs diff = " << std::abs( mean - M_PI ) << "standardDeviation = " << standardDeviation << std::endl;
            
           
//            if( std::abs( mean - M_PI ) < MAX_DEVIATION_IAV_STRAIGHT_LINE )
           if( !cornersFoundMeasured && ! angleCorrectionFound) // beause we want to correct based on a straight line.
           {
                // straight line detected. Take a subset (2 middle quarters) of this line and do a fit for line as well as the corresponding fit for the ranges expected in the WM
//                 unsigned int segmentLength = staticSegment.size();       
//                 unsigned int startElement = segmentLength / SEGMENT_DIVISION_FOR_FITTING;
//                 unsigned int finalElement = segmentLength * (SEGMENT_DIVISION_FOR_FITTING - 1)/SEGMENT_DIVISION_FOR_FITTING;

                unsigned int segmentLength = staticSegment.size();       
                unsigned int startElement = 0; //segmentLength / SEGMENT_DIVISION_FOR_FITTING;
                unsigned int finalElement = segmentLength; //segmentLength * (SEGMENT_DIVISION_FOR_FITTING - 1)/SEGMENT_DIVISION_FOR_FITTING;

                std::vector<geo::Vec2f> measuredPoints(finalElement - startElement), modelledPoints(finalElement - startElement);
                unsigned int counter = 0;
                
        //             std::cout << "startElement = " << startElement << " finalElement = " << finalElement << " segmentLength = " << segmentLength  << std::endl;
        //             std::cout << "staticSegment[startElement] = " << staticSegment[startElement] << " staticSegment[finalElement] = " << staticSegment[finalElement] << std::endl;
                
                for(unsigned int iLineFit = startElement; iLineFit < finalElement; iLineFit ++)       
                {
        //                     std::cout << "staticSegment.size() = " << staticSegment.size() << " iLineFit = " << iLineFit <<  std::endl;
                        unsigned int element = staticSegment[iLineFit];
                        float angle = yawSensor + lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*element;
                        measuredPoints[counter].x = sensor_pose.getOrigin().getX() + sensor_ranges[element]*cos( angle );
                        measuredPoints[counter].y = sensor_pose.getOrigin().getY() + sensor_ranges[element]*sin( angle );
                        
                        modelledPoints[counter].x = sensor_pose.getOrigin().getX() + modelRangesAssociatedRanges2StaticWorld[element]*cos( angle );
                        modelledPoints[counter].y = sensor_pose.getOrigin().getY() + modelRangesAssociatedRanges2StaticWorld[element]*sin( angle );
                        
        //                     std::cout << "modelledPoints[counter] = " << modelledPoints[counter] << " measuredPoints[counter] = " << measuredPoints[counter] << "counter = " << counter << std::endl;
        //                     std::cout << "sensor_ranges[element] = " << model_ranges[element] << " counter = " << counter << std::endl;
                        counter++;
                        
                }

        //             std::cout << "Angle low = " << lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*staticSegment[startElement];
        //             std::cout << "Angle High = " << lrf_model_.getAngleMin() + lrf_model_.getAngleIncrement()*staticSegment[finalElement] << std::endl;
                
                visualization_msgs::Marker pointsMeasured, pointsModelled;
        //             points.markers.size() = measuredPoints.size() + modelledPoints.size();
                
        //             std::cout << "Measured points = " ;
                
                        pointsModelled.header.frame_id = "/map";
                        pointsModelled.header.stamp = scan->header.stamp;
                        pointsModelled.ns = "modelledPoints";
                        pointsModelled.id = 1;
                        pointsModelled.type = visualization_msgs::Marker::POINTS;
                        pointsModelled.action = visualization_msgs::Marker::ADD;
        //                     pointsModelled.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollSensor, pitchSensor, yawSensor );
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
                        
        //                     std::cout << "Modelled: p = " << p.x << ", " << p.y << ", " << p.z << std::endl;
                        
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
                        
        //                     std::cout << "Measured p = " << p.x << ", " << p.y << ", " << p.z << std::endl;
                        
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
//                 std::cout << "Diffangle = " << diffAngle << std::endl;
                
                if(diffAngle < MAX_DEVIATION_ANGLE_CORRECTION ) // Only correct if localization partially leads to problems
                {
                        angleCorrectionFound = true;
                }
                else
                {
                        ROS_WARN("Big angle correction required. Localisation correct?");
                }
                
           }
           
           if( cornerPointsFound && angleCorrectionFound )        
           {
                   break;
           }
            
           
    }
           geo::Pose3D sensor_poseCorrected;
//            std::cout << "angleCorrectionFound = " << angleCorrectionFound << std::endl;
           if( angleCorrectionFound )
           {                     
                   
            
//                std::cout << termcolor::cyan << "Yaw before = " << yawSensor << " diff = " << diffAngle ; // << "Measured Angle = " << measuredAngle << " Modelled Angle = " << modelledAngle  << " Yaw new =  " << yawSensor - (measuredAngle - modelledAngle) << termcolor::reset << std::endl;
                        yawSensor -= diffAngle;
//                         std::cout << " yaw new = " << yawSensor << termcolor::reset << std::endl;
                        ed::tracking::wrap2Interval(&yawSensor, (double) 0.0, (double) 2*M_PI);
        //             std::cout << "sensor_pose before = " << sensor_pose << std::endl;
                
                       // sensor_pose.setRPY(rollSensor, pitchSensor, yawSensor );
                        sensor_poseCorrected = sensor_pose;
                        sensor_poseCorrected.setRPY(rollSensor, pitchSensor, yawSensor );
                        
                        // Rotation updated, so this influences the delta in x,y. Recompute the cornerPointModelled
                        
//                         unsigned int lowerBoundShift = -measuredCornerElement;
//                         unsigned int upperBoundShift = modelRangesAssociatedRanges2StaticWorld.size() - measuredCornerElement;
//                         unsigned int elementOfCorner = measuredCornerElement + shift;
                        
                        

//                         std::cout << "measuredCornerElement = " << elementOfCornerModelled << " shift = " << shift << " elementOfCorner = " << elementOfCorner << std::endl;
                        
                        geo::Vector3 p_model = lrf_model_.rayDirections() [sensorElementOfCornerModelled] * modelRangesAssociatedRanges2StaticWorld[sensorElementOfCornerModelled];
                        geo::Vector3 p_modelled = sensor_poseCorrected * p_model;
//                         std::cout << " cornerPointModelled before correction = " << cornerPointModelled << std::endl;
                        cornerPointModelled = geo::Vec2f ( p_modelled.x, p_modelled.y );
        //             std::cout << "sensor_pose after = " << sensor_pose << std::endl;
                        
        //                     std::cout << termcolor::red  << "pose_buffer_init_.size() = " << pose_buffer_init_.size() << termcolor::reset << std::endl;
                        // TODO
                        /*
                        if( pose_buffer_init_.empty() )
                        {
                                std::cout << "Ros time = " << ros::Time::now() << " tLatestPoseInit = " << tLatestPoseInit << " diff = " <<  ros::Time::now() - tLatestPoseInit << std::endl;
                                if( !pose_buffer_.empty() && ros::Time::now() - tLatestPoseInit > DELAY_AFTER_INIT)
                                {
                                        
                                        geometry_msgs::PoseWithCovarianceStamped::ConstPtr robotPose = pose_buffer_.front();
                                        geometry_msgs::PoseWithCovarianceStamped updatedPose;
                                        
                                        updatedPose.header = robotPose->header;
                                        updatedPose.pose = robotPose->pose;
                                        
                                        // assumption: orientation robot and sensor similar
                                        updatedPose.pose.pose.orientation.x = sensor_pose.getQuaternion().getX();
                                        updatedPose.pose.pose.orientation.y = sensor_pose.getQuaternion().getY();
                                        updatedPose.pose.pose.orientation.z = sensor_pose.getQuaternion().getZ();
                                        updatedPose.pose.pose.orientation.w = sensor_pose.getQuaternion().getW();
                                        improvedRobotPos_pub_.publish(updatedPose); 
                                }
                        } 
                
                        
                        */
           }
            
//             std::cout << "cornerPointsFound && correctXYpos_ = " << cornerPointsFound << correctXYpos_ << std::endl;
            
            if( cornerPointsFound && correctXYpos_)
            {
//              std::cout << "Diff init " << std::endl;       
//                     geo::Vec2f diff = cornerPointMeasured - cornerPointModelled;
             geo::Vec2f diff = xyDiff;
                    
//                     std::cout << termcolor::magenta << "x,y diff = " << diff << " cornerPointMeasured = " << cornerPointMeasured << ", " << cornerPointModelled << termcolor::reset << std::endl;
                    
                    geo::Vec3T< geo::real > updatedPos(sensor_poseCorrected.getOrigin().getX() - diff.x, sensor_poseCorrected.getOrigin().getY() - diff.y, sensor_poseCorrected.getOrigin().getZ() );
                    sensor_poseCorrected.setOrigin( updatedPos );
                    
            }
            
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
            
            
    
    
//     std::cout << " Eng of staticSegments loop." << std::endl;
    
//     float MD_roll, MD_pitch, MD_yaw;
    
    
//     geo::Pose3D poseWP = e.get()->pose();
//             geo::Quaternion rotationWP = poseWP.getQuaternion();

//             tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
    
//      tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
//             tf::Matrix3x3 matrix ( q );
//             double WP_roll, WP_pitch, WP_yaw;
//             matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );
    
//             sensorRotation.setRPY ( MD_roll, MD_pitch, MD_yaw );
            
//             mobidikPose.setBasis ( rotation );
//     sensor_pose.getBasis().getRotation();
    
//     sensor_pose.setRPY( sensorRotation );

//                         std::cout << "Going to render world" << std::endl;                        
        
    renderWorld(sensor_poseCorrected, model_ranges, world, lrf_model_);
    
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];
//         model_rangesOut.push_back(rm);
        
//         std::cout << "Sensor ranges = " << sensor_ranges[i] << ", model_ranges = " << model_ranges[i]<< std::endl;

        if (rs <= 0
                || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                || (std::abs(rm - rs) < world_association_distance_))
        {
//                 modelRangesAssociatedRanges2StaticWorld[i] = model_ranges[i];
                sensor_ranges[i] = 0;
        }
    }   
     std::vector<ScanSegment> segments = determineSegments(sensor_ranges, max_gap_size_, min_segment_size_pixels_, segment_depth_threshold_, lrf_model_, min_cluster_size_, max_cluster_size_ );    
    
    
    
    // TODO for the static segments, take the first one, check if there are enough points, check if there is a straight line (on a subset as there might be differences with the model and the measurement),
    // do a fit, do the same fit for the modelled ranges on the same subset and do a correction on the relative angle. -> How to improve the AMCL-pose?
    

     

//     ScanSegmentInfo currentSegmentInfo; // TODO remove for initial segments
//     bool confidenceLeft; // check if the object might have been covered by an object on both sides to determine the confidence of the measurement
//     bool confidenceRight;
    
    
    // TODO make a function, such that the world-associations can be segmented as well!
    
//     // Find first valid value
//     ScanSegment current_segment;
//     for ( unsigned int i = 0; i < num_beams - 1; ++i )
//     {
//         if ( sensor_ranges[i] > 0 )
//         {
//             current_segment.push_back(i);
//             break;
//         }
//     }
//     
// //     if( DEBUG )
// //             std::cout << "Debug 6 \t";
// 
// //     if ( currentSegmentInfo.segmentRanges.empty() )
//     if ( current_segment.empty() )
//     {
//         return;
//     }
// 
// //     if( DEBUG )
//             std::cout << "Debug 7 \t";
//     int gap_size = 0;
//     std::vector<float> gapRanges;
// 
// //     for(unsigned int i = currentSegmentInfo.segmentRanges.front(); i < num_beams; ++i)
//     for(unsigned int i = current_segment.front(); i < num_beams; ++i)
//     {
//         float rs = sensor_ranges[i];
// 
//         if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segment_depth_threshold_ || i == num_beams - 1)
//         {
//             // Found a gap
//             ++gap_size;
//             gapRanges.push_back ( rs );
// 
//             if (gap_size >= max_gap_size_ || i == num_beams - 1)
//             {
//                  i = current_segment.back() + 1;
// 
//                 if (current_segment.size()  >= min_segment_size_pixels_)
//                 {
//                     // calculate bounding box
//                     geo::Vec2 seg_min, seg_max;
//                     for(unsigned int k = 0; k <  current_segment.size(); ++k)
//                     {
//                         geo::Vector3 p = lrf_model_.rayDirections()[ current_segment[k]] * sensor_ranges[current_segment[k]];
// 
//                         if (k == 0)
//                         {
//                             seg_min = geo::Vec2(p.x, p.y);
//                             seg_max = geo::Vec2(p.x, p.y);
//                         }
//                         else
//                         {
//                             seg_min.x = std::min(p.x, seg_min.x);
//                             seg_min.y = std::min(p.y, seg_min.y);
//                             seg_max.x = std::max(p.x, seg_max.x);
//                             seg_max.y = std::max(p.y, seg_max.y);
//                         }
//                     }
// 
//                     geo::Vec2 bb = seg_max - seg_min;
//                     if ( ( bb .x > min_cluster_size_ || bb.y > min_cluster_size_ ) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_ )
//                     {
//                         segments.push_back ( current_segment );
//                         
// //                         std::cout << "New segment added. Ranges = "<< std::endl;
// //                         for(unsigned int ii = 0; ii < current_segment.size(); ii++)
// //                         {
// //                                 std::cout << current_segment[ii] << "\t";
// //                         }
// //                         std::cout << "\n";
//                         
//                     }   
//                 }
// 
//                 current_segment.clear();
//                 gapRanges.clear();
// 
//                 // Find next good value
//                 while ( sensor_ranges[i] == 0 && i < num_beams )
//                 {
//                     ++i; // check for confidence left
//                 }
// 
//                 int nPointsToCheck = POINTS_TO_CHECK_CONFIDENCE;
//                 if ( i < nPointsToCheck )
//                 {
//                     nPointsToCheck = i;
//                 }
// 
//                 current_segment.push_back ( i );
//             }
//         }
//         else
//         {
//             gap_size = 0;
//             gapRanges.clear();
//             current_segment.push_back ( i );
//         }
//     }
    
     if( DEBUG )
            std::cout << "Debug 8 \t";
    
    // Try to associate remaining laser points to specific entities
    std::vector<ed::WorldModel::const_iterator> it_laserEntities;
    std::vector< EntityProperty > EntityPropertiesForAssociation;

    if( DEBUG )
std::cout << "Debug 8.1 \t";
    // Check which entities might associate for tracking based on their latest location in order to prevent to check all the entities 
    for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it )
    {
        const ed::EntityConstPtr& e = *e_it;
//                      std::cout << "Going to check entity with id = " << e->id() << std::endl;
        std::string laserID = "-laserTracking";
if( DEBUG )
std::cout << "Debug 8.2 \t";
        if ( e->id().str().length() < laserID.length() )
        {

if( DEBUG )
std::cout << "Debug 8.3 \t";
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

if( DEBUG )
std::cout << "Debug 8.4 \t";
            it_laserEntities.push_back ( e_it );
            
//             std::cout << "tracking loop: Entity added to list of laserEntities: id = " << e->id() << std::endl;
if( DEBUG )
std::cout << "Debug 8.4.1 \t";
// std::cout << "e id = " << e->id() << std::endl;
            ed::tracking::FeatureProperties featureProperties;
//std::cout << "bla " << featureProperties_.idx << std::endl;

	if( !e->property( featureProperties_) )
	{

std::cout << termcolor::cyan << "HEU" << termcolor::reset << std::endl;
ROS_WARN( "NO SEGFAULT NOW! :)"  );
        req.removeEntity ( e->id() );
		continue;
	}


featureProperties = e->property ( featureProperties_ );
if( DEBUG )
        std::cout << "Debug 8.4.2 \t";
            EntityProperty currentProperty;
if( DEBUG )
        std::cout << "Debug 8.4.3 \t";
            float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
            
//             featureProperties.printProperties();

if( DEBUG )
        std::cout << "Debug 8.5 \t";
            // For the entities which already exist in the WM, determine the relevant properties in order to determine which entities _might_ associate to which clusters
            if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() )
            {

if( DEBUG )
        std::cout << "Debug 8.6 \t";

                ed::tracking::Circle circle = featureProperties.getCircle();
                circle.predictAndUpdatePos(dt);
                
                currentProperty.entity_min.x = circle.get_x() - ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_max.x = circle.get_x() + ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_min.y = circle.get_y() - ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_max.y = circle.get_y() + ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                
                if( DEBUG )
std::cout << "Debug 8.7 \t";

            }
            else
            {
                    if( DEBUG )
std::cout << "Debug 8.8 \t";
                ed::tracking::Rectangle rectangle = featureProperties.getRectangle();
                rectangle.predictAndUpdatePos(dt);
                
                std::vector<geo::Vec2f> corners = rectangle.determineCorners ( ASSOCIATION_DISTANCE );
                currentProperty.entity_min = corners[0];
                currentProperty.entity_max = corners[0];
if( DEBUG )
std::cout << "Debug 8.9 \t";
                for ( unsigned int i_corner = 1; i_corner < corners.size(); i_corner++ )
                {
                        if( DEBUG )
std::cout << "Debug 8.10 \t";
                    currentProperty.entity_min.x = std::min ( corners[i_corner].x, currentProperty.entity_min.x );
                    currentProperty.entity_min.y = std::min ( corners[i_corner].y, currentProperty.entity_min.y );
                    currentProperty.entity_max.x = std::max ( corners[i_corner].x, currentProperty.entity_max.x );
                    currentProperty.entity_max.y = std::max ( corners[i_corner].y, currentProperty.entity_max.y );
                    if( DEBUG )
std::cout << "Debug 8.11 \t";
                }
            }
            EntityPropertiesForAssociation.push_back ( currentProperty );
            if( DEBUG )
std::cout << "Debug 8.12 \t";
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
    
    if( DEBUG )
            std::cout << "Debug 9 \t";
    
    
//      std::vector< std::vector< geo::Vec2f > > associatedPointsList ( it_laserEntities.size() );
     std::vector< PointsInfo > associatedPointsInfo( it_laserEntities.size() );
//       std::vector< std::vector< int > > segmentsIDs ( segments.size() ); // IDs of the points in the laser-array
     
//      std::cout << "segments.size() = " << segments.size() << std::endl;
//     visualization_msgs::MarkerArray markerArrayPoints;
//     unsigned int IDPoints = 0;
   

    sensor_msgs::LaserScan test;
    test = *scan;

for(unsigned int iTestLength = 0; iTestLength < test.ranges.size(); iTestLength++)
{
test.ranges[iTestLength] = 0.0;
}
    
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
test.ranges[j] = sensor_ranges[j];
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

if( DEBUG )
        std::cout << "Debug 11 \t";
    
        // After the properties of each segment are determined, check which clusters and entities might associate
        std::vector< int > possibleSegmentEntityAssociations;
//         std::cout << "it_laserEntities.size() = " << it_laserEntities.size() << std::endl;
        for ( unsigned int jj = 0; jj < it_laserEntities.size(); ++jj )
        {
                if( DEBUG )
                std::cout << "Debug 11.1 \t";
//                  const ed::EntityConstPtr& e = *it_laserEntities[jj];
//                 std::cout << "Entity min/mx = " <<  EntityPropertiesForAssociation[jj].entity_min.x << ", " << 
//                 EntityPropertiesForAssociation[jj].entity_max.x << ", " <<
//                 EntityPropertiesForAssociation[jj].entity_min.y << ", " <<
//                 EntityPropertiesForAssociation[jj].entity_max.y << std::endl;
           
//    if( DEBUG )
//            std::cout << "Debug 12 \t";
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
//             std::cout << "Debug 13 \t";
        // If a cluster could be associated to a (set of) entities, determine for each point to which entitiy it belongs based on a shortest distance criterion. 
        // If the distance is too large, initiate a new entity
//         std::vector<geo::Vec2f> pointsNotAssociated;
        PointsInfo pointsNotAssociated;
        std::vector<float> distances ( points.size() );
        std::vector<unsigned int> IDs ( points.size() ); // IDs of the entity which is closest to that point

            if( DEBUG_SF )
            std::cout << "Debug 13.1 \t";
        
        for ( unsigned int i_points = 0; i_points < points.size(); ++i_points )  // Determine closest object and distance to this object. If distance too large, relate to new object
        {
                    if( DEBUG )
            std::cout << "Debug 13.2, size = " << possibleSegmentEntityAssociations.size() << " \t";
            geo::Vec2f p = points[i_points];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = std::numeric_limits< unsigned int >::max();
                if( DEBUG )
            std::cout << "Debug 13.3 \t";
            
//              std::cout << "possibleSegmentEntityAssociations.size() = " << possibleSegmentEntityAssociations.size() << "\t";

            for ( unsigned int jj = 0; jj < possibleSegmentEntityAssociations.size(); jj++ )  // relevant entities only
            {
               // std::cout << "New pos \t";
                if( DEBUG )
                std::cout << "jj = " << jj << " \t";
                const ed::EntityConstPtr& e = *it_laserEntities[ possibleSegmentEntityAssociations[jj] ];

                if( !e-> property ( featureProperties_) )
                {
                        req.removeEntity ( e->id() );
                        std::cout << "req to remove ent " << e->id() << std::endl;
                        continue;
                }
                if( DEBUG )
                std::cout << "Debug 13.4 \t";
// std::cout << "Tracking plugin: Going to get properties of entity" << e->id() << "having age " << scan->header.stamp.toSec() - e->lastUpdateTimestamp() << " \t";
                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
//                 std::cout << "Properties obtained";
                float dist;
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                
                if( DEBUG )
                std::cout << "Debug 13.5 \t";

                //featureProperties.printProperties();

//                 std::cout << "entity id = " << e->id() << std::endl;
//                 std::cout << "hoi" << std::endl;
                double prob1 = featureProperties.getFeatureProbabilities().get_pCircle();
//                 std::cout << "test1 = " << prob1 << std::endl;
                double prob2 = featureProperties.getFeatureProbabilities().get_pRectangle();
//                 std::cout << "test2 = " << prob2 << std::endl;
                bool test = featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle();
//                 std::cout << "bool test = " << test  << std::endl;
//                 std::cout << "Debug 13.5.1 \t";
//  std::cout << "pCirc = " << prob1<< " pRect = " << prob2 << std::endl;
                
                if(prob1 == -1.0 || prob2 == -1.0 )
                {
//                         std::cout << "tracking plugin: req to remove entity with id = " << e->id() << std::endl;
                req.removeEntity ( e->id() );
                continue;
                }

                if ( prob1 > prob2 )  // entity is considered to be a circle
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
                    if( DEBUG )
                                std::cout << "Debug 13.11.4.1 \t";
                }
                if( DEBUG )
                 std::cout << "Debug 13.11.4.2 \t";
                if( DEBUG )
                 std::cout << "jj = " << jj << " possibleSegmentEntityAssociations.size() = " << possibleSegmentEntityAssociations.size() << " \t";
            }
                                                    if( DEBUG )
            std::cout << "Debug 13.11.5 \t";
            distances[i_points] = shortestDistance;

            if( DEBUG )
            std::cout << "Debug 13.11.6 \t";
            IDs[i_points] = id_shortestEntity;
        }
        
         if ( DEBUG_SF )
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
            if ( IDs[iDistances] == IDtoCheck && iDistances != distances.size() - 1 ) // ID similar and not at final reading, check if next element is associated to same entity or not
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
                        if( DEBUG_SF )
            std::cout << "Debug 13.14 \t";
                float minDistance = distances[firstElement];
                if( DEBUG_SF )
                std::cout << "Debug 13.14.0 \t";
                for ( unsigned int iiDistances = 1; iiDistances < iDistances; iiDistances++ )
                {
                    if ( distances[iiDistances] < minDistance )
                    {
                        minDistance = distances[iiDistances];
                    }
                }
                
                if( DEBUG_SF )
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
                        if( DEBUG_SF )
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
            if ( DEBUG_SF )
            std::cout << "Debug 13.18.1 Test\t";
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

                 if ( DEBUG_SF )
                std::cout << "Debug 13.18.2 Test\t";
//                 std::cout << "3: associated, previousSegmentAssociated" << associated << previousSegmentAssociated << std::endl;
                if ( associated )  // add all points to associated entity
                {
                        previousSegmentAssociated = true;
 
                        if ( DEBUG_SF )
                        {
            std::cout << "Debug 13.15 \t";
                    const ed::EntityConstPtr& entityToTest = *it_laserEntities[ possibleSegmentEntityAssociations[IDtoCheck] ];
            std::cout << "iDistances = " << iDistances << " points.size() = " << points.size() << " segmentIDs.size() = " << segmentIDs.size() << std::endl;
            std::cout << "IDtoCheck = " << IDtoCheck << " firstElement = " << firstElement << " iDistances = " << iDistances << std::endl;
            std::cout << "associatedPointsInfo.size() = " << associatedPointsInfo.size() << ", possibleSegmentEntityAssociations.size() = " << possibleSegmentEntityAssociations.size() << std::endl;
            std::cout << "possibleSegmentEntityAssociations[IDtoCheck] = " << possibleSegmentEntityAssociations[IDtoCheck] << std::endl;
                        }
            append( associatedPointsInfo.at ( possibleSegmentEntityAssociations[IDtoCheck] ).points, points, firstElement, iDistances );
             if ( DEBUG_SF )
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
             if ( DEBUG_SF )
            std::cout << "Debug 13.15.1 \t";
                }
                else
                {
 
                        if ( DEBUG_SF )
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
                 if ( DEBUG_SF )
                std::cout << "Debug 13.16.-1 \t";
            }
            
            firstElement = iDistances;
            IDtoCheck = IDs[iDistances];
        }

    }
    
    associatedPoints_pub_.publish(test);
//     std::cout << " Test zoveel " << std::endl;
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
   
    for ( unsigned int iList = 0; iList < associatedPointsInfo.size(); iList++ )
    {
//                              if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.1 \t";
                
            std::vector<unsigned int> IDs = associatedPointsInfo[iList].laserIDs;
      
                         if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.2 \t";
//             std::cout << "\nDebug 13.17.2.1 IDs.size() = " <<  IDs.size();
            if( IDs.size() == 0 )
                    continue;
            
//             std::cout << "\nIDs.size() = " <<  IDs.size();
            for(unsigned int iIDs = 1; iIDs < IDs.size(); iIDs++)
            {
//                                  if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.3 \t";
                    unsigned int idLow = iIDs - 1;
                    unsigned int gapSize = IDs[iIDs] - IDs[iIDs - 1];
//                     std::cout << "idLow = " << idLow << " gapSize = " << gapSize << " idHigh = " << iIDs << " senserElement Low = " << IDs[iIDs - 1] << " senserElement high = " << IDs[iIDs] << "\t";
                    if( gapSize >= min_gap_size_for_split_ )
                    {
//                                          if ( DEBUG_SF )
//                 std::cout << "13.17.4 Debug \t";
                        // check ranges in gap
                        // if there is a set of consecutive ranges (min_gap_size_for_split_) which is significantly larger than ranges before gap and after gap, this implies that there is free space
                        // instead of an object which blocks the view on the object. The entities are splitted and treated as 2 separate ones.
                            
                            unsigned int nLowElements, nHighElements;
                            if (idLow < min_gap_size_for_split_)
                            {
                                    nLowElements = idLow;
                            }
                            else
                            {
                                    nLowElements = min_gap_size_for_split_;
                            }
                            
                            if (iIDs > (IDs.size() - min_gap_size_for_split_))
                            {
                                    nHighElements = IDs.size() - iIDs;
                            }
                            else
                            {
                                    nHighElements = min_gap_size_for_split_;
                            }
//                                          if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.5 \t";
                            
                            float avgRangeLow = 0.0, avgRangeHigh = 0.0;
                            for(unsigned int iAvgLow = 0; iAvgLow < nLowElements; iAvgLow++)
                            {
                                    float range = sensor_ranges[idLow + iAvgLow];
                                    if (range == 0.0 ) // ranges were set to zero if associated to world
                                            range = scan->range_max;
                                    
                                    avgRangeLow += range;
                            }
                            avgRangeLow /= nLowElements;
//                                          if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.6 \t";
                            
                            for(unsigned int iAvgHigh = 0; iAvgHigh < nHighElements; iAvgHigh++)
                            {
                                    float range = sensor_ranges[iIDs + iAvgHigh];
                                    if (range == 0.0 ) // ranges were set to zero if associated to world
                                            range = scan->range_max;
                                    
                                    avgRangeHigh += range;
                            }
                            avgRangeHigh /= nHighElements;
//              if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.7 \t";
                            
                            float maxReference = std::max(avgRangeLow, avgRangeHigh);
                            unsigned int nLargerRanges = 0;

                            for(unsigned int iGap = idLow; iGap < IDs[iIDs]; iGap++ )
                            {
// //                                                  if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.8 \t";
                                    if( IDs[iGap] > maxReference )
                                    {
                                            nLargerRanges++;
                                    }
                                    else
                                    {
                                            nLargerRanges = 0;
                                    }
                                    
                                    if(nLargerRanges >= min_gap_size_for_split_)
                                    {
//                                                          if ( DEBUG_SF )
//                 std::cout << "Debug 13.17.9 \t";
                                            // split
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
//                       std::cout << "Debug 13.17.10 \t";
            }
            
//               std::cout << "Debug 13.17.11 \t";
            endOfLoop:;
//               std::cout << "Debug 13.17.12 \t";
               
    }
    
    
    
//     std::cout << "Before: associatedPointsInfo.size() = " << associatedPointsInfo.size() << std::endl;
    
    for ( unsigned int iList = 0; iList < associatedPointsInfo.size(); iList++ )
    {
            // As each point is associated to an object, it might happen that 2 objects close to each other can be seen as one. An 
            
//             if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.1 " << "iList = " << iList << "\t"; 
            ScanSegment IDs = associatedPointsInfo[iList].laserIDs;
            std::vector< geo::Vec2f > segmentPoints = associatedPointsInfo[iList].points;
      
//                  if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.2 \t"; 
//                 std::cout << "IDs.size() = " << IDs.size() << std::endl;
//            std::cout << "IDs = " << std::endl;
//             for (unsigned int iTest = 0; iTest < IDs.size(); iTest++)
//             {
//                 std::cout << IDs[iTest] << "\t" << sensor_ranges[IDs[iTest]] << "\t" << segmentPoints[iTest].x << "\t" << segmentPoints[iTest].y << std::endl;
//             }   
//             std::cout << "\nDebug 13.18.2.1 \t";
            
//             std::cout << "min_segment_size_pixels_ = " << min_segment_size_pixels_ << std::endl;
            
            unsigned int nPointsForAvg;
            min_segment_size_pixels_ % 2 == 0 ? nPointsForAvg = min_segment_size_pixels_ / 2 : nPointsForAvg = (min_segment_size_pixels_ - 1) / 2;
            
            if( IDs.size() < 2*nPointsForAvg ) // 2 times, because we have to determine the average for the lower and upper part
            {
//                     std::cout << "continue"<< std::endl;
                    continue;
            }
            
//             std::cout << "nPointsForAvg = " << nPointsForAvg << std::endl;
            
//             if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.3 \t"; 
//             float rangeLowSum = 0.0, rangeHighSum = 0.0;
            
            geo::Vec2f pointLowSum, pointHighSum;
            pointLowSum.x = 0.0;
            pointLowSum.y = 0.0;
            pointHighSum.x = 0.0;
            pointHighSum.y = 0.0;
            
            for(unsigned int iAvgLow = 0; iAvgLow < nPointsForAvg; iAvgLow++)
            {
//                     std::cout << "iAvgLow  = " <<  iAvgLow  << std::endl;
//                     std::cout << "IDs[iAvgLow]  = " <<  IDs[iAvgLow]  << std::endl;
//                     std::cout << "sensor_ranges[ IDs[iAvgLow] ] = " << sensor_ranges[ IDs[iAvgLow] ] << std::endl;
//                      std::cout << "test" << std::endl;
//                     rangeLowSum += sensor_ranges[ IDs[iAvgLow] ];
                    
                    pointLowSum += segmentPoints[iAvgLow];
//                       std::cout << "test2" << std::endl;
            }
//              if ( DEBUG_SF )
//                       std::cout << "test3" << std::endl; 
            geo::Vec2f avgPointLow = pointLowSum / nPointsForAvg;
             
            for(unsigned int iAvgHigh = nPointsForAvg; iAvgHigh < 2*nPointsForAvg; iAvgHigh++)
            {
//                     std::cout << "iAvgHigh  = " <<  iAvgHigh  << std::endl;
//                     std::cout << "IDs[iAvgHigh]  = " <<  IDs[iAvgHigh]  << std::endl;
//                     std::cout << "sensor_ranges[ IDs[iAvgHigh] ] = " << sensor_ranges[ IDs[iAvgHigh] ] << std::endl;
//                     std::cout << "IDs.size() = " << IDs.size() << std::endl;
                    
//                     rangeHighSum += sensor_ranges[ IDs[iAvgHigh] ];
                    pointHighSum += segmentPoints [iAvgHigh];
            }
//             std::cout << "test4" << std::endl; 
//             if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.5 \t"; 
            geo::Vec2f avgPointHigh = pointHighSum / nPointsForAvg;
            
            bool splitFound = false;
            
             if( std::fabs( IDs[2*nPointsForAvg] - IDs[0]) <=  2*nPointsForAvg + N_POINTS_MARGIN_FOR_BEING_CONSECUTIVE &&
                 std::fabs( IDs[2*nPointsForAvg] - IDs[0]) >=  2*nPointsForAvg - N_POINTS_MARGIN_FOR_BEING_CONSECUTIVE ) // points must be consecutive
             { 
//                      std::cout << "test4.1" << std::endl; 
//                      if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.6 \t"; 
//                      std::cout << "avg ranges & abs diff = " << avgPointHigh << ", " << avgPointLow << ", " << avgPointHigh.dist( avgPointLow ) << std::endl;
                     
                if( avgPointHigh.dist( avgPointLow ) >  dist_for_object_split_ )
                {
                        
//                         std::cout << "Going to split 1, avgRangeHigh = " << avgRangeHigh << " avgRangeLow = " << avgRangeLow << "split @ " << nPointsForAvg << std::endl;
                        
//                         if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.7 \t"; 
                        PointsInfo splittedInfo;       
                        unsigned int position2Split = nPointsForAvg;//IDs[nPointsForAvg];
                        std::vector<geo::Vec2f> points = associatedPointsInfo[iList].points;             
                        std::copy( associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end(), std::back_inserter(splittedInfo.laserIDs) );
                        std::copy( associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end(),  std::back_inserter(splittedInfo.points) );
                        associatedPointsInfo.push_back( splittedInfo );
                        
//                         std::cout << "test4.2" << std::endl; 
                        
//                                                 if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.8 \t"; 
                        associatedPointsInfo[iList].laserIDs.erase (associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end() );
                        associatedPointsInfo[iList].points.erase (associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end() );
                        splitFound = true;
//                         if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.9, split @ " << position2Split  << "for iList = " << iList << "\t"; 
//                 std::cout << termcolor::magenta << "Debug 13.18.9, split @" << position2Split << "for iList = " << iList << " associatedPointsInfo[iList].laserIDs.size() = " <<  associatedPointsInfo[iList].laserIDs.size() << "\t" << termcolor::reset; 
//                 std::cout << "test5" << std::endl; 
                }
                    
//                             if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.10 " << std::endl; 
             }
//               std::cout << termcolor::magenta << "test6" << termcolor::reset << std::endl; 
//             if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.11 \t"; 
//               std::cout << "IDs.size() = " << IDs.size() << std::endl;
//               std::cout << "nPointsForAvg = " << nPointsForAvg << std::endl;
//               std::cout << "IDs.size() - 2*nPointsForAvg - 1 = " << IDs.size() - 2*nPointsForAvg - 1 << std::endl;
              
              
            for(unsigned int iIDs = 0; iIDs < (IDs.size() - 2*nPointsForAvg - 1) && !splitFound && IDs.size() >= ( 2*nPointsForAvg + 1); iIDs++) // -1 because the first segment is already determined as above
            { 
//                     std::cout << "test7" << std::endl; 
//                     if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.12 \t";
//                     std::cout << "IDs.size() = " << IDs.size() << std::endl;
//                 std::cout << "(IDs.size() - 2*nPointsForAvyg - 1) = " << (IDs.size() - 2*nPointsForAvg - 1) << std::endl;
//                 std::cout << "iIDs = " << iIDs ;
//                 std::cout << " IDs[iIDs] = " << IDs[iIDs];
//                 std::cout << " IDs.size() = " << IDs.size();
//                 std::cout << " IDs[iIDs + 2*nPointsForAvg] = " << IDs[iIDs + 2*nPointsForAvg] << std::endl;
//                 std::cout << " sensor_ranges.size() = " << sensor_ranges.size();
                
                
                    pointLowSum -= segmentPoints[iIDs];
//                     std::cout << "pointLowSum: substracted " << segmentPoints[iIDs] << "\t";
                    pointLowSum += segmentPoints[iIDs + nPointsForAvg];
                    
//                     std::cout << "pointLowSum: added " << segmentPoints[iIDs + nPointsForAvg] << "\t";
                    
//                     std::cout << "test7.0.1"<< std::endl;
//                     if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.13 \t"; 
                    pointHighSum -= segmentPoints[iIDs + nPointsForAvg];
//                       std::cout << "pointHighSum: substracted " << segmentPoints[iIDs + nPointsForAvg] << "\t";
//                     std::cout << "test7.0.1"<< std::endl;
                    pointHighSum += segmentPoints[iIDs + 2*nPointsForAvg];
//                       std::cout << "pointHighSum: added " << segmentPoints[iIDs + 2*nPointsForAvg] << "\t";
                    
//                      pointLowSum -= segmentPoints[iIDs];
// //                     std::cout << "test7.0.1"<< std::endl;
//                     pointLowSum += segmentPoints[iIDs + nPointsForAvg];
// //                     std::cout << "test7.0.1"<< std::endl;
// //                     if ( DEBUG_SF )
// //                 std::cout << "Debug 13.18.13 \t"; 
//                     pointHighSum -= segmentPoints[iIDs + nPointsForAvg];
// //                     std::cout << "test7.0.1"<< std::endl;
//                     pointHighSum += segmentPoints[iIDs + 2*nPointsForAvg];
                    
//                     if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.14 \t"; 
//                         std::cout << "test7.1" << std::endl; 
                    if( IDs[iIDs + 2*nPointsForAvg] - IDs[iIDs] ==  2*nPointsForAvg) // points must be consecutive
                    {
//                             std::cout << "test8" << std::endl; 
//                             if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.15 \t"; 
                        avgPointLow = pointLowSum / nPointsForAvg;
                        avgPointHigh = pointHighSum / nPointsForAvg;
                    
//                         if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.16 \t"; 
                        
//                           std::cout << "avg ranges & abs diff = " << avgPointHigh << ", " << avgPointLow << ", " << avgPointHigh.dist( avgPointLow ) << std::endl;
                        
                        if( avgPointHigh.dist( avgPointLow ) >  dist_for_object_split_ )
                        {
//                                 if ( DEBUG_SF )0
//                 std::cout << "Debug 13.18.17 \t"; 
                                
//                                 std::cout << "test9" << std::endl; 
                                
                                PointsInfo splittedInfo;       
                                unsigned int position2Split = iIDs;//IDs[nPointsForAvg];
                                std::vector<geo::Vec2f> points = associatedPointsInfo[iList].points; 
                                
//     std::cout << "Going to split 2, avgRangeHigh = " << avgRangeHigh << " avgRangeLow = " << avgRangeLow << "split @ " << position2Split << std::endl;
    
        for(unsigned int iPrint = 0; iPrint < associatedPointsInfo[iList].laserIDs.size(); iPrint++)
        {
                std::cout << associatedPointsInfo[iList].laserIDs[iPrint] << "\t";
                
                if(associatedPointsInfo[iList].laserIDs[iPrint]  > sensor_ranges.size())
                {
//                               std::cout << "sensor_ranges.size() = " << sensor_ranges.size() << std::endl;
                        ROS_ERROR("BAD INFORMATION 0!!");
                         exit(0);
                }
        }
                                
//                                 std::cout << "position2Split = " << position2Split << " associatedPointsInfo.laserIDs.size() = " << associatedPointsInfo[iList].laserIDs.size() << " \n";
                                std::copy( associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end(), std::back_inserter(splittedInfo.laserIDs) );
                                std::copy( associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end(),  std::back_inserter(splittedInfo.points) );
                                associatedPointsInfo.push_back( splittedInfo );
//                                 std::cout << "test9" << std::endl; 
//                                                  if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.18 \t";    
//                                                          std::cout << "associatedPointsInfo[iList].laserIDs.size() = "<<  associatedPointsInfo[iList].laserIDs.size() << " @iList = " << iList << "\t";
                                associatedPointsInfo[iList].laserIDs.erase (associatedPointsInfo[iList].laserIDs.begin() + position2Split, associatedPointsInfo[iList].laserIDs.end() );
                                associatedPointsInfo[iList].points.erase (associatedPointsInfo[iList].points.begin() + position2Split, associatedPointsInfo[iList].points.end() );
                                splitFound = true;
                                
//                                  std::cout << "position2Split = " << position2Split << " associatedPointsInfo.laserIDs.size() = " << associatedPointsInfo[iList].laserIDs.size() << " \n";
                                for(unsigned int iPrint = 0; iPrint < associatedPointsInfo[iList].laserIDs.size(); iPrint++)
                                {
//                                         std::cout << associatedPointsInfo[iList].laserIDs[iPrint] << "\t";
                                        
                                        if(associatedPointsInfo[iList].laserIDs[iPrint]  > sensor_ranges.size())
                                        {
//                                                 std::cout << "sensor_ranges.size() = " << sensor_ranges.size() << std::endl;
                                                ROS_ERROR("BAD INFORMATION 1!!");
                                                exit(0);
                                        }
                                }
                                
                                
                                for(unsigned int iPrint = 0; iPrint <splittedInfo.laserIDs.size(); iPrint++)
                                {
//                                         std::cout << splittedInfo.laserIDs[iPrint] << "\t";
                                        
                                        if(splittedInfo.laserIDs[iPrint]  > sensor_ranges.size())
                                        {
//                                                 std::cout << "sensor_ranges.size() = " << sensor_ranges.size() << std::endl;
                                                ROS_ERROR("BAD INFORMATION 2!!");
                                                exit(0);
                                        }
                                }
        
                                
                                
//                                  if ( DEBUG_SF )
//                                  std::cout << termcolor::magenta << "Debug 13.18.18.1, split @ " << position2Split << " associatedPointsInfo[iList].laserIDs.size() = " <<  associatedPointsInfo[iList].laserIDs.size() << "\t" << termcolor::reset; 
//                                  std::cout << "test10" << std::endl; 
                        }
//                         if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.19 \t"; 
                    }
//                      std::cout << "test11" << std::endl; 
//                     if ( DEBUG_SF )
//                 std::cout << "Debug 13.18.10 \t"; 
            }
//             if ( DEBUG_SF )
//                 std::cout << "End of splitting for consecutive sections" << std::endl; 
    }
    
//     std::cout << "associatedPointsInfo.size() = " << associatedPointsInfo.size() << std::endl;
    
    

//     if( DEBUG )
//             std::cout << "Debug 14 \t";
    
    // TODO check at which point the segment should be splitted
    
    std::vector<measuredPropertyInfo> measuredProperties ( associatedPointsInfo.size() ); // The first sequence in this vector (with the length of laser entitities) consits of the properties corresponding to existing entities
    
//    std::vector<ed::tracking::FeatureProperties> measuredProperties;
//     std::vector<ed::tracking::FeatureProperties> measuredProperties ( associatedPointsInfo.size() ); // The first sequence in this vector (with the length of laser entitities) consits of the properties corresponding to existing entities
//      measuredProperties.resize(  associatedPointsInfo.size() );     // The first sequence in this vector (with the length of laser entitities) are the properties corresponding to existing entities
    
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
       
       if( points.size() < min_segment_size_pixels_ )
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
       
      
        
 if ( DEBUG_SF )
               std::cout << "Debug 15 \t";
       
        std::vector<unsigned int> cornerIndices;
        std::vector<geo::Vec2f>::iterator it_start = points.begin();
        std::vector<geo::Vec2f>::iterator it_end = points.end();
        unsigned int cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
          
//  std::cout << "while processing: " << std::endl;
        if( ed::tracking::findPossibleCorner ( points, &cornerIndices, &it_start, &it_end, MIN_DISTANCE_CORNER_DETECTION, min_segment_size_pixels_ ) )
        {
                cornerIndex = cornerIndices[0];
        }
//  std::cout << "while processing2: " << std::endl;       
        for ( std::vector<unsigned int>::const_iterator it_in = cornerIndices.begin(); it_in != cornerIndices.end(); ++it_in )
        {
             const unsigned int& index = *it_in;
        }
        
 if ( DEBUG_SF )
                std::cout << "Debug 15.0 \t";

        ed::tracking::Circle circle;   
        ed::tracking::Rectangle rectangle;    
        std::vector<geo::Vec2f>::iterator it_low, it_high;
        
//         std::cout << "Debug 15.1 \t";
        ed::tracking::FITTINGMETHOD method = ed::tracking::CIRCLE;
        float errorCircle = ed::tracking::fitObject ( points, method, &cornerIndex, &rectangle, &circle, &it_low, &it_high, sensor_pose, min_segment_size_pixels_ );
        unsigned int elementLow = associatedPointsInfo[iList].laserIDs[0];
        unsigned int elementHigh = associatedPointsInfo[iList].laserIDs.back();
//         measuredProperties[iList].confidenceCircle = ed::tracking::determineSegmentConfidence ( scan, elementLow, elementHigh);
         if ( DEBUG_SF )
std::cout << "Debug 15.2 \t";
        method = ed::tracking::determineCase ( points, &cornerIndex, &it_low, &it_high, sensor_pose,  min_segment_size_pixels_); // chose to fit a single line or a rectangle (2 lines)
        float errorRectangle = ed::tracking::fitObject ( points, method,  &cornerIndex, &rectangle, &circle, &it_low, &it_high,  sensor_pose, min_segment_size_pixels_ );
         if ( DEBUG_SF )
std::cout << "Debug 15.3 \t";
        measuredProperties[iList].confidenceRectangleWidth = false;
        measuredProperties[iList].confidenceRectangleWidthLow = false;
        measuredProperties[iList].confidenceRectangleWidthHigh = false;
        measuredProperties[iList].confidenceRectangleDepth = false;
        measuredProperties[iList].confidenceRectangleDepthLow = false;
        measuredProperties[iList].confidenceRectangleDepthHigh = false;
        measuredProperties[iList].methodRectangle = method;
        measuredProperties[iList].fittingErrorCircle = errorCircle;
        measuredProperties[iList].fittingErrorRectangle = errorRectangle;
        
//         std::cout << "errorRectangle = " << errorRectangle << std::endl;
//         std::cout << "measuredProperties[iList].fittingErrorRectangle = " << measuredProperties[iList].fittingErrorRectangle << std::endl;
        
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
//         std::cout << "associatedPointsInfo[iList].laserIDs = " << std::endl;
        for(unsigned int iPrint = 0; iPrint < associatedPointsInfo[iList].laserIDs.size(); iPrint++)
        {
//                 std::cout << associatedPointsInfo[iList].laserIDs[iPrint] << "\t";
                
                if(associatedPointsInfo[iList].laserIDs[iPrint]  > sensor_ranges.size())
                {
                              std::cout << "sensor_ranges.size() = " << sensor_ranges.size() << std::endl;
                        ROS_ERROR("BAD INFORMATION!!");
                         exit(0);
                }
        }
        
//         std::cout << "associatedPointsInfo[iList].laserIDs.size() = " << associatedPointsInfo[iList].laserIDs.size() << std::endl;
//         std::cout << "IDs low & high = " << IDLowWidth << ", " << IDHighWidth << ", " << IDLowDepth << ", " << IDHighDepth << std::endl;
//         std::cout << " IDLowWidth = " << IDLowWidth << " IDHighWidth = " << IDHighWidth << " IDLowDepth = " << IDLowDepth << " IDHighDepth = " << IDHighDepth << std::endl;
         if ( DEBUG_SF )
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
                                         
//                                          std::cout << "associatedPointsInfo[iList].points.size() = " << associatedPointsInfo[iList].points.size() << ". PointIDHighWidth = " << PointIDHighWidth << " Points = " << std::endl;
//                                          for(unsigned int iPointsTest = 0; iPointsTest < associatedPointsInfo[iList].points.size(); iPointsTest++)
//                                          {
//                                                 std::cout << associatedPointsInfo[iList].points[iPointsTest] << "\t";
//                                          }
                                         
                                         
                                         
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
//          if ( DEBUG_SF )
//                     std::cout << "Debug 16.1 \t";
         
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
                 rectangle.printProperties();
                std::cout << "Prob = " << prob.get_pCircle() << ", " << prob.get_pRectangle() << std::endl;
                std::cout << "Method = " << method << std::endl;
                std::cout << "Errors = " << errorRectangle << ", " << errorCircle << std::endl;
            }
//             else
//             {
//                     std::cout << rectangle.get_yaw() << std::endl;
//             }

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
       
   if( DEBUG )
            std::cout << "Debug 17 \t";
    
    unsigned int marker_ID = 0; // To Do: After tracking, the right ID's should be created. The ID's are used to have multiple markers.

    ed::tracking::FeatureProperties measuredProperty, entityProperties; // Measured properties and updated properties
    ed::UUID id;
//     std::vector<ed::WorldModel::const_iterator> it_laserEntities; 

    if( DEBUG )
            std::cout << "Debug 18 \t";
    
    visualization_msgs::MarkerArray markers;
    unsigned int ID = 100;
    
    for ( unsigned int iProperties = 0; iProperties < measuredProperties.size(); iProperties++ ) // Update associated entities
    {
                if( DEBUG )
            std::cout << "Debug 18.1 \t";
            if(!measuredProperties[iProperties].propertiesDescribed ) 
                    continue;
            
                if( DEBUG )
            std::cout << "Debug 18.2 \t";
            
        measuredProperty = measuredProperties[iProperties].featureProperty;
        
//         std::cout << "measuredProperties[iProperties]..fittingErrorRectangle" << measuredProperties[iProperties].fittingErrorRectangle << std::endl;
//         std::cout << "Method = " << measuredProperties[iProperties].methodRectangle << std::endl;
        
        // temporary: pub measured properties in order to visualize the properties which are added
        markers.markers.push_back( getMarker(measuredProperty, ID++) ); // TEMP

        
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
                    std::cout << "Debug zoveel " << std::endl;

            if ( !e->hasFlag ( "locked" ) )
            {
                entityProperties = e->property ( featureProperties_ );
                ed::tracking::Rectangle entityRectangle = entityProperties.getRectangle();
                ed::tracking::Circle entityCircle = entityProperties.getCircle();
                
//                 std::cout << "\nFor entity " << termcolor::blue << e->id() << termcolor::reset << std::endl;
                
                float Q = 0.4; // Measurement noise covariance. TODO: let it depend on if an object is partially occluded. Now, objects are assumed to be completely visible
                float R = 0.2; // Process noise covariance
                float RVariable = 1000000*R*pow(measuredProperties[iProperties].fittingErrorRectangle, 2.0);
                float largeCovariance = 100000.0;
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
//                 RmRectangle.diagonal() << R, R, R, R, R;
                      RmRectangle.diagonal() << R, R, RVariable, R, R;
                      
//                 std::cout << "RVariable = " << RVariable << std::endl;
//                 std::cout << "measuredProperties[iProperties].fittingErrorRectangle = " << measuredProperties[iProperties].fittingErrorRectangle << std::endl;
                
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
                
             /*         std::cout << "measuredProperties = " << 
                        measuredProperties[iProperties].confidenceRectangleWidthLow <<
                        measuredProperties[iProperties].confidenceRectangleWidthHigh <<
                        measuredProperties[iProperties].confidenceRectangleDepthLow <<
                        measuredProperties[iProperties].confidenceRectangleDepthHigh << std::endl;
                 */     
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
                
//                 if( DEBUG )
//                         std::cout << "Test 3 \t";
               
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
                
//                 if( DEBUG )
//                         std::cout << "Test 4 \t";
                
         /*       
                 std::cout << "Before update: " << std::endl;
                 entityProperties.rectangle_.printProperties();
                std::cout << " \n Measurement: \t" ;
                measuredProperty.rectangle_.printProperties();
           */     
                entityProperties.updateRectangleFeatures(QmRectangle, RmRectangle, zmRectangle, dt, sensor_pose);
             /*
                std::cout << "\nAfter update: \t";
                entityProperties.rectangle_.printProperties();
               */ 

                
                // update circular properties
                Eigen::MatrixXf QmCircle = Eigen::MatrixXf::Zero( 5, 5 );
                Eigen::MatrixXf RmCircle = Eigen::MatrixXf::Zero( 3, 3 );
                
//                 if( DEBUG )
//                         std::cout << "Test 5 \t";
                QmCircle.diagonal() << Q, Q, Q, Q, Q;
                RmCircle.diagonal() << R, R, R;
                
                // TODO what to do with position information if there is low confidence in with and depth? Position information should be updated with respect to an anchor point!
                if( measuredProperties[iProperties].confidenceRectangleDepth == false )
                {
                        QmCircle( 4, 4 ) = largeCovariance;
//                         std::cout << " QmCircle = " << QmCircle << std::endl;
                } 
                
//                 if( DEBUG )
//                         std::cout << "Test 6 \t";
                Eigen::VectorXf zmCircle( 3 );
                zmCircle <<
                measuredProperty.getCircle().get_x(),
                measuredProperty.getCircle().get_y(),
                measuredProperty.getCircle().get_radius();
                

                
                
//                 if( DEBUG )
//                         std::cout << "Test 7 \t";
                entityProperties.updateCircleFeatures(QmCircle, RmCircle, zmCircle, dt);
//                 if( DEBUG )
//                         std::cout << "Test 8 \t";
                entityProperties.updateProbabilities ( measuredProperty.getFeatureProbabilities() );
                
//                 std::cout << "Entity Property after = " << std::endl; entityProperties.printProperties();
                
//                 if( DEBUG )
//                         std::cout << "Test 9 \t";
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
        //        std::cout << termcolor::magenta << "Going to set prop for entity with id = " << id << " having prop " << termcolor::reset << std::endl;
        //        entityProperties.printProperties();
                
//                 std::cout << "Tracking plugin updates: id = " << id << std::endl;
//                 entityProperties.printProperties();
            req.setProperty ( id, featureProperties_, entityProperties );
//             measuredProperty.getRectangle().printProperties();
//             std::cout << "Measured yaw = " << measuredProperty.getRectangle().get_yaw() << std::endl;
            std::cout << "\n";
//             std::cout << "Prop of ent. " << id << " updated with properties = "; entityProperties.printProperties(); 
//            std::cout << " measuredProperty of ent " << id << " = "; measuredProperty.printProperties();
            
            req.setPose ( id, new_pose );

            // Set timestamp
            req.setLastUpdateTimestamp ( id, scan->header.stamp.toSec() );
            req.setExistenceProbability ( id, existenceProbability );
        }
    }
    
    ObjectMarkers_pub_.publish(markers);


// - - - - - - - - - - - - - - - - -

//     std::cout << "tracking plugin: Total took " << t_total.getElapsedTimeInMilliSec() << " ms. \n\n\n" << std::endl;
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
