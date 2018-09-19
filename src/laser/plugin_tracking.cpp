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

namespace
{

typedef std::vector<unsigned int> ScanSegment;

struct ScanSegmentInfo
{
    ScanSegment segmentRanges;
    bool confidenceLeft;
    bool confidenceRight;
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

/*
visualization_msgs::Marker getMarker ( ed::tracking::FeatureProperties& featureProp, int ID, bool possiblyMobidik ) // TODO move to ed_rviz_plugins?
{
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;
    
    if ( possiblyMobidik )
    {
        color.r = 0;
        color.g = 0;
        color.b = 1;
        color.a = ( float ) 0.5;
        ed::tracking::Rectangle rectangle = featureProp.getRectangle();
        rectangle.setMarker ( marker , ID, color, "Mobidik" );
        
    }
    else
    {
        color.r = 0;
        color.g = 1;
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
    }
    
    return marker;
}
*/

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
//    ObjectMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/objectMarkers", 3 );

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
    
    std::cout << " Process started with buffer size of" << scan_buffer_.size() << std::endl;

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
            sensor_ranges[i] = sensor_ranges[i - 1];
        }
    }
    std::cout << "Debug 2 \t";

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
std::cout << "Debug 3 \t";
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
    
    std::cout << "Debug 4 \t";

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    std::vector<float> sensor_rangesOriginal = sensor_ranges;
    
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        if (rs <= 0
                || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                || (std::abs(rm - rs) < world_association_distance_))
            sensor_ranges[i] = 0;
    }

    // - - - - - - - - - - - - - - - - - -
    // Segment the remaining points into clusters

    std::cout << "Debug 5 \t";
    
    std::vector<ScanSegmentInfo> segments;

    // Find first valid value
     ScanSegmentInfo currentSegmentInfo;
    bool confidenceLeft; // check if the object might have been covered by an object on both sides to determine the confidence of the measurement
    bool confidenceRight;
    
// TODO: confidence low/high should be compared to original data!
    for ( unsigned int i = 0; i < num_beams - 1; ++i )
    {
        if ( sensor_ranges[i] > 0 )
        {
            currentSegmentInfo.segmentRanges.push_back ( i );
            if ( i == 0 )
            {
                confidenceLeft = 0; // Because we have no proof that the complete side of the object is observed
            }
            else
            {
                confidenceRight = 1;
            }
            break;
        }
    }
    
    std::cout << "Debug 6 \t";

    if ( currentSegmentInfo.segmentRanges.empty() )
    {
        return;
    }

    
    std::cout << "Debug 7 \t";
    int gap_size = 0;
    std::vector<float> gapRanges;

    for(unsigned int i = currentSegmentInfo.segmentRanges.front(); i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];

        if (rs == 0 || std::abs(rs - sensor_ranges[currentSegmentInfo.segmentRanges.back()]) > segment_depth_threshold_ || i == num_beams - 1)
        {
            // Found a gap
            ++gap_size;
            gapRanges.push_back ( rs );

            if (gap_size >= max_gap_size_ || i == num_beams - 1)
            {
                 i = currentSegmentInfo.segmentRanges.back() + 1;

                if (currentSegmentInfo.segmentRanges.size()  >= min_segment_size_pixels_)
                {
                    // calculate bounding box
                    geo::Vec2 seg_min, seg_max;
                    for(unsigned int k = 0; k <  currentSegmentInfo.segmentRanges.size(); ++k)
                    {
                        geo::Vector3 p = lrf_model_.rayDirections()[ currentSegmentInfo.segmentRanges[k]] * sensor_ranges[currentSegmentInfo.segmentRanges[k]];

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
                        confidenceRight = true;
                        for ( unsigned int l = currentSegmentInfo.segmentRanges.size() - POINTS_TO_CHECK_CONFIDENCE; confidenceRight && l < currentSegmentInfo.segmentRanges.size(); l++ )
                        {
                            for ( unsigned int m = 0; confidenceRight && m < gapRanges.size(); m++ )
                            {
                                bool check = gapRanges[m] < currentSegmentInfo.segmentRanges[l] ;
                                bool check2 = gapRanges[m] >= 0 + EPSILON;
                                if ( gapRanges[m] < sensor_rangesOriginal[currentSegmentInfo.segmentRanges[l]] && gapRanges[m] >= 0 + EPSILON)
                                {
                                    confidenceRight = false;
                                }
                            }
                        }

                        currentSegmentInfo.confidenceLeft = confidenceLeft;
                        currentSegmentInfo.confidenceRight = confidenceRight;

                        segments.push_back ( currentSegmentInfo );
                    }   
                }

                currentSegmentInfo.segmentRanges.clear();
                gapRanges.clear();

                // Find next good value
                while ( sensor_ranges[i] == 0 && i < num_beams )
                {
                    ++i; // check for confidence low
                }

                int nPointsToCheck = POINTS_TO_CHECK_CONFIDENCE;
                if ( i < nPointsToCheck )
                {
                    nPointsToCheck = i;
                }

                confidenceLeft = true;
                float rsToCheck = sensor_ranges[i];
                for ( unsigned int l = i - nPointsToCheck; confidenceLeft && l < i; l++ )
                {
                    float rsToCompare = sensor_rangesOriginal[l];
                    bool check = rsToCheck > rsToCompare;
                    bool check2 = rsToCompare <= 0 + EPSILON;
                    if ( rsToCheck > rsToCompare && rsToCompare >= 0 + EPSILON )
                    {
                        confidenceLeft = false;
                    }
                }

                currentSegmentInfo.segmentRanges.push_back ( i );
            }
        }
        else
        {
            gap_size = 0;
            gapRanges.clear();
            currentSegmentInfo.segmentRanges.push_back ( i );
        }
    }
    
    std::cout << "Debug 8 \t";
    
    // Try to associate remaining laser points to specific entities
    std::vector<ed::WorldModel::const_iterator> it_laserEntities;
    std::vector< EntityProperty > EntityPropertiesForAssociation;

    // Check which entities might associate for tracking based on their latest location
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
//                 circle.predictAndUpdatePos(dt);
                
                currentProperty.entity_min.x = circle.get_x() - ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_max.x = circle.get_x() + ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_min.y = circle.get_y() - ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
                currentProperty.entity_max.y = circle.get_y() + ( 0.5*ASSOCIATION_DISTANCE + circle.get_radius() );
            }
            else
            {
                ed::tracking::Rectangle rectangle = featureProperties.getRectangle();
//                 rectangle.predictAndUpdatePos(dt);
                
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
    
    std::cout << "Debug 9 \t";
    
     std::vector< std::vector<geo::Vec2f> > associatedPointsList ( it_laserEntities.size() );
     
//      std::cout << "segments.size() = " << segments.size() << std::endl;
    
    for ( unsigned int iSegments = 0; iSegments < segments.size(); ++iSegments )
    {
        // First, determine the properties of each segment
        ScanSegment& segment = segments[iSegments].segmentRanges;
        unsigned int segment_size = segment.size();

        std::vector<geo::Vec2f> points ( segment_size );
        geo::Vec2f seg_min, seg_max;

        std::cout << "Debug 10 \t";
        
//         std::cout << "Points in segment " << iSegments << " = " << std::endl;
        for ( unsigned int iSegment = 0; iSegment < segment_size; ++iSegment )
        {
            unsigned int j = segment[iSegment];
            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[iSegment] = geo::Vec2f ( p.x, p.y );
            
//             std::cout << points[iSegment] << std::endl;
            
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
        
//         std::cout << "Segm min/mx = " <<   seg_min.x << ", " << 
//         seg_min.y << ", " << 
//         seg_max.x << ", " << 
//         seg_max.y << std::endl;

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

    
    std::cout << "Debug 13 \t";
        // If a cluster could be associated to a (set of) entities, determine for each point to which entitiy it belongs based on a shortest distance criterion. 
        // If the distance is too large, initiate a new entity
        std::vector<geo::Vec2f> pointsNotAssociated;
        std::vector<float> distances ( points.size() );
        std::vector<unsigned int> IDs ( points.size() ); // IDs of the entity which is closest to that point

        for ( unsigned int i_points = 0; i_points < points.size(); ++i_points )  // Determine closest object and distance to this object. If distance too large, relate to new object
        {
            geo::Vec2f p = points[i_points];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = std::numeric_limits< unsigned int >::max();

            for ( unsigned int jj = 0; jj < possibleSegmentEntityAssociations.size(); ++jj )  // relevant entities only
            {
                const ed::EntityConstPtr& e = *it_laserEntities[ possibleSegmentEntityAssociations[jj] ];
                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
                float dist;
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                
                if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() )  // entity is considered to be a circle
                {
                    ed::tracking::Circle circle = featureProperties.getCircle();  
//                     circle.predictAndUpdatePos( dt );
                    dist = std::abs ( std::sqrt ( std::pow ( p.x - circle.get_x(), 2.0 ) + std::pow ( p.y - circle.get_y(), 2.0 ) ) - circle.get_radius() ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                }
                else     // entity is considered to be a rectangle. Check if point is inside the rectangle
                {
                    ed::tracking::Rectangle rectangle = featureProperties.getRectangle();
//                     rectangle.predictAndUpdatePos( dt );

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

        for ( unsigned int iDistances = 1; iDistances < distances.size(); iDistances++ )
        {
            if ( IDs[iDistances] == IDtoCheck && iDistances != distances.size() - 1 ) // ID similar and not at final reading, check next element
            {
                continue;
            }

            unsigned int length = iDistances - firstElement;
           // bool test = length < min_segment_size_pixels_ ;

            if ( length >= min_segment_size_pixels_ )
            {
                float minDistance = distances[firstElement];
                for ( unsigned int iiDistances = 1; iiDistances < iDistances; iiDistances++ )
                {
                    if ( distances[firstElement] < minDistance )
                    {
                        minDistance = distances[iiDistances];
                    }
                }

                if ( minDistance < MIN_ASSOCIATION_DISTANCE )  // add all points to associated entity
                {
                    const ed::EntityConstPtr& entityToTest = *it_laserEntities[ possibleSegmentEntityAssociations[IDtoCheck] ];
                    for ( unsigned int i_points = firstElement; i_points < iDistances; ++i_points )
                    {
                        associatedPointsList.at ( possibleSegmentEntityAssociations[IDtoCheck] ).push_back ( points[i_points] );
                    }
                }
                else
                {
                    pointsNotAssociated.clear();
                    for ( unsigned int i_points = firstElement; i_points < iDistances; ++i_points )
                    {
                        pointsNotAssociated.push_back ( points[i_points] );
                    }
                    associatedPointsList.push_back ( pointsNotAssociated );
                }
            }
            firstElement = iDistances;
            IDtoCheck = IDs[iDistances];
        }

    }
    
    std::cout << "Debug 14 \t";
    
    // TODO check at which point the segment should be splitted
    
//    std::vector<ed::tracking::FeatureProperties> measuredProperties;
    std::vector<ed::tracking::FeatureProperties> measuredProperties ( associatedPointsList.size() ); // The first sequence in this vector (with the length of laser entitities) are the properties corresponding to existing entities
   
    std::cout << "associatedPointsList.size() = " << associatedPointsList.size() << std::endl;
    
    std::vector<bool> propertiesDescribed( associatedPointsList.size(), false );
   for ( unsigned int iList = 0; iList < associatedPointsList.size(); iList++ )
//     for ( std::vector<ScanSegmentInfo>::const_iterator it = segments.begin(); it != segments.end(); ++it )
    {
//         const ScanSegmentInfo& segment = *it; // TODO make it a scansegment with the info of confidence high and low
        //std::vector<geo::Vec2f> points ( segment.segmentRanges.size() );
       std::vector<geo::Vec2f> points  = associatedPointsList[iList] ;
       
//        std::cout << "points.size() = " << points.size() <<  std::endl;
       
       if( points.size() == 0 )
               continue;
       
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
        
        std::cout << "Debug 16 \t";

        ed::tracking::Circle circle;   
        ed::tracking::Rectangle rectangle;    
        std::vector<geo::Vec2f>::iterator it_low, it_high;
        
        ed::tracking::FITTINGMETHOD method = ed::tracking::CIRCLE;
        float error_circle2 = ed::tracking::fitObject ( points, method, &cornerIndex, &rectangle, &circle, &it_low, &it_high, sensor_pose );

        method = ed::tracking::determineCase ( points, &cornerIndex, &it_low, &it_high, sensor_pose ); // chose to fit a single line or a rectangle (2 lines)        
        float error_rectangle2 = ed::tracking::fitObject ( points, method,  &cornerIndex, &rectangle, &circle, &it_low, &it_high,  sensor_pose );

        ed::tracking::FeatureProbabilities prob;
        prob.setMeasurementProbabilities ( error_rectangle2, error_circle2,2*circle.get_radius() , nominal_corridor_width_ );

        ed::tracking::FeatureProperties properties;
        properties.setFeatureProbabilities ( prob );
        properties.setCircle ( circle );
        properties.setRectangle ( rectangle );
        measuredProperties[iList] =  properties ;        
        propertiesDescribed[iList] = true;
    }  
    
    
    std::cout << "Debug 17 \t";
    
    unsigned int marker_ID = 0; // To Do: After tracking, the right ID's should be created. The ID's are used to have multiple markers.

    ed::tracking::FeatureProperties measuredProperty, entityProperties; // Measured properties and updated properties
    ed::UUID id;
//     std::vector<ed::WorldModel::const_iterator> it_laserEntities; // TODO should contain all entities described by the laser

    std::cout << "Debug 18 \t";
    
    for ( unsigned int iProperties = 0; iProperties < measuredProperties.size(); iProperties++ ) // Update associated entities
    {
            if(!propertiesDescribed[iProperties] ) 
                    continue;
            
        measuredProperty = measuredProperties[iProperties];

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
            
            std::cout << "Debug 1 " << std::endl;

            if ( !e->hasFlag ( "locked" ) )
            {
                entityProperties = e->property ( featureProperties_ );
                ed::tracking::Rectangle entityRectangle = entityProperties.getRectangle();
                ed::tracking::Circle entityCircle = entityProperties.getCircle();
                
                float Q = 0.1; // Measurement noise covariance. TODO: let it depend on if an object is partially occluded. Now, objects are assumed to be completely visible
                float R = 0.2; // Process noise covariance
                float dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();
                std::cout << "Test 1  \t";
                
                // update rectangular properties
                Eigen::MatrixXd QmRectangle = Eigen::MatrixXd::Zero( 8, 8 );
                Eigen::MatrixXd RmRectangle = Eigen::MatrixXd::Zero( 5,5 );
                
                std::cout << "Test 2 \t";
                QmRectangle.diagonal() << Q, Q, Q, Q, Q, Q, Q, Q;
                RmRectangle.diagonal() << R, R, R, R, R;
                
                std::cout << "Test 3 \t";
                
                // As there are model differences between the width and the depth of the entity and the latest measurement, the position information should be corrected for that
                
                float thetaPred = entityRectangle.get_yaw() + dt*entityRectangle.get_yawVel();
                float deltaWidth =  entityRectangle.get_w() - measuredProperty.getRectangle().get_w();
                float deltaDepth = entityRectangle.get_d() - measuredProperty.getRectangle().get_d();
                
                float st = std::sin( thetaPred );
                float ct = std::sin( thetaPred );
                
                float deltaX =   deltaWidth * ct + deltaDepth * st;
                float deltaY = - deltaWidth * st + deltaDepth * ct;
                      
                Eigen::VectorXd zmRectangle( 5 );
                zmRectangle <<  
                measuredProperty.getRectangle().get_x() + deltaX, 
                measuredProperty.getRectangle().get_y() + deltaY, 
                measuredProperty.getRectangle().get_yaw(),  
                measuredProperty.getRectangle().get_w(), 
                measuredProperty.getRectangle().get_d();
                
                std::cout << "Test 4 \t";
                entityProperties.updateRectangleFeatures(QmRectangle, RmRectangle, zmRectangle, dt);
             
                // update circular properties
                Eigen::MatrixXd QmCircle = Eigen::MatrixXd::Zero( 5, 5 );
                Eigen::MatrixXd RmCircle = Eigen::MatrixXd::Zero( 3, 3 );
                
                std::cout << "Test 5 \t";
                QmCircle.diagonal() << Q, Q, Q, Q, Q;
                RmCircle.diagonal() << R, R, R;
                
                std::cout << "Test 6 \t";
                Eigen::VectorXd zmCircle( 3 );
                zmCircle <<
                measuredProperty.getCircle().get_x(),
                measuredProperty.getCircle().get_y(),
                measuredProperty.getCircle().get_radius();
                
                std::cout << "Test 7 \t";
                entityProperties.updateCircleFeatures(QmCircle, RmCircle, zmCircle, dt);
                
                std::cout << "Test 8 \t";
                entityProperties.updateProbabilities ( measuredProperty.getFeatureProbabilities() );
                
                std::cout << "Test 9 \t";
//                 float Q = 0.1; // Measurement noise covariance. TODO: let it depend on if an object is partially occluded. Now, objects are assumed to be completely visible
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

                // TODO determine properties of updated entities instead of current measurements

          //      std::cout << "Association with entity " << e->id() << " need to update properties" << std::endl;

            }

            // Update existence probability
//             entityProperties = measuredProperty; // TODO temporary, need to update this with KF-update
            double p_exist = e->existenceProbability();
            existenceProbability = std::min ( 1.0, p_exist + 0.1 ) ;// TODO: very ugly prob update
            id = e->id();

        }
        else
        {
            // create a new entity
            // Generate unique ID
            id = ed::Entity::generateID().str() + "-laserTracking";
          //  std::cout << "New entity created with ID = " << id  << "from scan with timestamp = " << scan->header.stamp.toSec() << ". Pos = " << measuredProperty.getRectangle().get_x() << ", " << measuredProperty.getRectangle().get_y() << std::endl;

            // Update existence probability
            existenceProbability = 1.0; // TODO magic number
            entityProperties = measuredProperty;
            
//             std::cout << "For id = " << std::endl;
        }
        geo::Pose3D new_pose;

        if ( entityProperties.getFeatureProbabilities().get_pCircle() < entityProperties.getFeatureProbabilities().get_pRectangle() )
        {
            // determine corners
            ed::tracking::Rectangle rectangle = entityProperties.getRectangle();
//             std::cout << "For id = " << id << " properties are: " ; rectangle.printValues();
            //   std::vector<geo::Vec2f> corners = entityProperties.getRectangle().determineCorners ( 0.0 );
            new_pose = rectangle.getPose();
        }
        else
        {
            // determine cilinder-properties
            ed::tracking::Circle circle = entityProperties.getCircle();
//             std::cout << "For id = " << id << " properties are: " ;  circle.printProperties();
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
    
    // TODO: determine mobidik-properties somewhere else
    
    // Publish the fitted segments on the ObjectMarkers_pub_-topic // TODO: communicate via ED
    //-------------------------------------------------------------------------------------
/*     
     for ( int ii = 0; ii < measuredProperties.size(); ii++ )
    {
        ed::tracking::FeatureProperties property = measuredProperties[ii];
        bool possiblyMobidik = false;        
        
        if ( property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle() && // Dimension check
                property.rectangle_.get_d() < MOBIDIK_WIDTH + MOBIDIK_MARGIN &&
                property.rectangle_.get_w() < MOBIDIK_WIDTH + MOBIDIK_MARGIN &&
                ( property.rectangle_.get_d() > MOBIDIK_WIDTH - MOBIDIK_MARGIN ||
                  property.rectangle_.get_w() > MOBIDIK_WIDTH - MOBIDIK_MARGIN ) )
        {

            for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
            {
                const ed::EntityConstPtr& e = *it;

                std::string MobiDikWaitingAreaID = "MobidikArea";

                if ( e->id().str().length() < MobiDikWaitingAreaID.length() )
                {
                    continue;
                }

                if ( e->id().str().substr ( 0, MobiDikWaitingAreaID.length() ) == MobiDikWaitingAreaID )
                {
                        // It is assumed here that there is a navigation task, so only points on the ground are taken into consideration
                        
                        std::vector<geo::Vector3> points = e.get()->shape().get()->getMesh().getPoints();
                        std::vector<geo::Vector3> groundPoints;
                        const geo::Vec3T<double> pose = e.get()->pose().getOrigin();
                        
                        for(unsigned int iPoints = 0; iPoints < points.size(); iPoints++)
                        {
                                if(points[iPoints].getZ() == 0)
                                {
                                        groundPoints.push_back( points[iPoints] + pose );
                                }
                        }        
                       geo::Vector3 mobidikPoint( property.getRectangle().get_x(), property.getRectangle().get_y(), property.getRectangle().get_z() );
                       
                    if( isInside( groundPoints, mobidikPoint) )
                    {
                        possiblyMobidik = true;
                    }
                }   
            }
        }
        
        visualization_msgs::Marker marker = getMarker ( property, ii, possiblyMobidik ); // TODO make an entity within ED of the object (correct data-association!!) and do a query via a plugin if objects of a certain type are required
        markerArray.markers.push_back( marker );
        
    }
    */
    
// - - - - - - - - - - - - - - - - -

    std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms. \n\n\n" << std::endl;
}

// ----------------------------------------------------------------------------------------------------


void LaserPluginTracking::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    scan_buffer_.push(msg);
}


ED_REGISTER_PLUGIN(LaserPluginTracking)
