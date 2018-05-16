#include "plugin.h"

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

#include "problib/conversions.h"

// Property info
#include "ed/featureProperties_info.h"

namespace
{

typedef std::vector<unsigned int> ScanSegment;

struct ScanSegmentInfo
{
    ScanSegment segmentRanges;
    bool confidenceLow;
    bool confidenceHigh;
};

struct EntityUpdate
{

    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

struct EntityProperty
{
    geo::Vec2f entity_min;
    geo::Vec2f entity_max;
};

// ----------------------------------------------------------------------------------------------------

double getFittingError ( const ed::Entity& e, const geo::LaserRangeFinder& lrf, const geo::Pose3D& rel_pose,
                         const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                         int& num_model_points )
{
    std::vector<double> test_model_ranges = model_ranges;

    // Render the entity with the given relative pose
    geo::LaserRangeFinder::RenderOptions opt;
    opt.setMesh ( e.shape()->getMesh(), rel_pose );

    geo::LaserRangeFinder::RenderResult res ( test_model_ranges );
    lrf.render ( opt, res );

    int n = 0;
    num_model_points = 0;
    double total_error = 0;
    for ( unsigned int i = 0; i < test_model_ranges.size(); ++i )
    {
        double ds = sensor_ranges[i];
        double dm = test_model_ranges[i];

        if ( ds <= 0 )
        {
            continue;
        }

        ++n;

        if ( dm <= 0 )
        {
            total_error += 0.1;
            continue;
        }

        double diff = std::abs ( ds - dm );
        if ( diff < 0.1 )
        {
            total_error += diff;
        }
        else
        {
            if ( ds > dm )
            {
                total_error += 1;
            }
            else
            {
                total_error += 0.1;
            }
        }

        ++num_model_points;
    }

    return total_error / ( n+1 ); // to be sure to never divide by zero.
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D getPoseFromCache ( const ed::Entity& e, std::map<ed::UUID,geo::Pose3D>& pose_cache )
{
    const ed::UUID ID = e.id();
    geo::Pose3D old_pose = e.pose();
    if ( pose_cache.find ( ID ) == pose_cache.end() )
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

geo::Pose3D fitEntity ( const ed::Entity& e, const geo::Pose3D& sensor_pose, const geo::LaserRangeFinder& lrf,
                        const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                        float x_window, float x_step, float y_window, float y_step, float yaw_min, float yaw_plus, float yaw_step, std::map<ed::UUID,geo::Pose3D>& pose_cache )
{
    const geo::Pose3D& old_pose = getPoseFromCache ( e, pose_cache );

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    double min_error = 1e6;
    geo::Pose3D best_pose = e.pose();


    for ( float dyaw = yaw_min; dyaw <= yaw_plus; dyaw += yaw_step )
    {
        geo::Mat3 rot;
        rot.setRPY ( 0, 0, dyaw );
        geo::Pose3D test_pose = old_pose;
        test_pose.R = old_pose.R * rot;

        for ( float dx = -x_window; dx <= x_window; dx += x_step )
        {
            test_pose.t.x = old_pose.t.x + dx;
            for ( float dy = -y_window; dy <= y_window; dy += y_step )
            {
                test_pose.t.y = old_pose.t.y + dy;

                int num_model_points;
                double error = getFittingError ( e, lrf, sensor_pose_inv * test_pose, sensor_ranges, model_ranges, num_model_points );

                if ( error < min_error && num_model_points >= 3 )
                {
                    best_pose = test_pose;
                    min_error = error;
                }
            }
        }
    }

    return best_pose;
}


// ----------------------------------------------------------------------------------------------------

bool pointIsPresent ( double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges )
{

    int i_beam = lrf.getAngleUpperIndex ( x_sensor, y_sensor );
    if ( i_beam < 0 || i_beam >= sensor_ranges.size() )
    {
        return true;    // or actually, we don't know
    }

    float rs = sensor_ranges[i_beam];
    return rs == 0 || geo::Vec2 ( x_sensor, y_sensor ).length() > rs - 0.1;
}

// ----------------------------------------------------------------------------------------------------

bool pointIsPresent ( const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges )
{
    return pointIsPresent ( p_sensor.x, p_sensor.y, lrf, sensor_ranges );
}

}

LaserPlugin::LaserPlugin() : tf_listener_ ( 0 )
{
}

// ----------------------------------------------------------------------------------------------------

LaserPlugin::~LaserPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::initialize ( ed::InitData& init )
{
    tue::Configuration& config = init.config;

    std::string laser_topic;
    config.value ( "laser_topic", laser_topic );
    config.value ( "world_association_distance", world_association_distance_ );
    config.value ( "min_segment_size_pixels", min_segment_size_pixels_ );
    config.value ( "segment_depth_threshold", segment_depth_threshold_ );
    config.value ( "min_cluster_size", min_cluster_size_ );
    config.value ( "max_cluster_size", max_cluster_size_ );
    config.value ( "max_gap_size", max_gap_size_ );
    config.value ( "entity_timeout", entity_timeout_ );



    int i_fit_entities = 0;
    config.value ( "fit_entities", i_fit_entities, tue::OPTIONAL );
    fit_entities_ = ( i_fit_entities != 0 );

    int i_check_door_status = 0;
    config.value ( "check_door_status", i_check_door_status, tue::OPTIONAL );
    check_door_status_ = ( i_check_door_status != 0 );

    if ( config.hasError() )
    {
        return;
    }

    ros::NodeHandle nh;
    nh.setCallbackQueue ( &cb_queue_ );

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan> ( laser_topic, 3, &LaserPlugin::scanCallback, this );
    tf_listener_ = new tf::TransformListener;

    init.properties.registerProperty ( "Feature", featureProperties_, new FeaturPropertiesInfo ); // example given in ED/ed/examples/custom_properties. Update the probabilities using an update-request
    // TODO defined in multiple places now
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::process ( const ed::WorldModel& world, ed::UpdateRequest& req )
{
    cb_queue_.callAvailable();

    while ( !scan_buffer_.empty() )
    {
        sensor_msgs::LaserScan::ConstPtr scan = scan_buffer_.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF

        try
        {
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform ( "map", scan->header.frame_id, scan->header.stamp, t_sensor_pose );
            scan_buffer_.pop();
            geo::Pose3D sensor_pose;
            geo::convert ( t_sensor_pose, sensor_pose );
            update ( world, scan, sensor_pose, req );
        }
        catch ( tf::ExtrapolationException& ex )
        {
            ROS_WARN_STREAM_DELAYED_THROTTLE ( 10, "ED Laserplugin: " << ex.what() );
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform ( "map", scan->header.frame_id, ros::Time ( 0 ), latest_transform );

                if ( scan_buffer_.front()->header.stamp > latest_transform.stamp_ )
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
            catch ( tf::TransformException& exc )
            {
                scan_buffer_.pop();
            }
        }
        catch ( tf::TransformException& exc )
        {
            ROS_ERROR_STREAM_DELAYED_THROTTLE ( 10, "ED Laserplugin: " << exc.what() );
            scan_buffer_.pop();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::update ( const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                           const geo::Pose3D& sensor_pose, ed::UpdateRequest& req )
{
    std::cout << "Start of plugin" << std::endl;
    tue::Timer t_total;
    t_total.start();
    double current_time = ros::Time::now().toSec();

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges ( scan->ranges.size() );
    for ( unsigned int i = 0; i < scan->ranges.size(); ++i )
    {
        float r = scan->ranges[i];
        if ( r > scan->range_max )
        {
            sensor_ranges[i] = r;
        }
        else if ( r == r && r > scan->range_min )
        {
            sensor_ranges[i] = r;
        }
        else
        {
            sensor_ranges[i] = 0;
        }
    }

    unsigned int num_beams = sensor_ranges.size();

    if ( lrf_model_.getNumBeams() != num_beams )
    {
        lrf_model_.setNumBeams ( num_beams );
        lrf_model_.setAngleLimits ( scan->angle_min, scan->angle_max );
        lrf_model_.setRangeLimits ( scan->range_min, scan->range_max );
    }

    // - - - - - - - - - - - - - - - - - -
    // Filter laser data (get rid of ghost points)

    for ( unsigned int i = 1; i < num_beams - 1; ++i )
    {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if ( std::abs ( rs - sensor_ranges[i - 1] ) > 0.1 && std::abs ( rs - sensor_ranges[i + 1] ) > 0.1 )   // TODO: magic number
        {
            sensor_ranges[i] = sensor_ranges[i - 1];
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    std::vector<double> model_ranges ( num_beams, 0 );
    int nEntities = world.numEntities();
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;
        if ( e->shape() && e->has_pose() && ! ( e->hasType ( "left_door" ) || e->hasType ( "door_left" ) || e->hasType ( "right_door" ) || e->hasType ( "door_right" ) || e->hasFlag ( "non-localizable" ) ) )
        {
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * e->pose() );

            geo::LaserRangeFinder::RenderResult res ( model_ranges );
            lrf_model_.render ( opt, res );
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Fit the doors

    if ( fit_entities_ )
    {
        for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
        {
            const ed::EntityConstPtr& e = *it;
            if ( !e->shape() || !e->has_pose() )
            {
                continue;
            }

            geo::Pose3D e_pose_SENSOR = sensor_pose_inv * e->pose();

            // If not in sensor view, continue
            if ( e_pose_SENSOR.t.length2() > 5.0 * 5.0 || e_pose_SENSOR.t.x < 0 )
            {
                continue;
            }

            if ( e->hasType ( "left_door" ) || e->hasType ( "door_left" ) )
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity ( *e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache );
                req.setPose ( e->id(), new_pose );
                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * new_pose );

                geo::LaserRangeFinder::RenderResult res ( model_ranges );
                lrf_model_.render ( opt, res );
            }
            else if ( e->hasType ( "right_door" ) || e->hasType ( "door_right" ) )
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity ( *e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache );
                req.setPose ( e->id(), new_pose );

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * new_pose );

                geo::LaserRangeFinder::RenderResult res ( model_ranges );
                lrf_model_.render ( opt, res );
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    for ( unsigned int i = 0; i < num_beams; ++i )
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        if ( rs <= 0
                || ( rm > 0 && rs > rm ) // If the sensor point is behind the world model, skip it
                || ( std::abs ( rm - rs ) < world_association_distance_ ) )
        {
            sensor_ranges[i] = 0;
        }
    }

// - - - - - - - - - - - - - - - - - -
// Segment the remaining points into clusters

    std::vector<ScanSegmentInfo> segments;

// Find first valid value
    ScanSegmentInfo currentSegmentInfo;
    bool confidenceLow; // check if the object might have been covered by an object on both sides to determine the confidence of the measurement
    bool confidenceHigh;

    for ( unsigned int i = 0; i < num_beams; ++i )
    {
        if ( sensor_ranges[i] > 0 )
        {
            currentSegmentInfo.segmentRanges.push_back ( i );
            if ( i == 0 )
            {
                confidenceLow = 0; // Because we have no proof that the complete side of the object is observed
            }
            else
            {
                confidenceLow = 1;
            }
            break;
        }
    }

    if ( currentSegmentInfo.segmentRanges.empty() )
    {
        return;
    }

    int gap_size = 0;
    std::vector<float> gapRanges;

    for ( unsigned int i = currentSegmentInfo.segmentRanges.front(); i < num_beams; ++i )
    {
        float rs = sensor_ranges[i];
        if ( rs == 0 || std::abs ( rs - sensor_ranges[currentSegmentInfo.segmentRanges.back()] ) > segment_depth_threshold_ || i == num_beams - 1 )
        {
            // Found a gap or at final reading
            ++gap_size;
            gapRanges.push_back ( rs );

            if ( gap_size >= max_gap_size_ || i == num_beams - 1 )
            {
                i = currentSegmentInfo.segmentRanges.back() + 1;

                if ( currentSegmentInfo.segmentRanges.size() >= min_segment_size_pixels_ )
                {
                    // calculate bounding box
                    geo::Vec2 seg_min, seg_max;
                    for ( unsigned int k = 0; k < currentSegmentInfo.segmentRanges.size(); ++k )
                    {
                        geo::Vector3 p = lrf_model_.rayDirections() [currentSegmentInfo.segmentRanges[k]] * sensor_ranges[currentSegmentInfo.segmentRanges[k]];

                        if ( k == 0 )
                        {
                            seg_min = geo::Vec2 ( p.x, p.y );
                            seg_max = geo::Vec2 ( p.x, p.y );
                        }
                        else
                        {
                            seg_min.x = std::min ( p.x, seg_min.x );
                            seg_min.y = std::min ( p.y, seg_min.y );
                            seg_max.x = std::max ( p.x, seg_max.x );
                            seg_max.y = std::max ( p.y, seg_max.y );
                        }
                    }

                    geo::Vec2 bb = seg_max - seg_min;
                    if ( ( bb .x > min_cluster_size_ || bb.y > min_cluster_size_ ) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_ )
                    {
                        confidenceHigh = true;
                        for ( unsigned int l = currentSegmentInfo.segmentRanges.size() - POINTS_TO_CHECK_CONFIDENCE; confidenceHigh && l < currentSegmentInfo.segmentRanges.size(); l++ )
                        {
                            for ( unsigned int m = 0; confidenceHigh && m < gapRanges.size(); m++ )
                            {
                                bool check = gapRanges[m] < currentSegmentInfo.segmentRanges[l] ;
                                bool check2 = gapRanges[m] >= 0 + EPSILON;
                                if ( gapRanges[m] < sensor_ranges[currentSegmentInfo.segmentRanges[l]] && gapRanges[m] >= 0 + EPSILON )
                                {
                                    confidenceHigh = false;
                                }
                            }
                        }

                        currentSegmentInfo.confidenceLow = confidenceLow;
                        currentSegmentInfo.confidenceHigh = confidenceHigh;

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

                confidenceLow = true;
                float rsToCheck = sensor_ranges[i];
                for ( unsigned int l = i - nPointsToCheck; confidenceLow && l < i; l++ )
                {
                    float rsToCompare = sensor_ranges[l];
                    bool check = rsToCheck > rsToCompare;
                    bool check2 = rsToCompare <= 0 + EPSILON;
                    if ( rsToCheck > rsToCompare && rsToCompare >= 0 + EPSILON )
                    {
                        confidenceLow = false;
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

    // Try to associate sensor remaining laser point to specific entities
    std::vector<ed::WorldModel::const_iterator> it_laserEntities;
    std::vector< EntityProperty > EntityProperties;

    for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it )
    {
        const ed::EntityConstPtr& e = *e_it;
        std::string laserID = "-laser";

        if ( e->id().str().length() < laserID.length() )
        {
            continue;
        }
        if ( e->id().str().substr ( e->id().str().length() - 6 ) == laserID )  // entity described by laser before
        {
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

            it_laserEntities.push_back ( e_it );

            ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
            EntityProperty currentProperty;

            // For the entities which already exist in the WM, determine the relevant properties in order to determine which entities might associate to which clusters
            if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() )
            {
                ed::tracking::Circle circle = featureProperties.getCircle();

                currentProperty.entity_min.x = circle.get_x() - ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
                currentProperty.entity_max.x = circle.get_x() + ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
                currentProperty.entity_min.y = circle.get_y() - ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
                currentProperty.entity_max.y = circle.get_y() + ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
            }
            else
            {
                ed::tracking::Rectangle rectangle = featureProperties.getRectangle();

                std::vector<geo::Vec2f> corners = rectangle.determineCorners ( ADD_ASSOCIATION_DISTANCE );
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
            EntityProperties.push_back ( currentProperty );
        }

    }
    
    // For each entity, create a vector of points which associate to that entity. A closest-distance criterion is used
    std::vector< std::vector<geo::Vec2f> > pointsAssociatedList ( it_laserEntities.size() );

    for ( unsigned int iSegment = 0; iSegment < segments.size(); ++iSegment )
    {
        ScanSegment& segment = segments[iSegment].segmentRanges;
        unsigned int segment_size = segment.size();
    }

    for ( unsigned int iSegment = 0; iSegment < segments.size(); ++iSegment )
    {
        // First, determine the properties of each segment
        ScanSegment& segment = segments[iSegment].segmentRanges;
        unsigned int segment_size = segment.size();

        std::vector<geo::Vec2f> points ( segment_size );
        geo::Vec2f seg_min, seg_max;

        for ( unsigned int i = 0; i < segment_size; ++i )
        {
            unsigned int j = segment[i];
            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[i] = geo::Vec2f ( p.x, p.y );

            if ( i == 0 )
            {
                seg_min = points[i];
                seg_max = points[i];
            }
            else
            {
                seg_min.x = std::min ( points[i].x, seg_min.x );
                seg_min.y = std::min ( points[i].y, seg_min.y );
                seg_max.x = std::max ( points[i].x, seg_max.x );
                seg_max.y = std::max ( points[i].y, seg_max.y );
            }
        }

        // After the properties of each segment are determined, check which clusters and entities might associate
        std::vector< int > possibleClusterEntityAssociations;

        for ( unsigned int jj = 0; jj < it_laserEntities.size(); ++jj )
        {
            const ed::EntityConstPtr& e = *it_laserEntities[jj];

            // check if 1 of the extrema of the segment might be related to the exploded entity
            bool check1 =  seg_min.x > EntityProperties[jj].entity_min.x && seg_min.x < EntityProperties[jj].entity_max.x ;
            bool check2 =  seg_max.x > EntityProperties[jj].entity_min.x && seg_max.x < EntityProperties[jj].entity_max.x ;
            bool check3 =  seg_min.y > EntityProperties[jj].entity_min.y && seg_min.y < EntityProperties[jj].entity_max.y ;
            bool check4 =  seg_max.y > EntityProperties[jj].entity_min.y && seg_max.y < EntityProperties[jj].entity_max.y ;

            if ( check1 && check3 || check2 && check4 )
            {
                possibleClusterEntityAssociations.push_back ( jj );
            }
        }

        // If a cluster could be associated to a (set of) entities, determine for each point to which entitiy it belongs based on a shortest distance criterion. 
        // If the distance is too large, initiate a new entity
        std::vector<geo::Vec2f> pointsNotAssociated;
        std::vector<float> distances ( points.size() );
        std::vector<unsigned int> IDs ( points.size() );

        for ( unsigned int i_points = 0; i_points < points.size(); ++i_points )  // Determine closest object and distance to this object. If distance too large, relate to new object
        {
            geo::Vec2f p = points[i_points];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = std::numeric_limits< unsigned int >::max();

            for ( unsigned int jj = 0; jj < possibleClusterEntityAssociations.size(); ++jj )  // relevant entities only
            {
                const ed::EntityConstPtr& e = *it_laserEntities[ possibleClusterEntityAssociations[jj] ];
                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
                float dist;

                if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() )  // entity is considered to be a circle
                {
                    ed::tracking::Circle circle = featureProperties.getCircle();
                    dist = std::abs ( std::sqrt ( std::pow ( p.x - circle.get_x(), 2.0 ) + std::pow ( p.y - circle.get_y(), 2.0 ) ) - circle.get_R() ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                }
                else     // entity is considered to be a rectangle. Check if point is inside the rectangle
                {
                    ed::tracking::Rectangle rectangle = featureProperties.getRectangle();


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

        // check at which point the segment should be splitted
        unsigned int IDtoCheck = IDs[0];
        unsigned int firstElement = 0;

        for ( unsigned int iDistances = 1; iDistances < distances.size(); iDistances++ )
        {
            if ( IDs[iDistances] == IDtoCheck && iDistances != distances.size() - 1 ) // ID similar and not at final reading, check next element
            {
                continue;
            }

            unsigned int length = iDistances - firstElement;
            bool test = length < min_segment_size_pixels_ ;

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
                    const ed::EntityConstPtr& entityToTest = *it_laserEntities[ possibleClusterEntityAssociations[IDtoCheck] ];
                    for ( unsigned int i_points = firstElement; i_points < iDistances; ++i_points )
                    {
                        pointsAssociatedList.at ( possibleClusterEntityAssociations[IDtoCheck] ).push_back ( points[i_points] );
                    }
                }
                else
                {
                    pointsNotAssociated.clear();
                    for ( unsigned int i_points = firstElement; i_points < iDistances; ++i_points )
                    {
                        pointsNotAssociated.push_back ( points[i_points] );
                    }
                    pointsAssociatedList.push_back ( pointsNotAssociated );
                }


            }
            firstElement = iDistances;
            IDtoCheck = IDs[iDistances];

        }

    }

// - - - - - - - - - - - - - - - - - -
    std::vector<EntityUpdate> clusters;
    std::vector<ed::tracking::FeatureProperties> measuredProperties ( pointsAssociatedList.size() ); // The first sequence in this vector (with the length of laser entitities) are the properties corresponding to existing entities

    int initialSize = segments.size();

    // Determine the properties for each entity using the points which were associated in the previous step.
    for ( unsigned int iList = 0; iList < pointsAssociatedList.size(); iList++ )
    {
        std::vector<geo::Vec2f> points = pointsAssociatedList[iList];

        if ( points.size() < min_segment_size_pixels_ ) // Add an element to measured properties such that the element in the measuredProperties-vector still corresponds with the laser-entity-vector
        {
            ed::tracking::FeatureProperties properties; // values initialized with nan now
            measuredProperties[iList] =  properties;
            continue;
        }

        // check here for possible corners and splitting
        std::vector<unsigned int> cornerIndices;
        std::vector<ScanSegment> segmentsSplitted;
        std::vector< std::vector<geo::Vec2f> > pointsList; // Vector of points might be splitted as the area described might be a non-occluded area

        bool test = ed::tracking::findPossibleCorners ( points, &cornerIndices );

        if ( !test ) // if no corners detected, add the points to the points-vector
        {
            pointsList.push_back ( points );
        }
        else     // if corners detected, check if a valid region is described. If not, split these data
        {
            cornerIndices.insert ( cornerIndices.begin(), 0 ); // add index of first point of "points"
            cornerIndices.push_back ( points.size() - 1 );// add index of last point of "points"

            std::vector<geo::Vec2f> pointsToCheck;
            for ( unsigned int iIndex = 0; iIndex < cornerIndices.size(); ++iIndex )
            {
                pointsToCheck.push_back ( points[cornerIndices[iIndex]] );
            }

            unsigned int ID_low = cornerIndices[0];
            for ( unsigned int iPointsToCheck = 0; iPointsToCheck < ( pointsToCheck.size() - 2 ); iPointsToCheck++ )
            {
                unsigned int cornerIndex = cornerIndices[iPointsToCheck + 1], ID_high;// = cornerIndices[iPointsToCheck + 2];

                geo::Vec2f startPoint = pointsToCheck[iPointsToCheck];
                geo::Vec2f cornerPoint = pointsToCheck[iPointsToCheck];
                geo::Vec2f endPoint = pointsToCheck[iPointsToCheck];

                geo::Vec2f centerPoint = 0.5* ( startPoint + endPoint );

                float distMidPoint2 = pow ( cornerPoint.x-sensor_pose.getOrigin().getX(), 2.0 ) + pow ( cornerPoint.y-sensor_pose.getOrigin().getY(), 2.0 );
                float distcenterPoint2 = pow ( centerPoint.x-sensor_pose.getOrigin().getX(), 2.0 ) + pow ( centerPoint.y-sensor_pose.getOrigin().getY(), 2.0 );

                bool criterion = distcenterPoint2 < distMidPoint2;
                bool addPointsToPointlist = false;


                if ( distcenterPoint2 <  distMidPoint2 ) // split data at midpoint, because otherwise we will describe a square (or circle) at a region which is not occupied
                {
                    ID_high = cornerIndices[cornerIndex];
                    addPointsToPointlist = true;
                }
                else if ( iPointsToCheck + 3 == pointsToCheck.size() )   // at latest check, so add data to pointslist as well
                {
                    addPointsToPointlist = true;
                    ID_high = cornerIndices[pointsToCheck.size() - 1];
                }

                if ( addPointsToPointlist )
                {
                    if ( ID_high - ID_low >= min_segment_size_pixels_ )
                    {
                        geo::Vec2f seg_min, seg_max;
                        std::vector<geo::Vec2f> pointsLow;
                        for ( unsigned int i_seg = ID_low; i_seg <= ID_high; ++i_seg )
                        {
                            geo::Vec2f p ( points[i_seg].x,  points[i_seg].y ) ; // TODO Right z-coordinate? Not used at the moment!
                            pointsLow.push_back ( p );

                            if ( i_seg == ID_low )
                            {
                                seg_min = p;
                                seg_max = p;
                            }
                            else
                            {
                                seg_min.x = std::min ( p.x, seg_min.x );
                                seg_min.y = std::min ( p.y, seg_min.y );
                                seg_max.x = std::max ( p.x, seg_max.x );
                                seg_max.y = std::max ( p.y, seg_max.y );
                            }
                        }

                        geo::Vec2f bb = seg_max - seg_min;

                        if ( ( bb .x > min_cluster_size_ || bb.y > min_cluster_size_ ) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_ )
                        {
                            pointsList.push_back ( pointsLow );
                        }
                    }

                    if ( iPointsToCheck != ( pointsToCheck.size() - 3 ) )
                    {
                        ID_low = cornerIndices[cornerIndex + 1];
                        iPointsToCheck = cornerIndex;
                    }
                }
            }
        }
        
        unsigned int LargestSegment = 0;
        unsigned int IDLargestSegment;

        for ( unsigned int iPointsList = 0; iPointsList < pointsList.size(); ++iPointsList )
        {
            // Assumption: when an element is splitted, the largest element is the one corresponding to the entity descibed before. Valid?
            if ( pointsList[iPointsList].size() > LargestSegment )
            {
                LargestSegment = pointsList[iPointsList].size();
                IDLargestSegment = iPointsList;
            }
        }

        // Here we start to fit the entity-properties on the segmented data
        for ( unsigned int iPointsList = 0; iPointsList < pointsList.size(); ++iPointsList )
        {
            points = pointsList[iPointsList];
            std::vector<unsigned int> cornerIds;

            std::vector<geo::Vec2f>::iterator it_start = points.begin();
            std::vector<geo::Vec2f>::iterator it_end = points.end();
            bool testForCorner = ed::tracking::findPossibleCorner ( points, &cornerIds, &it_start, &it_end ); // TODO Prevent recomputations
            unsigned int cornerIndex =  std::numeric_limits<unsigned int>::quiet_NaN(); // TODO hack
            if ( testForCorner )
            {
                cornerIndex = cornerIds[0];
            }


            clusters.push_back ( EntityUpdate() );
            EntityUpdate& cluster = clusters.back();

            cluster.pose = geo::Pose3D::identity();

            ed::tracking::Circle circle;
            ed::tracking::Rectangle rectangle;
            std::vector<geo::Vec2f>::iterator it_low, it_high;

            ed::tracking::FITTINGMETHOD method = ed::tracking::CIRCLE;
            float error_circle2 = ed::tracking::fitObject ( points, cluster.pose, method, &cornerIndex, &rectangle, &circle, &it_low, &it_high, sensor_pose );

            method = ed::tracking::determineCase ( points, &cornerIndex, &it_low, &it_high, sensor_pose ); // chose to fit a single line or a rectangle (2 lines)

            float error_rectangle2 = ed::tracking::fitObject ( points, cluster.pose, method,  &cornerIndex, &rectangle, &circle, &it_low, &it_high,  sensor_pose );

            ed::tracking::FeatureProbabilities prob;
            prob.setMeasurementProbabilities ( error_rectangle2, error_circle2, 2*circle.get_R(), MAX_CORRIDOR_WIDTH );

            ed::tracking::FeatureProperties properties;
            properties.setFeatureProbabilities ( prob );
            properties.setCircle ( circle );
            properties.setRectangle ( rectangle );

            if ( iPointsList == IDLargestSegment )
            {
                measuredProperties[iList] = properties;
            }
            else
            {
                measuredProperties.push_back ( properties );
            }

            // TODO: cleanup: remove objects which are fitted and clearly interfere with the walls -> more robustness on segmentation. Where to check?

            // --------------------------
            // Temp for RoboCup 2016; todo: remove after

            // Determine the cluster size
            geo::Vec2f diff = points.back() - points.front();
            float size_sq = diff.length2();
            if ( size_sq > 0.35 * 0.35 && size_sq < 0.8 * 0.8 )
            {
                cluster.flag = "possible_human";
            }
        }

        // --------------------------
    }

    // TODO: update properties of the entities. Right properties, position updated. Maintaining of properties although not observed anymore. TODO: determine covariance-matrices
    // TODO: simple version for tracking
    // TODO: set existence probability of the entities
    // TODO: merging of entities (for example when there was a partial occlusion). Use the uncertainty about the size of the object for it! -> less relevant for now
    // TODO: better localization based on laser data corresponding to data which are belonging to the environment by doing a fit-> smaller errors
    // TODO: Cleanup of source code

    unsigned int marker_ID = 0; // To Do: After tracking, the right ID's should be created. The ID's are used to have multiple markers.

    ed::tracking::FeatureProperties measuredProperty, entityProperties; // Measured properties and updated properties
    ed::UUID id;

    for ( unsigned int i_properties = 0; i_properties < measuredProperties.size(); i_properties++ )
    {
        measuredProperty = measuredProperties[i_properties];


        double existenceProbability;
        if ( i_properties < it_laserEntities.size() )
        {
            const ed::EntityConstPtr& e = * ( it_laserEntities[i_properties] );

            // check if new properties are measured.
            bool check1 = measuredProperty.getCircle().get_R() != measuredProperty.getCircle().get_R();
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

            if ( !e->hasFlag ( "locked" ) )
            {
                entityProperties = e->property ( featureProperties_ );

                float Q = 0.1; // Measurement noise covariance. TODO: let it depend on if an object is partially occluded. Now, objects are assumed to be completely visible
                float R = 0.0; // Process noise covariance

                Eigen::MatrixXd Qm ( 2, 2 ), Rm ( 2, 2 );
                Eigen::VectorXd z_k ( 2, 1 );

                Qm << Q, 0, 0, Q;
                Rm << R, 0, 0, R;

                z_k << measuredProperty.getRectangle().get_w(), measuredProperty.getRectangle().get_d(); // How are the width and depth determined? How to ensure the depth-information will not be confused with the width-information?

                entityProperties.updateProbabilities ( measuredProperty.getFeatureProbabilities() ); // TODO: update probabilities of the features -> Do we still need to use them? Because this will be solved in the PF
                entityProperties.updateCircleSize ( Q, R, measuredProperty.getCircle().get_R() );
                entityProperties.updateRectangleSize ( Qm, Rm, z_k );
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
            id = ed::Entity::generateID().str() + "-laser";

            // Update existence probability
            existenceProbability = 1.0; // TODO magic number
            entityProperties = measuredProperty;
        }
        geo::Pose3D new_pose;

        if ( entityProperties.getFeatureProbabilities().get_pCircle() < entityProperties.getFeatureProbabilities().get_pRectangle() )
        {
            // determine corners
            ed::tracking::Rectangle rectangle = entityProperties.getRectangle();
            std::vector<geo::Vec2f> corners = entityProperties.getRectangle().determineCorners ( 0.0 );
            new_pose = rectangle.getPose();
        }
        else
        {
            // determine cilinder-properties
            ed::tracking::Circle circle = entityProperties.getCircle();
            new_pose = circle.getPose();
        }

        bool check = true;
        if ( new_pose.t.getX() != new_pose.t.getX() || std::abs ( new_pose.t.getX() ) > COORDINATE_OUTSIDE_MAP ||
                new_pose.t.getZ() != new_pose.t.getZ() ||  std::abs ( new_pose.t.getY() ) > COORDINATE_OUTSIDE_MAP ||
                new_pose.t.getY() != new_pose.t.getY() ||  std::abs ( new_pose.t.getZ() ) > COORDINATE_OUTSIDE_MAP ||
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

// - - - - - - - - - - - - - - - - -

    std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms. \n\n\n" << std::endl;

}


// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback ( const sensor_msgs::LaserScan::ConstPtr& msg )
{
    scan_buffer_.push ( msg );
}

ED_REGISTER_PLUGIN ( LaserPlugin )
