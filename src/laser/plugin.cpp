#include "plugin.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
//#include <ed/helpers/depth_data_processing.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>
#include <numeric>
#include <cmath>
#include <iterator>

#include "problib/conversions.h"
//#include "problib/datatypes.h"

#include "ed_sensor_integration/properties/feature_info.h"


namespace
{

typedef std::vector<unsigned int> ScanSegment;

struct EntityUpdate {
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

struct EntityProperty{
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
    for ( unsigned int i = 0; i < test_model_ranges.size(); ++i ) {
        double ds = sensor_ranges[i];
        double dm = test_model_ranges[i];

        if ( ds <= 0 ) {
            continue;
        }

        ++n;

        if ( dm <= 0 ) {
            total_error += 0.1;
            continue;
        }

        double diff = std::abs ( ds - dm );
        if ( diff < 0.1 ) {
            total_error += diff;
        } else {
            if ( ds > dm ) {
                total_error += 1;
            } else {
                total_error += 0.1;
            }
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

    return total_error / ( n+1 ); // to be sure to never divide by zero.
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D getPoseFromCache ( const ed::Entity& e, std::map<ed::UUID,geo::Pose3D>& pose_cache )
{
    const ed::UUID ID = e.id();
    geo::Pose3D old_pose = e.pose();
    if ( pose_cache.find ( ID ) == pose_cache.end() ) {
        pose_cache[ID] = old_pose;
    } else {
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

//    int num_model_points;
//    int num_model_points2;
//    double min_error = 0.97 * getFittingError(e, lrf, sensor_pose_inv * e.pose(), sensor_ranges, model_ranges, num_model_points); //last position
//    geo::Pose3D best_pose = e.pose();

//    if (num_model_points < 70)
//    {
//        ROS_ERROR_STREAM("not fitting with last position (" << num_model_points << ")");
//        // check if virtual door in middle of range is visible
//        geo::Mat3 rot90;
//        rot90.setRPY(0, 0, 0.5*(yaw_min+yaw_plus));
//        geo::Pose3D pose90 = old_pose;
//        pose90.R = old_pose.R * rot90;
//        double error90 = 0.97 * getFittingError(e, lrf, sensor_pose_inv * pose90, sensor_ranges, model_ranges, num_model_points2);
//        if (num_model_points2 < 70)
//        {
//            //std::cout<<"not fitting with 90 degrees too" << std::endl;
//            return best_pose;
//        }
//        else
//        {
//            // fitting started after door in middle of range is visible. Be stricter on error.
//            num_model_points=num_model_points2;
//            min_error=0.7*error90;
//        }
//    }


    double min_error = 1e6;
    geo::Pose3D best_pose = e.pose();


    for ( float dyaw = yaw_min; dyaw <= yaw_plus; dyaw += yaw_step ) {
        geo::Mat3 rot;
        rot.setRPY ( 0, 0, dyaw );
        geo::Pose3D test_pose = old_pose;
        test_pose.R = old_pose.R * rot;

        for ( float dx = -x_window; dx <= x_window; dx += x_step ) {
            test_pose.t.x = old_pose.t.x + dx;
            for ( float dy = -y_window; dy <= y_window; dy += y_step ) {
                test_pose.t.y = old_pose.t.y + dy;

                int num_model_points;
                double error = getFittingError ( e, lrf, sensor_pose_inv * test_pose, sensor_ranges, model_ranges, num_model_points );

//                ROS_ERROR_STREAM("yaw = " << dyaw << ", error = " << error << ", minerror= " << min_error << ", num_model_points = " << num_model_points);

                if ( error < min_error && num_model_points >= 3 ) {
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

bool pointIsPresent ( double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges )
{
    int i_beam = lrf.getAngleUpperIndex ( x_sensor, y_sensor );
    if ( i_beam < 0 || i_beam >= sensor_ranges.size() ) {
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

// ----------------------------------------------------------------------------------------------------
void publishFeatures ( ed::tracking::FeatureProperties& featureProp, int ID, ros::Publisher& pub ) // TODO move to ed_rviz_plugins
{
    visualization_msgs::Marker marker;

    if ( featureProp.getFeatureProbabilities().get_pCircle() > featureProp.getFeatureProbabilities().get_pRectangle() ) {
        ed::tracking::Circle circle = featureProp.getCircle();
        circle.setMarker ( marker , ID );
    } else {
        ed::tracking::Rectangle rectangle = featureProp.getRectangle();
        rectangle.setMarker ( marker , ID );
    }

    pub.publish ( marker );
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


    int i_fit_entities = 0;
    config.value ( "fit_entities", i_fit_entities, tue::OPTIONAL );
    fit_entities_ = ( i_fit_entities != 0 );

    int i_check_door_status = 0;
    config.value ( "check_door_status", i_check_door_status, tue::OPTIONAL );
    check_door_status_ = ( i_check_door_status != 0 );

    if ( config.hasError() ) {
        return;
    }

    ros::NodeHandle nh;
    nh.setCallbackQueue ( &cb_queue_ );

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan> ( laser_topic, 3, &LaserPlugin::scanCallback, this );
    door_pub_ = nh.advertise<ropod_demo_dec_2017::doorDetection> ( "door", 3 );
    ObjectMarkers_pub_ = nh.advertise<visualization_msgs::Marker> ( "ObjectMarkers", 3 );

    tf_listener_ = new tf::TransformListener;

    //pose_cache.clear();

    init.properties.registerProperty ( "Feature", featureProperties_, new FeatureInfo ); // example given in ED/ed/examples/custom_properties. Update the probabilities using an update-request

}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::process ( const ed::WorldModel& world, ed::UpdateRequest& req )
{
    cb_queue_.callAvailable();

    while ( !scan_buffer_.empty() ) {
        sensor_msgs::LaserScan::ConstPtr scan = scan_buffer_.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF

        try {
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform ( "map", scan->header.frame_id, scan->header.stamp, t_sensor_pose );
            scan_buffer_.pop();
            geo::Pose3D sensor_pose;
            geo::convert ( t_sensor_pose, sensor_pose );
            update ( world, scan, sensor_pose, req );
        } catch ( tf::ExtrapolationException& ex ) {
            ROS_WARN_STREAM_DELAYED_THROTTLE ( 10, "ED Laserplugin: " << ex.what() );
            try {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform ( "map", scan->header.frame_id, ros::Time ( 0 ), latest_transform );

                if ( scan_buffer_.front()->header.stamp > latest_transform.stamp_ ) {
                    // Scan is too new
                    break;
                } else {
                    // Otherwise it has to be too old (pop it because we cannot use it anymore)
                    scan_buffer_.pop();
                }
            } catch ( tf::TransformException& exc ) {
                scan_buffer_.pop();
            }
        } catch ( tf::TransformException& exc ) {
            ROS_ERROR_STREAM_DELAYED_THROTTLE ( 10, "ED Laserplugin: " << exc.what() );
            scan_buffer_.pop();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::update ( const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                           const geo::Pose3D& sensor_pose, ed::UpdateRequest& req )
{
  std::cout << "Start of update" << std::endl;
    tue::Timer t_total;
    t_total.start();

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges ( scan->ranges.size() );
    for ( unsigned int i = 0; i < scan->ranges.size(); ++i ) {
        float r = scan->ranges[i];
        if ( r > scan->range_max ) {
            sensor_ranges[i] = r;
        } else if ( r == r && r > scan->range_min ) {
            sensor_ranges[i] = r;
        } else {
            sensor_ranges[i] = 0;
        }
    }

    unsigned int num_beams = sensor_ranges.size();

    if ( lrf_model_.getNumBeams() != num_beams ) {
        lrf_model_.setNumBeams ( num_beams );
        lrf_model_.setAngleLimits ( scan->angle_min, scan->angle_max );
        lrf_model_.setRangeLimits ( scan->range_min, scan->range_max );
    }

    // - - - - - - - - - - - - - - - - - -
    // Filter laser data (get rid of ghost points)

    for ( unsigned int i = 1; i < num_beams - 1; ++i ) {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if ( std::abs ( rs - sensor_ranges[i - 1] ) > 0.1 && std::abs ( rs - sensor_ranges[i + 1] ) > 0.1 ) { // TODO: magic number
            sensor_ranges[i] = sensor_ranges[i - 1];
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    std::vector<double> model_ranges ( num_beams, 0 );
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it ) {
        const ed::EntityConstPtr& e = *it;

        //std::cout << "BLA0 = " << e->hasType("hospital_test/door") << "type = " << e->type() << std::endl;

        //std::cout << "Localization plugin: id = " << e->id() << ". shape = " << e->shape() << std::endl;

        if ( e->shape() && e->has_pose() && ! ( e->hasType ( "left_door" ) || e->hasType ( "door_left" ) || e->hasType ( "right_door" ) || e->hasType ( "door_right" ) || e->hasFlag ( "non-localizable" ) ) ) {
            // std::cout << "Shape after = " << e->shape() << std::endl;
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * e->pose() );

            geo::LaserRangeFinder::RenderResult res ( model_ranges );
            lrf_model_.render ( opt, res );
        }
    }
std::cout << "Before door fitting" << std::endl;
    // - - - - - - - - - - - - - - - - - -
    // Fit the doors

    if ( fit_entities_ ) {
        std::cout << "Fitting!" << std::endl;

        for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it ) {
            const ed::EntityConstPtr& e = *it;
            //std::cout << e->type() << std::endl;

            if ( !e->shape() || !e->has_pose() ) {
                continue;
            }

            geo::Pose3D e_pose_SENSOR = sensor_pose_inv * e->pose();

            // If not in sensor view, continue
            if ( e_pose_SENSOR.t.length2() > 5.0 * 5.0 || e_pose_SENSOR.t.x < 0 ) {
                continue;
            }

            if ( e->hasType ( "left_door" ) || e->hasType ( "door_left" ) ) {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity ( *e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache );
                req.setPose ( e->id(), new_pose );
                //std::cout << "left_door" << std::endl;

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * new_pose );

                geo::LaserRangeFinder::RenderResult res ( model_ranges );
                lrf_model_.render ( opt, res );
            } else if ( e->hasType ( "right_door" ) || e->hasType ( "door_right" ) ) {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity ( *e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache );
                req.setPose ( e->id(), new_pose );
                //std::cout << "right_door" << std::endl;

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * new_pose );

                geo::LaserRangeFinder::RenderResult res ( model_ranges );
                lrf_model_.render ( opt, res );
            }
        }
    }

//    if ( check_door_status_ ) {
//         /* Points associated to door */
//
//         ed::UUID id;
//         for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it ) {
//             const ed::EntityConstPtr& e = *it;
//
//             if ( e->hasType ( "elevatordoor" ) ) {
//
// 	      // assume elevatordoor to be square
// 	      const ed::ConvexHull& entity_chull = e->convexHull();
//
// 	      //std::cout << entity_chull.edges << std::endl;
//
// 	      std::cout << "New door: "<< std::endl;
// 	      std::vector<float> x_coordinates(entity_chull.points.size());
// 	      std::vector<float> y_coordinates(entity_chull.points.size());
// 	      std::vector<float> dist2(entity_chull.points.size());
//
// 	      for(int ii = 0; ii < entity_chull.points.size(); ii++)
// 	      {
// 		x_coordinates[ii] = entity_chull.points[ii].x + e->pose().getOrigin().getX();
// 		y_coordinates[ii] = entity_chull.points[ii].y + e->pose().getOrigin().getY();
//
// 		std::cout << "Edges = " << entity_chull.edges[ii] << " Point = (" << x_coordinates[ii] << ", " << y_coordinates[ii] << ")" << std::endl;
//
// 		dist2[ii] =  pow(x_coordinates[ii] - sensor_pose.getOrigin().getX(), 2.0) + pow(y_coordinates[ii] - sensor_pose.getOrigin().getY(), 2.0);
//
// 	      }
//
// 	      // Get length at right side of the door
// 	       std::vector<float>::iterator closestPoint = min_element(dist2.begin(), dist2.end());
// 	       std::vector<float>::iterator previousPoint;
// 	       std::vector<float>::iterator nextPoint;
// 	       if(closestPoint == dist2.begin())
// 	       {
// 		 previousPoint = dist2.end();
// 		 nextPoint += 1;
// 	       }
// 	       else if(closestPoint == dist2.end())
// 	       {
// 		 previousPoint -= 1;
// 		 nextPoint = dist2.begin();
// 	       }
// 	       else
// 	       {
// 		 previousPoint -= 1;
// 		 nextPoint += 1;
// 	       }
//
// 	       float Edge_length = entity_chull.edges[*closestPoint].length2();
// 	       float Prev_Edge_length = entity_chull.edges[*closestPoint].length2();
// 	       float doorlength;
//
// 	       std::vector<geo::Vec2f> doorFront(2); // Vector with the 2 points forming the front side of the door
// 	       doorFront[0] = entity_chull.points[*closestPoint];
// 	       if(Prev_Edge_length > Edge_length)
// 	       {
// 		 doorFront[1] = entity_chull.points[*previousPoint];
// 		 doorlength = sqrt(Prev_Edge_length);
// 	       }
// 	       else
// 	       {
// 		 doorFront[1] = entity_chull.points[*nextPoint];
// 		 doorlength = sqrt(Edge_length);
// 	       }
//
//
//
// 	    /*  float inf = std::numeric_limits<float>::infinity();
// 	     *  //https://gamedev.stackexchange.com/questions/44483/how-do-i-calculate-distance-between-a-point-and-an-axis-aligned-rectangle/50722
// 	      float max_angle = -inf;
// 	      float min_angle = inf;
//
// 	      for(int ii = 0; ii < entity_chull.points.size(); ii++)
// 	      {
// 		x_coordinates[ii] = entity_chull.points[ii].x;
// 		y_coordinates[ii] = entity_chull.points[ii].y;
//
// 		float angle_coordinate = atan2(y_coordinates[ii]-sensor_pose.point.y, x_coordinates[ii]-sensor_pose.point.x);
// 		float angle_to_sensor = angle_coordinate-sensor_pose.getYaw();
// 		float wrapped_angle = angle_to_sensor -2*M_PIl*floor(angle_to_sensor/2*M_PIl);
//
// 		if(wrapped_angle > max_angle)
// 		  max_angle = wrapped_angle;
//
// 		if(wrapped_angle < min_angle)
// 		  min_angle = wrapped_angle;
//
// 		//std::cout << "x-coordinate = " << entity_chull.points[ii].x << " y-coordinate " << entity_chull.points[ii].y << std::endl;
// 	      }
//
// 	      float center_x = std::accumulate(x_coordinates.begin(), x_coordinates.end(), 0.0) / entity_chull.points.size() + e->pose().getOrigin().getX();
//
// 	      float center_y = std::accumulate(y_coordinates.begin(), y_coordinates.end(), 0.0) / entity_chull.points.size()+  e->pose().getOrigin().getY();
//
// 	      float length_x = *max_element(x_coordinates.begin(), x_coordinates.end())-*min_element(x_coordinates.begin(), x_coordinates.end());
// 	      float length_y = *max_element(y_coordinates.begin(), y_coordinates.end())-*min_element(y_coordinates.begin(), y_coordinates.end());
//
// 	      float dx = std::max(std::abs(sensor_pose.getOrigin().getX() - center_x) - 0.5*length_x, 0.0);
// 	      float dy = std::max(std::abs(sensor_pose.getOrigin().getY() - center_y) - 0.5*length_y, 0.0);
//
// 	      float length2 = dx * dx + dy * dy;
//
// 	      std::cout << "Dist to door = " << length2 << std::endl;
// 	      //std::cout << "x-sensor = " << sensor_pose.getOrigin().getX() << " center_x = " << center_x << " length_x = " << length_x  << std::endl;
// 	      //std::cout << "y-sensor = " << sensor_pose.getOrigin().getY() << " center_y = " << center_y << " length_y = " << length_y  << std::endl;
//
// 	      bool sensibility;
// 	      if(length2 < (scan->range_max * scan->range_max) && max_angle < scan->angle_max && min_angle > scan->angle_min)
// 	      {
// 		sensibility = true;
// 	      }
// 	      else
// 	      {
// 		sensibility = false;
// 	      }
// 	      */
//
// 	     // sum_x/entity_chull.points.size();
// 	     // float center_y = sum_y/entity_chull.points.size();
//
// 	      //std::cout << std::endl;
//
//
// 	     // ;
//
//                 id = e->id();
//                 //  std::cout << "id = " << e->id() << std::endl;
//
//                 // Set render options
// 		std::vector<double> model_ranges_door ( num_beams, 0 );
//                 geo::LaserRangeFinder::RenderOptions opt;
//                 opt.setMesh ( e->shape()->getMesh(), sensor_pose_inv * e->pose() );
//
//                 geo::LaserRangeFinder::RenderResult res ( model_ranges_door );
//                 lrf_model_.render ( opt, res ); /* so all data > 0 belong to door! */
//
//
// 		int firstPoint;
// 		bool firstPointFound = false;
// 		int lastPoint = 0;
// 		unsigned int lengthDoorCounter = 0;
// 		double sum = 0;
//                 unsigned int counterSum = 0;
//
// 		for ( unsigned int i = 0; i < num_beams; ++i ) {
//                     if ( model_ranges_door[i] > 0 ) {
// 		      if(!firstPointFound)
// 		      {
// 			firstPoint = i;
// 		      }
//
// 		      lastPoint = i;
// 		      lengthDoorCounter++;
// 		      sum += ( sensor_ranges[i] - model_ranges_door[i] );
//                       counterSum++;
//                     }
//                 }
//
//
// 	      // Assumption: small side of the foor not taken into consideration -> can be neglected
// 		float coverage = (model_ranges_door[lastPoint] - model_ranges_door[firstPoint])/doorlength;
//
//                 // std::cout << std::endl;
//
//                 double avg_dist = sum/counter;
//                 // std::cout << "sum = " << sum << "counter = " << counter << std::endl;
//                 double bound = 0.2;
//
//                 ropod_demo_dec_2017::doorDetection msg;
//
//                 msg.id = id.str();
//                 msg.type = "elevatordoor";
//                 msg.open = 0;
//                 msg.closed = 0;
//                 msg.undetectable = 0;
//
//                 //std::cout << "avg_dist = " << avg_dist << std::endl;
//                 if ( avg_dist >= bound ) {
//                     std::cout << "Door open" << std::endl;
//                     msg.open = 1;
//                     req.setFlag ( e->id(), "non-localizable" );
//
//                     if ( e->hasFlag ( "localizable" ) ) {
//                         req.removeFlag ( e->id(),"localizable" );
//                     }
//
//                 } else if ( avg_dist <= -bound || avg_dist != avg_dist || avg_dist > scan->range_max ) {
//                     std::cout << "Door not detecable" << std::endl;
//                     msg.undetectable = 1;
//                     req.setFlag ( e->id(), "localizable" );
//
//                     if ( e->hasFlag ( "non-localizable" ) ) {
//                         req.removeFlag ( e->id(),"non-localizable" );
//                     }
//                 } else {
//                     //std::cout << "Door closed" << std::endl;
//                     msg.closed = 1;
//                     req.setFlag ( e->id(), "localizable" );
//
//                     if ( e->hasFlag ( "non-localizable" ) ) {
//                         req.removeFlag ( e->id(),"non-localizable" );
//                     }
//                 }
//
//                 //e->printFlags();
//                 msg.header.stamp = ros::Time::now();
//
//                 door_pub_.publish ( msg );
//             }
//         }
//     }
    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    for ( unsigned int i = 0; i < num_beams; ++i ) {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        if ( rs <= 0
                || ( rm > 0 && rs > rm ) // If the sensor point is behind the world model, skip it
                || ( std::abs ( rm - rs ) < world_association_distance_ ) ) {
            sensor_ranges[i] = 0;
        }
    }

std::cout << "After render" << std::endl;

    /*
        std::vector<ed::WorldModel::const_iterator> it_laserEntities;
        // Try to associate sensor remaining laser point to specific entities
        for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it ) {
            const ed::EntityConstPtr& e = *e_it;

            if ( e->id().str().substr ( e->id().str().length() - 6 ) == "-laser" ) { // entity described by laser before
                it_laserEntities.push_back ( e_it );
            }

        }

        std::vector< std::vector<geo::Vec2f> > pointsAssociatedList ( it_laserEntities.size() );
        for ( unsigned int ii = 0; ii < num_beams; ++ii ) { // TODO relevant beams only

            geo::Vector3 p = lrf_model_.rayDirections() [ii] * sensor_ranges[ii];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = 0;

            for ( unsigned int jj = 0; jj < it_laserEntities.size(); ++jj ) { // TODO relevant entities only
                //ed::WorldModel::const_iterator itEntity = it_laserEntities[jj];
                const ed::EntityConstPtr& e = *it_laserEntities[jj];
                // get type -> rectangle or circle
                // determine distance

                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
                float dist;
                if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() ) { // entity is considered to be a circle
                    ed::tracking::Circle circle = featureProperties.getCircle();
                    dist = std::abs ( std::sqrt ( std::pow ( p.x - circle.get_x(), 2.0 ) + std::pow ( p.y - circle.get_y(), 2.0 ) -   circle.get_R() ) ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                } else { // entity is considered to be a rectangle
                    ed::tracking::Rectangle rectangle = featureProperties.getRectangle();

                    float x = rectangle.get_x();
                    float y = rectangle.get_y();
                    float theta = rectangle.get_theta();
                    float dx = 0.5*rectangle.get_w();
                    float dy = 0.5*rectangle.get_d();

                    float ct = cos ( theta );
                    float st = sin ( theta );

                    geo::Vec2f originCorner ( x + ct*-dx + st* dy, y + st*-dx + ct*-dy ); // Rotation matrix
                    geo::Vec2f corner1 ( x + ct* dx + st* dy, y + st* dx + ct*-dy );
                    geo::Vec2f corner2 ( x + ct* dx - st* dy, y + st*-dx + ct* dy );
                    geo::Vec2f corner3 ( x + ct*-dx - st* dy, y + st*-dx + ct* dy );

                    geo::Vec2f vx = corner1 - originCorner;
                    geo::Vec2f vy = corner3 - originCorner;

    		geo::Vec2f p_float(p.x, p.y);
                    geo::Vec2f pCorrected = p_float-originCorner;

                    // Test if point is inside rectangle https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
                    geo::Vec2f OP = pCorrected - originCorner; // Distance from origin to point which is tested
                    geo::Vec2f OC1 = corner1 - originCorner;
                    geo::Vec2f OC3 = corner3 - originCorner;

                    float OP_OC1   = OP.dot(OC1);    // elementwise summation
                    float OC1_OC1B = OC1.dot(OC1);
                    float OP_OC3   = OP.dot(OC3);
                    float OC3_OC3  = OC3.dot(OC3);

                    float minDistance = std::numeric_limits< float >::infinity();

                    if ( OP_OC1 > 0 && OC1_OC1B > OP_OC1 && OP_OC3 > 0 && OC3_OC3 > OP_OC3 ) { // point is inside the rectangle

                        std::vector<geo::Vec2f> p1Check;
                        p1Check.push_back ( originCorner );
                        p1Check.push_back ( corner1 );
                        p1Check.push_back ( corner2 );
                        p1Check.push_back ( corner3 );

                        std::vector<geo::Vec2f> p2Check;
                        p1Check.push_back ( corner3 );
                        p1Check.push_back ( originCorner );
                        p1Check.push_back ( corner1 );
                        p1Check.push_back ( corner2 );

                        for ( unsigned int ii_dist = 0; ii_dist < p1Check.size(); ii_dist++ ) {

                            float x1 = p1Check[ii_dist].x;
                            float x2 = p2Check[ii_dist].x;

                            float y1 = p1Check[ii_dist].y;
                            float y2 = p2Check[ii_dist].y;

                            float distance = std::abs ( ( y2 - y1 ) *p.x - ( x2 - x1 ) *p.y + x2*y1 -y2*x1 ) /std::sqrt ( std::pow ( y2-y1, 2.0 ) + std::pow ( x2-x1, 2.0 ) );

                            if ( distance < minDistance ) {
                                minDistance = distance;
                            }
                        }
                    } else { // point is outside the rectangle, https://stackoverflow.com/questions/44824512/how-to-find-the-closest-point-on-a-right-rectangular-prism-3d-rectangle/44824522#44824522
                        float tx = pCorrected.dot ( vx ) / ( vx.dot ( vx ) );
                        float ty = pCorrected.dot ( vx ) / ( vx.dot ( vx ) );

                        tx = tx < 0 ? 0 : tx > 1 ? 1 : tx;
                        ty = ty < 0 ? 0 : ty > 1 ? 1 : ty;

                        geo::Vec2f closestPoint = tx*vx + ty*vy + originCorner;

                        geo::Vec2f vector2Point = p_float - closestPoint;
                        minDistance = std::sqrt ( vector2Point.dot ( vector2Point ) );
                    }
                }


                if ( dist < shortestDistance ) {
                    shortestDistance = dist;
                    id_shortestEntity = jj;
                }

            }

            geo::Vec2f point ( p.x, p.y );
            pointsAssociatedList.at ( id_shortestEntity ).push_back ( point );


            //points.push_back(point);


            //.push_back(1.0, 2.0);
            //.push_back(p.x, p.y);

        }
        */
// What to do if points need to be associated to an object, but is segmented out? For example when it rotates? Prediction stap on latest information?
// Can we assume the update-rate is fast relative to the speed of movement, so the minimum distance criterion is sufficient?


// - - - - - - - - - - - - - - - - - -
// Segment the remaining points into clusters

    std::vector<ScanSegment> segments;

// Find first valid value
    ScanSegment current_segment;

// std::cout << "nbeams = " << num_beams << std::endl;
    for ( unsigned int i = 0; i < num_beams; ++i ) {
        if ( sensor_ranges[i] > 0 ) {
            current_segment.push_back ( i );
            break;
        }
    }

    if ( current_segment.empty() ) {
        //std::cout << "No residual point cloud!" << std::endl;
        return;
    }

    int gap_size = 0;

    for ( unsigned int i = current_segment.front(); i < num_beams; ++i ) {
        float rs = sensor_ranges[i];

        if ( rs == 0 || std::abs ( rs - sensor_ranges[current_segment.back()] ) > segment_depth_threshold_ || i == num_beams - 1 ) {
            // Found a gap or at final reading
            ++gap_size;

            if ( gap_size >= max_gap_size_ || i == num_beams - 1 ) {
                i = current_segment.back() + 1;

                if ( current_segment.size() >= min_segment_size_pixels_ ) {
                    // calculate bounding box
                    geo::Vec2 seg_min, seg_max;
                    for ( unsigned int k = 0; k < current_segment.size(); ++k ) {
                        geo::Vector3 p = lrf_model_.rayDirections() [current_segment[k]] * sensor_ranges[current_segment[k]];

                        if ( k == 0 ) {
                            seg_min = geo::Vec2 ( p.x, p.y );
                            seg_max = geo::Vec2 ( p.x, p.y );
                        } else {
                            seg_min.x = std::min ( p.x, seg_min.x );
                            seg_min.y = std::min ( p.y, seg_min.y );
                            seg_max.x = std::max ( p.x, seg_max.x );
                            seg_max.y = std::max ( p.y, seg_max.y );
                        }
                    }

                    geo::Vec2 bb = seg_max - seg_min;
                    if ( ( bb .x > min_cluster_size_ || bb.y > min_cluster_size_ ) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_ ) {
                        segments.push_back ( current_segment );
                    }
                }

                current_segment.clear();

                // Find next good value
                while ( sensor_ranges[i] == 0 && i < num_beams ) {
                    ++i;
                }

                current_segment.push_back ( i );
            }
        } else {
            gap_size = 0;
            current_segment.push_back ( i );
        }
    }
    
    std::cout << "Before segmenting laser-entities" << std::endl;

    std::vector<ed::WorldModel::const_iterator> it_laserEntities;
    // Try to associate sensor remaining laser point to specific entities
    std::vector< EntityProperty > EntityProperties;
    for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it ) {
        const ed::EntityConstPtr& e = *e_it;
	
	std::cout << "Test1" << std::endl;
	//	std::cout << "Id = " << e->id().str() << " length = " <<  e->id().str().length() << std::endl;
	
	std::string laserID = "-laser";
	if( e->id().str().length() < laserID.length() )
	  continue;

        if ( e->id().str().substr ( e->id().str().length() - 6 ) == laserID ) { // entity described by laser before
            it_laserEntities.push_back ( e_it );
	    std::cout << "Test2" << std::endl;
	    ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
            float dist;
	    EntityProperty currentProperty;
	    std::cout << "Test3" << std::endl;
            if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() ) { // entity is considered to be a circle
	      std::cout << "Test4" << std::endl;
                ed::tracking::Circle circle = featureProperties.getCircle();

                currentProperty.entity_min.x = circle.get_x() - ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
                currentProperty.entity_max.x = circle.get_x() + ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
                currentProperty.entity_min.y = circle.get_y() - ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );
                currentProperty.entity_max.y = circle.get_y() - ( 0.5*ADD_ASSOCIATION_DISTANCE + circle.get_R() );

            } else {
std::cout << "Test5" << std::endl;
                ed::tracking::Rectangle rectangle = featureProperties.getRectangle();

		std::vector<geo::Vec2f> corners = rectangle.determineCorners(ADD_ASSOCIATION_DISTANCE);
             /*   float x = rectangle.get_x();
                float y = rectangle.get_y();
                float theta = rectangle.get_theta();
                float dx = 0.5* ( rectangle.get_w() + ADD_ASSOCIATION_DISTANCE ); // blow up for associations
                float dy = 0.5* ( rectangle.get_d() + ADD_ASSOCIATION_DISTANCE );

                float ct = cos ( theta );
                float st = sin ( theta );

		std::cout << "Test6" << std::endl;
		
                geo::Vec2f originCorner ( x + ct*-dx + st* dy, y + st*-dx + ct*-dy ); // Rotation matrix
                geo::Vec2f corner1 ( x + ct* dx + st* dy, y + st* dx + ct*-dy );
                geo::Vec2f corner2 ( x + ct* dx - st* dy, y + st*-dx + ct* dy );
                geo::Vec2f corner3 ( x + ct*-dx - st* dy, y + st*-dx + ct* dy );

                std::vector< geo::Vec2f > corners;
                corners.push_back ( originCorner );
                corners.push_back ( corner1 );
                corners.push_back ( corner2 );
                corners.push_back ( corner3 );
                */

                currentProperty.entity_min = corners[0];
                currentProperty.entity_max = corners[0];
		std::cout << "Test7" << std::endl;
                for ( unsigned int i_corner = 1; i_corner < corners.size(); i_corner++ ) {
                    currentProperty.entity_min.x = std::min ( corners[i_corner].x, currentProperty.entity_min.x );
                    currentProperty.entity_min.y = std::min ( corners[i_corner].y, currentProperty.entity_min.y );
                    currentProperty.entity_max.x = std::max ( corners[i_corner].x, currentProperty.entity_max.x );
                    currentProperty.entity_max.y = std::max ( corners[i_corner].y, currentProperty.entity_max.y );
		    std::cout << "Test8" << std::endl;
                }
            }

            EntityProperties.push_back(currentProperty);
	    std::cout << "Test9" << std::endl;
        }

    }
    
    std::cout << "Laser entities segmented. Size = " << it_laserEntities.size() << std::endl;


    // fast check: which entities and clusters might associate?
    //TODO propagate entity based on its latest velocity information
    // check distance criterion

    //std::vector<EntityUpdate> clustersCheck;
    //std::vector< > possibleClusterEntityAssociations;
    
    std::vector< std::vector<geo::Vec2f> > pointsAssociatedList ( it_laserEntities.size() ); //list of points associated to each entity
    for ( unsigned int iSegment = 0; iSegment < segments.size(); ++iSegment ) {

        ScanSegment& segment = segments[iSegment];
        unsigned int segment_size = segment.size();

        std::vector<geo::Vec2f> points ( segment_size );

        geo::Vec2f seg_min, seg_max;
        for ( unsigned int i = 0; i < segment_size; ++i ) {
            unsigned int j = segment[i];

            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[i] = geo::Vec2f ( p.x, p.y );


            if ( i == 0 ) {
                seg_min = points[i];
                seg_max = points[i];
            } else {
                seg_min.x = std::min ( points[i].x, seg_min.x );
                seg_min.y = std::min ( points[i].y, seg_min.y );
                seg_max.x = std::max ( points[i].x, seg_max.x );
                seg_max.y = std::max ( points[i].y, seg_max.y );
            }
        }
        //}

        std::vector< int > possibleClusterEntityAssociations;
        for ( unsigned int jj = 0; jj < it_laserEntities.size(); ++jj ) {
            const ed::EntityConstPtr& e = *it_laserEntities[jj];
           
            // check if 1 of the extrema of the segment might be related to the exploded entity
            bool check1 =  seg_min.x > EntityProperties[jj].entity_min.x && seg_min.x < EntityProperties[jj].entity_max.x ;
            bool check2 =  seg_max.x > EntityProperties[jj].entity_min.x && seg_max.x < EntityProperties[jj].entity_max.x ;
            bool check3 =  seg_min.y > EntityProperties[jj].entity_min.y && seg_min.y < EntityProperties[jj].entity_max.y ;
            bool check4 =  seg_max.y > EntityProperties[jj].entity_min.y && seg_max.y < EntityProperties[jj].entity_max.y ;

            if ( check1 || check2 || check3 || check4 ) {
                possibleClusterEntityAssociations.push_back ( jj );
            }


        }


        //}


   std::cout << "Before possible associsations" << std::endl;

       // If a cluster could be associated to a (set of) entities, determine for each point to which entitiy it belongs based on a shortest distance criterion. If the distance is too large, initiate a new entity
        std::vector<geo::Vec2f> pointsNotAssociated;
        //for ( unsigned int ii = 0; ii < num_beams; ++ii ) { // TODO relevant beams only
	for ( unsigned int i_points = 0; i_points < points.size(); ++i_points ) { // Determine closest object and distance to this object. If distance too large, relate to new object
	

            //geo::Vector3 p = lrf_model_.rayDirections() [ii] * sensor_ranges[ii];
	  geo::Vec2f p = points[i_points];
            float shortestDistance = std::numeric_limits< float >::max();
            unsigned int id_shortestEntity = 0;

            for ( unsigned int jj = 0; jj < possibleClusterEntityAssociations.size(); ++jj ) { // relevant entities only
                //ed::WorldModel::const_iterator itEntity = it_laserEntities[jj];
                const ed::EntityConstPtr& e = *it_laserEntities[jj];
                // get type -> rectangle or circle
                // determine distance

                ed::tracking::FeatureProperties featureProperties = e->property ( featureProperties_ );
                float dist;
                if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() ) { // entity is considered to be a circle
                    ed::tracking::Circle circle = featureProperties.getCircle();
                    dist = std::abs ( std::sqrt ( std::pow ( p.x - circle.get_x(), 2.0 ) + std::pow ( p.y - circle.get_y(), 2.0 ) -   circle.get_R() ) ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                } else { // entity is considered to be a rectangle. Check if point is inside the rectangle
                    ed::tracking::Rectangle rectangle = featureProperties.getRectangle();

		    std::vector<geo::Vec2f> corners = rectangle.determineCorners( 0.0 );
                  /*  float x = rectangle.get_x();
                    float y = rectangle.get_y();
                    float theta = rectangle.get_theta();
                    float dx = 0.5*rectangle.get_w();
                    float dy = 0.5*rectangle.get_d();

                    float ct = cos ( theta );
                    float st = sin ( theta );

                    geo::Vec2f originCorner ( x + ct*-dx + st* dy, y + st*-dx + ct*-dy ); // Rotation matrix
                    geo::Vec2f corner1 ( x + ct* dx + st* dy, y + st* dx + ct*-dy );
                    geo::Vec2f corner2 ( x + ct* dx - st* dy, y + st*-dx + ct* dy );
                    geo::Vec2f corner3 ( x + ct*-dx - st* dy, y + st*-dx + ct* dy );*/

                    geo::Vec2f vx = corners[1] - corners[0];
                    geo::Vec2f vy = corners[3] - corners[0];

                    geo::Vec2f pCorrected = p-corners[0];

                    // Test if point is inside rectangle https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
                    geo::Vec2f OP = pCorrected - corners[0]; // Distance from origin to point which is tested
                    geo::Vec2f OC1 = corners[1] - corners[0];
                    geo::Vec2f OC3 = corners[3] - corners[0];

                    float OP_OC1   = OP.dot ( OC1 ); // elementwise summation
                    float OC1_OC1B = OC1.dot ( OC1 );
                    float OP_OC3   = OP.dot ( OC3 );
                    float OC3_OC3  = OC3.dot ( OC3 );

                    float minDistance = std::numeric_limits< float >::infinity();

                    if ( OP_OC1 > 0 && OC1_OC1B > OP_OC1 && OP_OC3 > 0 && OC3_OC3 > OP_OC3 ) { // point is inside the rectangle

                        std::vector<geo::Vec2f> p1Check = corners;

                        std::vector<geo::Vec2f> p2Check = corners; // place last element at begin
			corners.insert(corners.begin(), corners.back());
			corners.erase(corners.end());

                        for ( unsigned int ii_dist = 0; ii_dist < p1Check.size(); ii_dist++ ) {

                            float x1 = p1Check[ii_dist].x;
                            float x2 = p2Check[ii_dist].x;

                            float y1 = p1Check[ii_dist].y;
                            float y2 = p2Check[ii_dist].y;

                            float distance = std::abs ( ( y2 - y1 ) *p.x - ( x2 - x1 ) *p.y + x2*y1 -y2*x1 ) /std::sqrt ( std::pow ( y2-y1, 2.0 ) + std::pow ( x2-x1, 2.0 ) );

                            if ( distance < minDistance ) {
                                minDistance = distance;
                            }
                        }
                    } else { // point is outside the rectangle, https://stackoverflow.com/questions/44824512/how-to-find-the-closest-point-on-a-right-rectangular-prism-3d-rectangle/44824522#44824522
                        float tx = pCorrected.dot ( vx ) / ( vx.dot ( vx ) );
                        float ty = pCorrected.dot ( vx ) / ( vx.dot ( vx ) );

                        tx = tx < 0 ? 0 : tx > 1 ? 1 : tx;
                        ty = ty < 0 ? 0 : ty > 1 ? 1 : ty;

                        geo::Vec2f closestPoint = tx*vx + ty*vy + corners[0];

                        geo::Vec2f vector2Point = p - closestPoint;
                        minDistance = std::sqrt ( vector2Point.dot ( vector2Point ) );
                    }
                }

                if ( dist < shortestDistance) {
                    shortestDistance = dist;
                    id_shortestEntity = jj;
                }
            }

            if(shortestDistance < MIN_ASSOCIATION_DISTANCE){ // point related to an entity
	      pointsAssociatedList.at ( id_shortestEntity ).push_back ( p );
	    } else { // new entity should be initiated
	      pointsNotAssociated.push_back((p));
	    }
        }
        
	  pointsAssociatedList.push_back(pointsNotAssociated); // check on minimum size will be when the properties should be determined.
    }


   std::cout << "Per beam association" << std::endl;
// - - - - - - - - - - - - - - - - - -
// Convert the segments to convex hulls, and check for collisions with other convex hulls
    std::vector<EntityUpdate> clusters;
    std::vector<ed::tracking::FeatureProperties> measuredProperties; // The first sequence in this vector (with the length of laser entitities) are the properties corresponding to existing entities
    
    std::cout << "Bla1" << std::endl;
 //   int counter = 0;
    int initialSize = segments.size();

    // Determine the properties for each entity using the points which were associated in the previous step.
    //for ( unsigned int iSegment = 0; iSegment < segments.size(); ++iSegment ) {
    for(unsigned int iList = 0; iList < pointsAssociatedList.size(); iList++){ 
        //  bool splitSegmentsFlag = false;
std::cout << "Bla2" << std::endl;
   //     counter++;
        //std::vector<ScanSegment>::iterator it_test = segments.begin();
       // ScanSegment& segment = segments[iSegment];
        //unsigned int segment_size = segment.size();

       // std::vector<geo::Vec2f> points ( segment_size );

	std::vector<geo::Vec2f> points = pointsAssociatedList[iList];
	
	std::cout << "points.size() initially = " << points.size() << " minclusterSize = " << min_segment_size_pixels_ << " Bool = " << (points.size() < min_segment_size_pixels_ )<< std::endl;
	if(points.size() < min_segment_size_pixels_) // Add an element to measured properties such that the element in the measuredProperties-vector still corresponds with the laser-entity-vector
	{  std::cout << "Bla2.1" << std::endl;
            ed::tracking::FeatureProperties properties; // values initialized with nan now
            measuredProperties.push_back ( properties );
	    continue;
	}
	std::cout << "Bla3" << std::endl;
	
       /* float z_min, z_max;

        for ( unsigned int i = 0; i < segment_size; ++i ) {
            unsigned int j = segment[i];

            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[i] = geo::Vec2f ( p.x, p.y ); // TODO Is there a need to recompute the points again?

            if ( i == 0 ) {
                z_min = p.z;
                z_max = p.z;
            } else {
                z_min = std::min<float> ( z_min, p.z );
                z_max = std::max<float> ( z_max, p.z );
            }
        }
*/
        /*     int chull_size = points.size();
             std::cout << " \n Points = " << std::endl;
             for ( unsigned int i_print = 0; i_print < chull_size ; i_print++ ) {
                 std::cout << points[i_print] << ";" << std::endl;
             }
             std::cout << std::endl;
          */

        // check here for possible corners and splitting
        std::vector<unsigned int> cornerIndices;
        std::vector<ScanSegment> segmentsSplitted;
        std::vector< std::vector<geo::Vec2f> > pointsList;
        std::vector<unsigned int> cornersAfterSplitting;
        unsigned int cornerIndicesID;
std::cout << "Bla3.1" << std::endl;
std::cout << "poinst size = " << points.size() << std::endl;

bool test = ed::tracking::findPossibleCorners ( points, &cornerIndices );
std::cout << "poinst size = " << points.size() << "ncornerIdices = " << cornerIndices.size() << std::endl;

        if ( ! test) { // if no corners detected, add the points to the points-vector
	  	    std::cout << "Bla4" << std::endl;
            pointsList.push_back ( points );
            cornersAfterSplitting.push_back ( std::numeric_limits<unsigned int>::quiet_NaN() );

        } else { // if corners detected, check if a valid region is described. If not, split these data
std::cout << "Bla5" << std::endl;
            cornerIndices.insert ( cornerIndices.begin(), 0 ); // add index of first point of "points"
            cornerIndices.push_back ( points.size() - 1 );// add index of last point of "points"

            std::vector<geo::Vec2f> pointsToCheck;
            for ( unsigned int iIndex = 0; iIndex < cornerIndices.size(); ++iIndex ) {
                pointsToCheck.push_back ( points[cornerIndices[iIndex]] );
		std::cout << "Bla6" << std::endl;
            }

            while ( pointsToCheck.size() >= 2 ) {
	      std::cout << "Bla7" << std::endl;
                unsigned int ID_low = cornerIndices[0], ID_high;

                unsigned int cornerAfterSplitting;
                if ( pointsToCheck.size() == 2 ) {
                    cornerIndicesID = 1;
                    ID_high = cornerIndices[cornerIndicesID];
                    cornerAfterSplitting = std::numeric_limits<unsigned int>::quiet_NaN();
                } else {
		  std::cout << "Bla8" << std::endl;
                    geo::Vec2f startPoint = pointsToCheck[0];
                    geo::Vec2f midPoint = pointsToCheck[1];
                    geo::Vec2f endPoint = pointsToCheck[2];

                    geo::Vec2f centerPoint = 0.5* ( startPoint + endPoint );

                    float distMidPoint2 = pow ( midPoint.x-sensor_pose.getOrigin().getX(), 2.0 ) + pow ( midPoint.y-sensor_pose.getOrigin().getY(), 2.0 );
                    float distcenterPoint2 = pow ( centerPoint.x-sensor_pose.getOrigin().getX(), 2.0 ) + pow ( centerPoint.y-sensor_pose.getOrigin().getY(), 2.0 );


                    if ( distcenterPoint2 <  distMidPoint2 ) { // split data at midpoint, because otherwise we will describe a square at a region which is not necessarily occupied
                        cornerIndicesID = 1;
                        ID_high = cornerIndices[cornerIndicesID];
                        cornerAfterSplitting = std::numeric_limits<unsigned int>::quiet_NaN();
                    } else {
                        cornerIndicesID = 2;
                        ID_high = cornerIndices[cornerIndicesID];
                        cornerAfterSplitting = cornerIndices[1] - ID_low;
                    }
                }
std::cout << "Bla9" << std::endl;
                if ( ID_high - ID_low >= min_segment_size_pixels_ ) {
                    geo::Vec2f seg_min, seg_max;
                    std::vector<geo::Vec2f> pointsLow;
                    for ( unsigned int i_seg = ID_low; i_seg <= ID_high; ++i_seg ) {
                        geo::Vec2f p ( points[i_seg].x,  points[i_seg].y ) ; // TODO Right z-coordinate? Not used at the moment!
                        pointsLow.push_back ( p );

                        if ( i_seg == ID_low ) {
                            seg_min = p;
                            seg_max = p;
                        } else {
                            seg_min.x = std::min ( p.x, seg_min.x );
                            seg_min.y = std::min ( p.y, seg_min.y );
                            seg_max.x = std::max ( p.x, seg_max.x );
                            seg_max.y = std::max ( p.y, seg_max.y );
                        }
                    }

                    geo::Vec2f bb = seg_max - seg_min;

                    if ( ( bb .x > min_cluster_size_ || bb.y > min_cluster_size_ ) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_ ) {
                        pointsList.push_back ( pointsLow );
                    }
                }

                std::cout << "Bla10" << std::endl;
                pointsToCheck.erase ( pointsToCheck.begin(), pointsToCheck.begin() + cornerIndicesID );
                cornerIndices.erase ( cornerIndices.begin(), cornerIndices.begin() + cornerIndicesID );
                cornersAfterSplitting.push_back ( cornerAfterSplitting );
            }
        }
std::cout << "Bla11" << std::endl;
        for ( unsigned int iPointsList = 0; iPointsList < pointsList.size(); ++iPointsList ) {
            points = pointsList[iPointsList];
            unsigned int cornerIndex = cornersAfterSplitting[iPointsList];

            clusters.push_back ( EntityUpdate() );
            EntityUpdate& cluster = clusters.back();

            cluster.pose = geo::Pose3D::identity();
           // ed::convex_hull::create ( points, z_min, z_max, cluster.chull, cluster.pose );

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

            measuredProperties.push_back ( properties );

	    std::cout << "Bla12" << std::endl;
            // TODO: cleanup: remove objects which are fitted and clearly interfere with the walls -> more robustness on segmentation. Where to check?

            // --------------------------
            // Temp for RoboCup 2016; todo: remove after

            // Determine the cluster size
            geo::Vec2f diff = points.back() - points.front();
            float size_sq = diff.length2();
            if ( size_sq > 0.35 * 0.35 && size_sq < 0.8 * 0.8 ) {
                cluster.flag = "possible_human";
            }
        }

        // --------------------------
    }

    std::cout << "After determining properties measuredProperties size = " << measuredProperties.size() << "laser-associations.size = " << it_laserEntities.size() << std::endl;
    
    // pointsAssociatedList.size() is the number of entities being tracked by the laser. The first set of elements in measured-properties, namely with this pointsAssociatedList.size(), should correspond to these entities.
    // TODO: if there are no points associated to an entity, an empty element should be added to measuredProperties indicating that there are no associations.	
    // TODO: what to do when data, which are associated to an entity, are splitted? Which part should be pushed back at the end of the measured-properties list and which part should associate to the existing entity?
    // TODO: update properties of the entities. TODO: determine covariance-matrices
    // TODO: set existence probability of the entities
    // TODO: check which properties should be maintained, such as geometries, timestamp-update
    // TODO: remove chull-properties
    // TODO: Initiate new entities
    // TODO: Cleanup of source code

// Create selection of world model entities that could associate
/*
    float max_dist = 0.3;

    std::vector<ed::EntityConstPtr> entities;
    if ( !clusters.empty() ) {
        geo::Vec2 area_min ( clusters[0].pose.t.x, clusters[0].pose.t.y );
        geo::Vec2 area_max ( clusters[0].pose.t.x, clusters[0].pose.t.y );
        for ( std::vector<EntityUpdate>::const_iterator it = clusters.begin(); it != clusters.end(); ++it ) {
            const EntityUpdate& cluster = *it;

            area_min.x = std::min ( area_min.x, cluster.pose.t.x );
            area_min.y = std::min ( area_min.y, cluster.pose.t.y );

            area_max.x = std::max ( area_max.x, cluster.pose.t.x );
            area_max.y = std::max ( area_max.y, cluster.pose.t.y );
        }

        area_min -= geo::Vec2 ( max_dist, max_dist );
        area_max += geo::Vec2 ( max_dist, max_dist );

        for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it ) {
            const ed::EntityConstPtr& e = *e_it;
            if ( e->shape() || !e->has_pose() ) {
                continue;
            }

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            if ( entity_chull.points.empty() ) {
                continue;
            }

            //            if (e->existenceProbability() < 0.5 && scan_msg_->header.stamp.toSec() - e->lastUpdateTimestamp() > 1.0) // TODO: magic numbers
            //            {
            //                req.removeEntity(e->id());
            //                continue;
            //            }

            if ( entity_pose.t.x < area_min.x || entity_pose.t.x > area_max.x
                    || entity_pose.t.y < area_min.y || entity_pose.t.y > area_max.y ) {
                continue;
            }

            entities.push_back ( e );
        }
    }

// Create association matrix
    ed_sensor_integration::AssociationMatrix assoc_matrix ( clusters.size() );
    for ( unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster ) {
        const EntityUpdate& cluster = clusters[i_cluster];

        for ( unsigned int i_entity = 0; i_entity < entities.size(); ++i_entity ) {
            const ed::EntityConstPtr& e = entities[i_entity];

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            float dx = entity_pose.t.x - cluster.pose.t.x;
            float dy = entity_pose.t.y - cluster.pose.t.y;
            float dz = 0;

            if ( entity_chull.z_max + entity_pose.t.z < cluster.chull.z_min + cluster.pose.t.z
                    || cluster.chull.z_max + cluster.pose.t.z < entity_chull.z_min + entity_pose.t.z )
                // The convex hulls are non-overlapping in z
            {
                dz = entity_pose.t.z - cluster.pose.t.z;
            }

            float dist_sq = ( dx * dx + dy * dy + dz * dz );

            // TODO: better prob calculation
            double prob = 1.0 / ( 1.0 + 100 * dist_sq );

            double dt = scan->header.stamp.toSec() - e->lastUpdateTimestamp();

            double e_max_dist = std::max ( 0.2, std::min ( 0.5, dt * 10 ) );

            if ( dist_sq > e_max_dist * e_max_dist ) {
                prob = 0;
            }

            if ( prob > 0 ) {
                assoc_matrix.setEntry ( i_cluster, i_entity, prob );
            }
        }
    }

    ed_sensor_integration::Assignment assig;
    if ( !assoc_matrix.calculateBestAssignment ( assig ) ) {
        std::cout << "WARNING: Association failed!" << std::endl;
        return;
    }
    */

    //std::vector<int> entities_associated ( entities.size(), -1 );

    unsigned int marker_ID = 0; // To Do: After tracking, the right ID's should be created. The ID's are used to have multiple markers.

    
    
    //measuredProperties;
    
    //it_laserEntities;
    
    ed::tracking::FeatureProperties measuredProperty, entityProperties; // Measured properties and updated properties
    ed::UUID id;
    
    for ( unsigned int i_properties = 0; i_properties < measuredProperties.size(); i_properties++ ) {
        measuredProperty = measuredProperties[i_properties];

	std::cout << "i_properties = " << i_properties << std::endl;
	
        if ( i_properties < it_laserEntities.size() ) {
	  const ed::EntityConstPtr& e = * ( it_laserEntities[i_properties] );

	  std::cout << "Hatseflats" << std::endl;
            // check if new properties are measured.
            bool check1 = measuredProperty.getCircle().get_R() != measuredProperty.getCircle().get_R();
            bool check2 = measuredProperty.getRectangle().get_w() != measuredProperty.getRectangle().get_w();
            bool check3 = measuredProperty.getRectangle().get_d() != measuredProperty.getRectangle().get_d();
std::cout << "Hatseflats 2" << std::endl;
            if ( check1 || ( check2 && check3 ) ) {
                //Clear unassociated entities in view
std::cout << "Hatseflat3" << std::endl;
                const geo::Pose3D& pose = e->pose();

                // Transform to sensor frame
                geo::Vector3 p = sensor_pose.inverse() * pose.t;

                if ( !pointIsPresent ( p, lrf_model_, sensor_ranges ) ) {
                    double p_exist = e->existenceProbability();
                    if ( p_exist < 0.3 ) { // TODO: magic number
                        req.removeEntity ( e->id() );
			
			std::cout << "Entity removed " << std::endl;
                    } else {
                        req.setExistenceProbability ( e->id(), std::max ( 0.0, p_exist - 0.1 ) ); // TODO: very ugly prob update
                    }
                }
std::cout << "Hatseflats4. We are going to continue!" << std::endl;
                continue;
            }

            // check if new information is available and update properties
std::cout << "Hatseflat5" << std::endl;
            if ( !e->hasFlag ( "locked" ) ) {
                ed::tracking::FeatureProperties propertiesMeasured = measuredProperties[i_properties];
                entityProperties = e->property ( featureProperties_ );

                float Q = 0.1; // Measurement noise covariance. TODO: let it depend on if an object is occluded. Now, objects are assumed to be completely visible
                float R = 0.0; // Process noise covariance

                Eigen::MatrixXd Qm ( 2, 2 ), Rm ( 2, 2 );
                Eigen::VectorXd z_k ( 2, 1 );

                Qm << Q, 0, 0, Q;
                Rm << R, 0, 0, R;
		
		std::cout << "Plugin.cpp: Qm = " << Qm << " Rm = " << Rm << std::endl;
		
                z_k << propertiesMeasured.getRectangle().get_w(), propertiesMeasured.getRectangle().get_d();

                entityProperties.updateProbabilities ( propertiesMeasured.getFeatureProbabilities() ); // TODO: update probabilities of the features -> Do we still need to use them? Because this will be solved in the PF
                entityProperties.updateCircleFeatures ( Q, R, propertiesMeasured.getCircle().get_R() );
                entityProperties.updateRectangleFeatures ( Qm, Rm, z_k );
            }


            // Update existence probability
            double p_exist = e->existenceProbability();
            req.setExistenceProbability ( e->id(), std::min ( 1.0, p_exist + 0.1 ) ); // TODO: very ugly prob update

            id = e->id();

        } else {
            // create a new entity
            // Generate unique ID
            id = ed::Entity::generateID().str() + "-laser";

            // Update existence probability
            req.setExistenceProbability ( id, 1.0 ); // TODO magic number

            entityProperties = measuredProperty;

        }
        
        // determine convex hull and pose
        ed::ConvexHull new_chull;
        geo::Pose3D new_pose;
	
	if( entityProperties.getFeatureProbabilities().get_pCircle() < entityProperties.getFeatureProbabilities().get_pRectangle() ){
	 // determine corners 
	  ed::tracking::Rectangle rectangle = entityProperties.getRectangle();
	  std::vector<geo::Vec2f> corners = entityProperties.getRectangle().determineCorners( 0.0 );
	  new_pose = rectangle.getPose();
	  
	  ed::convex_hull::create ( corners, sensor_pose.getOrigin().getZ(), sensor_pose.getOrigin().getZ(), new_chull, new_pose ); // height assumed to be similar to sensor-height
	  
	  
	} else {
	 // determine cilinder-properties 
	   ed::tracking::Circle circle = entityProperties.getCircle();
	  new_pose = circle.getPose();
	  std::vector< geo::Vec2f > convexHullPoints = circle.convexHullPoints(20);
	  
	  ed::convex_hull::create ( convexHullPoints, sensor_pose.getOrigin().getZ(), sensor_pose.getOrigin().getZ(), new_chull, new_pose ); // height assumed to be similar to sensor-height
	}
    
    
        // Set feature properties en publish geometries
        req.setProperty ( id, featureProperties_, entityProperties );
	req.setConvexHullNew ( id, new_chull, new_pose, scan->header.stamp.toSec(), scan->header.frame_id );
        publishFeatures ( entityProperties, marker_ID++, ObjectMarkers_pub_ );

        // Set timestamp
        req.setLastUpdateTimestamp ( id, scan->header.stamp.toSec() );
    }
    
  /*  
    for ( unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster ) {
        const EntityUpdate& cluster = clusters[i_cluster];

        // Get the assignment for this cluster
        int i_entity = assig[i_cluster];

        ed::UUID id;
        ed::ConvexHull new_chull;
        geo::Pose3D new_pose;

        ed::tracking::FeatureProperties featureProperty;

        if ( i_entity == -1 ) {
            // No assignment, so add as new cluster
            new_chull = cluster.chull;
            new_pose = cluster.pose;

            // Generate unique ID
            id = ed::Entity::generateID().str() + "-laser";

            // Update existence probability
            req.setExistenceProbability ( id, 1.0 ); // TODO magic number

            featureProperty = measuredProperties[i_cluster];
        } else {
            // Mark the entity as being associated
            entities_associated[i_entity] = i_cluster;

            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];
            //            const ed::ConvexHull& entity_chull = e->convexHullNew();
            //            const geo::Pose3D& entity_pose = e->pose();

            //            std::vector<geo::Vec2f> new_points_MAP;

            //            // Add the points of the cluster
            //            for(std::vector<geo::Vec2f>::const_iterator p_it = cluster.chull.points.begin(); p_it != cluster.chull.points.end(); ++p_it)
            //                new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster.pose.t.x, p_it->y + cluster.pose.t.y));

            //            // Add the entity points that are still present in the depth map (or out of view)
            //            for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin(); p_it != entity_chull.points.end(); ++p_it)
            //            {
            //                geo::Vec2f p_chull_MAP(p_it->x + entity_pose.t.x, p_it->y + entity_pose.t.y);

            //                geo::Vector3 p = sensor_pose.inverse() * geo::Vector3(p_chull_MAP.x, p_chull_MAP.y, entity_pose.t.z);

            //                if (pointIsPresent(p, lrf_model_, sensor_ranges))
            //                {
            //                    new_points_MAP.push_back(p_chull_MAP);
            //                }
            //            }

            //            double new_z_min = cluster.chull.z_min;
            //            double new_z_max = cluster.chull.z_max;

            //            // And calculate the convex hull of these points
            //            ed::convex_hull::create(new_points_MAP, new_z_min, new_z_max, new_chull, new_pose);

            if ( !e->hasFlag ( "locked" ) ) {
                new_chull = cluster.chull;
                new_pose = cluster.pose;

                ed::tracking::FeatureProperties propertiesMeasured = measuredProperties[i_cluster];
                featureProperty = e->property ( featureProperties_ );
                featureProperty.updateProbabilities ( propertiesMeasured.getFeatureProbabilities() ); // TODO: update properties of the features
                featureProperty.setCircle ( propertiesMeasured.getCircle() ); // TODO determine a proper method to update the circle and rectangle properties
                featureProperty.setRectangle ( propertiesMeasured.getRectangle() ); // Now, the properties of the latest measurements are used
            }

            // Update existence probability
            double p_exist = e->existenceProbability();
            req.setExistenceProbability ( e->id(), std::min ( 1.0, p_exist + 0.1 ) ); // TODO: very ugly prob update

            // update feature probabilities here
            //req.setProperty<>(e->id());

            id = e->id();
        }

        // Set convex hull and pose
        if ( !new_chull.points.empty() ) {
            req.setConvexHullNew ( id, new_chull, new_pose, scan->header.stamp.toSec(), scan->header.frame_id );

            // --------------------------
            // Temp for RoboCup 2015; todo: remove after

            if ( !cluster.flag.empty() ) {
                req.setFlag ( id, cluster.flag );
            }

            // --------------------------
        }

        // Set feature properties en publish geometries
        req.setProperty ( id, featureProperties_, featureProperty );
        publishFeatures ( featureProperty, marker_ID++, ObjectMarkers_pub_ );

        // Set timestamp
        req.setLastUpdateTimestamp ( id, scan->header.stamp.toSec() );
    }
*/
// - - - - - - - - - - - - - - - - - 

 std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms." << std::endl;

}


// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback ( const sensor_msgs::LaserScan::ConstPtr& msg )
{
    //std::cout << "Received message @ timestamp " << ros::Time::now() << std::endl;

    scan_buffer_.push ( msg );
}


ED_REGISTER_PLUGIN ( LaserPlugin )
