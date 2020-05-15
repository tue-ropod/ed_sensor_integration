#include "plugin_support_functions.h"

bool sortBySegmentSize(const tracking::ScanSegment &lhs, const tracking::ScanSegment &rhs)
{
    return lhs.size() > rhs.size(); 
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

bool pointIsPresent(double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    int i_beam = lrf.getAngleUpperIndex(x_sensor, y_sensor);
    if (i_beam < 0 || i_beam >= sensor_ranges.size())
        return true; // or actually, we don't know

    float rs = sensor_ranges[i_beam];
    return rs == 0 || geo::Vec2(x_sensor, y_sensor).length() > rs - 0.1;
}

bool pointIsPresent(const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    return pointIsPresent(p_sensor.x, p_sensor.y, lrf, sensor_ranges);
}

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
                      float x_window, float x_step, float y_window, float y_step, float yaw_min, float yaw_plus, 
                      float yaw_step, std::map<ed::UUID,geo::Pose3D>& pose_cache)
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

void addEvidenceWIRE(wire_msgs::WorldEvidence& world_evidence, tracking::FeatureProperties measuredProperty )
// Converter from measured properties to WIRE-info
{
        wire_msgs::Property properties;
        properties.attribute = "positionAndDimension";  
        
        std::shared_ptr<pbl::Gaussian> zRectangle = std::make_shared<pbl::Gaussian>(RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE);
        zRectangle->setMean(measuredProperty.rectangle_.get_H()* measuredProperty.rectangle_.getState());
        zRectangle->setCovariance(measuredProperty.rectangle_.get_H()* measuredProperty.rectangle_.getCovariance()*measuredProperty.rectangle_.get_H().t());
        
        std::shared_ptr<pbl::Gaussian> zCircle = std::make_shared<pbl::Gaussian>(CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE);
        zCircle->setMean(measuredProperty.circle_.get_H()* measuredProperty.circle_.getState());
        zCircle->setCovariance(measuredProperty.circle_.get_H()* measuredProperty.circle_.getCovariance()*measuredProperty.circle_.get_H().t());

        pbl::Hybrid  hyb;
        hyb.addPDF(*zRectangle,measuredProperty.getFeatureProbabilities().get_pRectangle());
        hyb.addPDF(*zCircle, measuredProperty.getFeatureProbabilities().get_pCircle());
        
        pbl::PDFtoMsg(hyb, properties.pdf);
        wire_msgs::ObjectEvidence obj_evidence;
        obj_evidence.properties.push_back(properties);
        
        world_evidence.object_evidence.push_back(obj_evidence);
}

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

unsigned int determineClosestObject(geo::Vec2f point, float *shortestDistance, float timeStampScan,
                        std::vector< int > possibleSegmentEntityAssociations, 
                        std::vector<ed::WorldModel::const_iterator> it_laserEntities, 
                        ed::PropertyKey<tracking::FeatureProperties> featurePropertiesKey)
{
            *shortestDistance = std::numeric_limits< float >::max();

            unsigned int id_shortestEntity = std::numeric_limits< unsigned int >::max();

            for ( unsigned int jj = 0; jj < possibleSegmentEntityAssociations.size(); jj++ )  // relevant entities only
            {
                const ed::EntityConstPtr& e = *it_laserEntities[ possibleSegmentEntityAssociations[jj] ];

                tracking::FeatureProperties featureProperties = e->property ( featurePropertiesKey );

                float dist;
                float dt = timeStampScan - e->lastUpdateTimestamp();

                if ( featureProperties.getFeatureProbabilities().get_pCircle() > featureProperties.getFeatureProbabilities().get_pRectangle() ) 
                {
                    tracking::Circle circle = featureProperties.getCircle();  
                    circle.predictAndUpdatePos( dt ); // TODO Do this update once at initialisation
                    dist = std::abs ( std::sqrt ( std::pow ( point.x - circle.get_x(), 2.0 ) + std::pow ( point.y - circle.get_y(), 2.0 ) ) - circle.get_radius() ); // Distance of a point to a circle, see https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
                }
                else     // entity is considered to be a rectangle. Check if point is inside the rectangle
                {
                    tracking::Rectangle rectangle = featureProperties.getRectangle();
                    rectangle.predictAndUpdatePos( dt );// TODO Do this update once at initialisation

                    std::vector<geo::Vec2f> corners = rectangle.determineCorners ( 0.0 );

                    geo::Vec2f vx = corners[1] - corners[0];
                    geo::Vec2f vy = corners[3] - corners[0];

                    geo::Vec2f pCorrected = point - corners[0];

                    // Test if point is inside rectangle https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
                    geo::Vec2f OP = point - corners[0]; // Distance from origin to point which is tested
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

                            float distance = std::abs ( ( y2 - y1 ) *point.x - ( x2 - x1 ) *point.y + x2*y1 -y2*x1 ) /std::sqrt ( std::pow ( y2-y1, 2.0 ) + std::pow ( x2-x1, 2.0 ) );
                            
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

                        geo::Vec2f vector2Point = point - closestPoint;
                        minDistance = std::sqrt ( vector2Point.dot ( vector2Point ) );
                    }

                    dist = minDistance;
                }
                
                if ( dist < *shortestDistance )
                {
                        *shortestDistance = dist;
                        id_shortestEntity = jj;
                }
            }

//             std::cout << "shortestDistance for point " << points[i_points] << " = " << shortestDistance << " to entity = " << id_shortestEntity << std::endl;

            //distances[i_points] = shortestDistance;
            //IDs[i_points] = id_shortestEntity;

            return id_shortestEntity;
}