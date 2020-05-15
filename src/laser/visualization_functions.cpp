#include "visualization_functions.h"

#define COLOR_SIZE 27
float COLORS[COLOR_SIZE][3] = { { 0.6, 0.6, 0.6},
                        { 0.6, 0.6, 0.4},
                        { 0.6, 0.6, 0.2},
                        { 0.6, 0.4, 0.6},
                        { 0.6, 0.4, 0.4},
                        { 0.6, 0.4, 0.2},
                        { 0.6, 0.2, 0.6},
                        { 0.6, 0.2, 0.4},
                        { 0.6, 0.2, 0.2},
                        { 0.4, 0.6, 0.6},
                        { 0.4, 0.6, 0.4},
                        { 0.4, 0.6, 0.2},
                        { 0.4, 0.4, 0.6},
                        { 0.4, 0.4, 0.4},
                        { 0.4, 0.4, 0.2},
                        { 0.4, 0.2, 0.6},
                        { 0.4, 0.2, 0.4},
                        { 0.4, 0.2, 0.2},
                        { 0.2, 0.6, 0.6},
                        { 0.2, 0.6, 0.4},
                        { 0.2, 0.6, 0.2},
                        { 0.2, 0.4, 0.6},
                        { 0.2, 0.4, 0.4},
                        { 0.2, 0.4, 0.2},
                        { 0.2, 0.2, 0.6},
                        { 0.2, 0.2, 0.4},
                        { 0.2, 0.2, 0.2}
                      };

visualization_msgs::Marker getMarker ( tracking::FeatureProperties& featureProp, int ID) // TODO move to ed_rviz_plugins?
{
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;

        color.r = 0;
        color.g = 255;
        color.b = 0;
        color.a = ( float ) 0.5;

        if ( featureProp.getFeatureProbabilities().get_pCircle() > featureProp.getFeatureProbabilities().get_pRectangle() )
        {
            tracking::Circle circle = featureProp.getCircle();
            circle.setMarker ( marker , ID, color );
        }
        else
        {
            tracking::Rectangle rectangle = featureProp.getRectangle();
            rectangle.setMarker ( marker , ID, color );
        }
        
         if(marker.pose.position.x != marker.pose.position.x || marker.pose.position.y != marker.pose.position.y || marker.pose.position.z != marker.pose.position.z ||
             marker.pose.orientation.x !=  marker.pose.orientation.x || marker.pose.orientation.y !=  marker.pose.orientation.y || marker.pose.orientation.z !=  marker.pose.orientation.z ||
             marker.pose.orientation.w !=  marker.pose.orientation.w || marker.scale.x != marker.scale.x || marker.scale.y != marker.scale.y || marker.scale.z != marker.scale.z )
     {
             
                featureProp.printProperties();
                ROS_WARN( "Publishing of object with nan" ); 
                
                exit (EXIT_FAILURE);
        }
        
    return marker;
}

void pubPoints ( visualization_msgs::MarkerArray *markerArray, std::vector<geo::Vec2f> points, unsigned int *ID )
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

    marker.lifetime = ros::Duration ( MARKER_TIMEOUT_TIME );

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

visualization_msgs::Marker visualizePoints( std::vector< geo::Vec2f> points, std::string frame_id, std::string nameSpace, ros::Time timeStamp, float height, int colorID)
{
    int color = colorID % COLOR_SIZE;

    visualization_msgs::Marker pointsMarker;

    pointsMarker.header.frame_id = frame_id;
    pointsMarker.header.stamp = timeStamp;
    pointsMarker.ns = nameSpace;
    pointsMarker.id = 1;
    pointsMarker.type = visualization_msgs::Marker::POINTS;
    pointsMarker.action = visualization_msgs::Marker::ADD;
    pointsMarker.scale.x = 0.1;
    pointsMarker.scale.y = 0.1;
    pointsMarker.scale.z = 0.1;
    pointsMarker.color.r = COLORS[color][0];
    pointsMarker.color.g = COLORS[color][1];
    pointsMarker.color.b = COLORS[color][2];
    pointsMarker.color.a = 1.0;
    pointsMarker.lifetime = ros::Duration(MARKER_TIMEOUT_TIME);

    for (int iPoints = 0; iPoints < points.size(); iPoints++)
    {
        geometry_msgs::Point p;

        p.x = points[iPoints].x;
        p.y = points[iPoints].y;
        p.z = height;

        pointsMarker.points.push_back(p);
    }

    return pointsMarker;
}