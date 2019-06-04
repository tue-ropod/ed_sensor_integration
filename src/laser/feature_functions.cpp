
#include "feature_functions.h"

namespace ed
{


namespace tracking
{
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.75.5153&rep=rep1&type=pdf
void determineIAV(std::vector<float> ranges, float* mean, float* standardDeviation, geo::LaserRangeFinder lrf_model, unsigned int firstElement, unsigned int finalElement )
 // Internal Angle Variance
// TODO used at the moment?
{
        geo::Vec2f A, C;
        
        float angle = lrf_model.getAngleMin() + lrf_model.getAngleIncrement()*firstElement;
        A.x = ranges[firstElement]*cos(angle);
        A.y = ranges[firstElement]*sin(angle);
        
        angle = lrf_model.getAngleMin() + lrf_model.getAngleIncrement()*finalElement;
        C.x = ranges[finalElement]*cos(angle);
        C.y = ranges[finalElement]*sin(angle);
        
        std::vector <float> inscribedAngles(finalElement - firstElement - 2, 0.0);
        unsigned int counter = 0;
        *mean = 0.0;
        
        for( unsigned int ii = firstElement + 1; ii < finalElement - 1; ii++ )
        {
                geo::Vec2f B;
                
                angle = lrf_model.getAngleMin() + lrf_model.getAngleIncrement()*ii;
                B.x = ranges[ii]*cos(angle);
                B.y = ranges[ii]*sin(angle);
                
                float a2 = pow( B.x-C.x, 2.0 ) + pow( B.y-C.y, 2.0 );
                float b2 = pow( A.x-C.x, 2.0 ) + pow( A.y-C.y, 2.0 );
                float c2 =  pow( A.x-B.x, 2.0 ) + pow( A.y-B.y, 2.0 );
                
                float a = sqrt ( a2 );
                float c = sqrt ( c2 );
                
                inscribedAngles[counter] = acos( (b2 - a2 - c2) / (-2*a*c) );
                *mean += inscribedAngles[counter];
                counter++;    
        }
        *mean /= counter;
        
        counter = 0;
        *standardDeviation = 0.0;
        
        for( unsigned int ii = firstElement + 1; ii < finalElement - 1; ii++ )
        {
                *standardDeviation += std::pow( inscribedAngles[counter] - *mean, 2.0 );
                counter++;    
        }
        
        *standardDeviation = std::sqrt( *standardDeviation / counter );
}

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine )
{
    // Determine is a line or a rectangle should be fitted. In case of a rectangle, the number of elements for both sides should meet the minimum number of points for a line fit
    // for both lines. Otherwise, a line will be fitted on the remaining points.

    *it_low = points.begin();
    *it_high = points.end();

    bool includeCorner = *cornerIndex > 0;
    unsigned int remainingSize = points.size();
    
    // In the case of including a corner, check if both segments have enough points to describe a line with. If not, do not use these data.
    if ( includeCorner ) {
        unsigned int nPointsLow = *cornerIndex + 1; // + 1 because the corner can be used for both lines
        unsigned int nPointsHigh = points.size() - *cornerIndex;
        
        bool fitSingleline = false;
        bool pointsRemoved = false;

        if ( nPointsLow < minPointsLine ) 
        { // Part of section too smal -> remove it from the data which are analyzed and try to fit line again
            *it_low += *cornerIndex;
            remainingSize = nPointsHigh;
            pointsRemoved = true;
        }
        
        if ( nPointsHigh < minPointsLine ) 
        {
            *it_high -= nPointsHigh;
            remainingSize = nPointsLow;
            pointsRemoved = true;
        }       

        if ( pointsRemoved && remainingSize < minPointsLine ) 
        {
            *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
	    return NONE;	    
        }
        else if ( pointsRemoved && remainingSize >= minPointsLine ) 
        {
            *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
            return LINE;
        }
        else  
        { // we dit not remove points and a corner is present
            return RECTANGLE;
        }
    }
    else if(remainingSize >= minPointsLine)
    {
        return LINE;
    }
    else{
        return NONE;
    }
        return NONE;
}

float fitObject ( std::vector<geo::Vec2f>& points, int FITTINGMETHOD,  unsigned int* cornerIndex, ed::tracking::Rectangle* rectangle, ed::tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine )
{
    switch ( FITTINGMETHOD )
    {
    case NONE:
    {
        return std::numeric_limits<float>::infinity();
    }
    case LINE:
    {
        return setRectangularParametersForLine ( points,  it_low,  it_high, rectangle, sensor_pose, minPointsLine );
    }
    case CIRCLE:
    {
        return fitCircle ( points, circle, sensor_pose );
    }
    case RECTANGLE:
    {
        return fitRectangle ( points, rectangle, sensor_pose , *cornerIndex, minPointsLine );
    }
    }
    return false; // end reached without doing something
}

bool determineCornerConfidence(const sensor_msgs::LaserScan::ConstPtr& scan, unsigned int element, bool checkElementLow) 
{
        unsigned int num_beams = scan->ranges.size();
        unsigned int nPointsToCheck = POINTS_TO_CHECK_CONFIDENCE;

        if(checkElementLow)
        {
                if ( element < nPointsToCheck )
                {
                        // Because we have no proof that the edges of the object are observed
                        return false;
                }
                else
                {
                        
                        float rsToCheck = scan->ranges[element];
                        for ( unsigned int l = element - nPointsToCheck; l < element; l++ )
                        {
                                float rsToCompare = scan->ranges[l];
                                if ( rsToCheck > rsToCompare + LASER_ACCURACY && rsToCompare >= 0 + EPSILON )
                                {
                                        return false;
                                }
                        }
                }
                return true;    
        }
        
        else // we need to analyse elementHigh
        {
        
                if ( num_beams - element < nPointsToCheck )
                {
                        return false;
                }
                else
                {
                        float rsToCheck = scan->ranges[element];
                        for ( unsigned int l = element; l < element + nPointsToCheck; l++ )
                        {
                                float rsToCompare = scan->ranges[l];
                                if ( rsToCheck > rsToCompare + LASER_ACCURACY && rsToCompare >= 0 + EPSILON )
                                {
                                        return false;
                                }
                        }     
                }
                return true; 
        }
}

int maxCrossCorrelation(std::vector<float>& measuredRanges, std::vector<unsigned int>::iterator measuredRangesStartElement,  std::vector<unsigned int>::iterator measuredRangesFinalElement,
                        std::vector<float>& modelledRanges, std::vector<unsigned int>::iterator modelledRangesStartElement,  std::vector<unsigned int>::iterator modelledRangesFinalElement)
{
        unsigned int nModelledRanges = std::distance(modelledRangesStartElement, modelledRangesFinalElement);
        unsigned int nIterations = 2*(nModelledRanges - 1);
        int delta = -nModelledRanges + 1;
        
        float maxCorrelation = std::numeric_limits<float>::infinity();
        int deltaOptimal = 0;
        
        for (unsigned int ii = 0; ii < nIterations; ii++)
        {
                float crossCorrelation = 0.0;
                for(std::vector<unsigned int>::iterator it = measuredRangesStartElement; it != measuredRangesFinalElement; it++)
                {
                        float modelledRange, measuredRange;
                        unsigned int element = *it;
                        
                        measuredRange = measuredRanges[element];
                        int modelledElement = element + delta;
                        if(modelledElement < 0 || modelledElement > measuredRanges.size() )
                        {
                                modelledRange = 0.0; 
                        }
                        else 
                        {
                            
                            modelledRange = modelledRanges[modelledElement];
                        }

                        crossCorrelation += std::pow(modelledRange-measuredRange, 2.0);
                }
                
                if( crossCorrelation < maxCorrelation )
                {
                        maxCorrelation = crossCorrelation;
                        deltaOptimal = delta;
                }
                
                delta++;
        }
        
        return deltaOptimal;
}

geo::Vec2f avg ( std::vector<geo::Vec2f>& points, std::vector<geo::Vec2f>::const_iterator it_start, std::vector<geo::Vec2f>::const_iterator it_end )
{
    geo::Vec2f avg_point;
    avg_point.x = avg_point.y= 0.0;

    for ( std::vector<geo::Vec2f>::const_iterator it = it_start; it != it_end; ++it ) {
        geo::Vec2f point = *it;
        avg_point.x += point.x;
        avg_point.y += point.y;
    }

    unsigned int nElements = std::distance ( it_start, it_end );
    avg_point.x /= nElements;
    avg_point.y /= nElements;

    return ( avg_point );
}

geo::Vec2f projectPointOnLine(geo::Vec2f p1Line, geo::Vec2f p2Line, geo::Vec2f point2Project)
{
        float x1 = p1Line.x, x2 = p2Line.x, x3 = point2Project.x;
        float y1 = p1Line.y, y2 = p2Line.y, y3 = point2Project.y;
        
        float factor = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / (pow(y2-y1, 2.0) + pow(x2-x1, 2.0)); // https://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
        
        geo::Vec2f intersection;
        intersection.x = x3 - factor * (y2-y1); // Now, x, y corrected is on the edge in the depth dimension. The position still need to be corrected in the width dimension.
        intersection.y = y3 + factor * (x2-x1);
        
        return intersection;
}

Eigen::MatrixXf kalmanUpdate(Eigen::MatrixXf F, Eigen::MatrixXf H, Eigen::MatrixXf *P, Eigen::MatrixXf x_k_1_k_1, Eigen::MatrixXf z_k, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
    Eigen::MatrixXf I;
    I.setIdentity ( F.rows(), F.cols() );
    Eigen::MatrixXf x_k_k_1 = F*x_k_1_k_1;
    Eigen::MatrixXf P_k_k_1 = F* (*P) * F.transpose() + Q;
    Eigen::MatrixXf y_k = z_k - H*x_k_k_1;
    Eigen::MatrixXf S_k = H*P_k_k_1*H.transpose() + R;
    Eigen::MatrixXf K_k = P_k_k_1*H.transpose() *S_k.inverse();
    Eigen::MatrixXf x_k_k = x_k_k_1 + K_k*y_k;
    Eigen::MatrixXf P_k_k = ( I - K_k*H ) *P_k_k_1;  
    
    *P = P_k_k;
    
    return x_k_k;
}

//Fast Line, Arc/Circle and Leg Detection from Laser Scan Data in a Player Driver: http://miarn.sourceforge.net/pdf/a1738b.pdf
float fitCircle ( std::vector<geo::Vec2f>& points, ed::tracking::Circle* circle, const geo::Pose3D& pose )
{
    // according to https://dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
    float x_avg = 0.0, y_avg = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        x_avg += points[i].x;
        y_avg += points[i].y;
    }

    x_avg /= points.size();
    y_avg /= points.size();

    std::vector<float> ui ( points.size() ), vi ( points.size() );
    float Suu = 0.0, Suv = 0.0, Suuu = 0.0, Suvv = 0.0, Svv = 0.0, Svvv = 0.0, Svuu = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        ui[i] = points[i].x -x_avg;
        vi[i] = points[i].y -y_avg;

        Suu += ui[i]*ui[i];
        Suv += ui[i]*vi[i];
        Suuu += ui[i]*ui[i]*ui[i];
        Suvv += ui[i]*vi[i]*vi[i];

        Svv += vi[i]*vi[i];
        Svvv += vi[i]*vi[i]*vi[i];
        Svuu += vi[i]*ui[i]*ui[i];
    }

    float a = Suu;
    float b = Suv;
    float c = 0.5* ( Suuu+Suvv );
    float d = Suv;
    float e = Svv;
    float f = 0.5* ( Svvv+Svuu );

    float vc = ( f - c*d/a ) / ( e-b*d/a );
    float uc = ( c-vc*b ) /a;

    float xc = uc+x_avg;
    float yc = vc+y_avg;

    float alpha = uc*uc+vc*vc+ ( Suu+Svv ) /points.size();
    float radius = std::sqrt ( alpha );
     
    float sum = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) 
    {
        float error = fabs ( sqrt ( pow ( xc - points[i].x, 2.0 ) + pow ( yc - points[i].y, 2.0 ) ) - radius ); // distance between a point and a circle;
        float absError = sqrt( pow ( error, 2.0 ) );
        sum += absError;
    }

    float roll = 0.0, pitch = 0.0, yaw = 0.0;
    circle->setProperties ( xc, yc, pose.getOrigin().getZ(), roll, pitch, yaw, radius); // Assumption: object-height identical to sensor-height
    return sum/points.size();
}

Circle::Circle()
{
    float notANumber = 0.0/0.0;
//     P_.setIdentity( 7, 7 );
    P_.setIdentity( 4, 4 );
    Pdim_.setIdentity( 1, 1 ); 
    this->setProperties( notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber  ); // Produces NaN values, meaning that the properties are not initialized yet
    xVel_   = 0.0;
    yVel_   = 0.0;
}

void Circle::setProperties ( float x, float y, float z, float roll, float pitch, float yaw, float radius )
{
    x_ = x;
    y_ = y;
    z_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    radius_ = radius;
}

void Circle::printProperties ( )
{
    std::cout << "Circle prop : " ;
    std::cout << "x_ = " << x_;
    std::cout << " y_ = " << y_;
    std::cout << " xVel_ = " << xVel_;
    std::cout << " yVel_ = " << yVel_;
    std::cout << " roll_ = " << roll_;
    std::cout << " pitch_ = " << pitch_;
    std::cout << " yaw_ = " << yaw_;
    std::cout << " radius = " << radius_ ;
    std::cout << " P_ = " << P_;
//     std::cout << "Pdim_ = " << Pdim_ << std::endl;
}

void Circle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
   std_msgs::ColorRGBA color;
   color.a = 0.5;
   color.r = 0.0;
   color.g = 1.0;
   color.b = 0.0;
   
   this->setMarker ( marker, ID, color ); 
}

void Circle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Position Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll_, pitch_, yaw_ );
    marker.scale.x = 2*radius_;
    marker.scale.y = 2*radius_;
    marker.scale.z = 0.1;
    
    marker.color = color;

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void Circle::setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Translational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;

    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    float rollVel = 0.0;
    float pitchVel = 0.0;
    float yawVel = atan2( yVel_, xVel_ );
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollVel, pitchVel, yawVel );

    marker.scale.x = sqrt( pow(xVel_, 2.0) + pow(yVel_, 2.0) );
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

std::vector< geo::Vec2f > Circle::convexHullPoints(unsigned int nPoints)
{
  std::vector< geo::Vec2f > Points(nPoints);
  float deltaAngle = 2*M_PIl / nPoints;
  
  for(unsigned int ii = 0; ii < nPoints; ii++)
  {
    float angle = ii*deltaAngle;
    Points[ii].x = x_ + radius_*cos(angle);
    Points[ii].y = y_ + radius_*sin(angle);
  }
  
  return Points;
}

float Circle::predictX( float dt )
{
        return x_ + dt * xVel_;
}

float Circle::predictY( float dt )
{
        return y_ + dt * yVel_;
}

void Circle::predictPos( float* predictedX, float* predictedY, float dt )
{
        *predictedX = predictX( dt );
        *predictedY = predictY( dt );
}

void Circle::predictAndUpdatePos( float dt )
{
        x_ = predictX( dt );
        y_ = predictY( dt );
}


float fitRectangle ( std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, const geo::Pose3D& pose , unsigned int cornerIndex, unsigned int minPointsLine )
{
//std::cout << "fitRectangleFuntion "<< std::endl;
    std::vector<geo::Vec2f>::iterator it_start = points.begin();
    std::vector<geo::Vec2f>::iterator it_splitLow = points.begin() + cornerIndex - 1; // +1 and -1 to prevent problems when a corner is not visible due to an occlusion
    std::vector<geo::Vec2f>::iterator it_splitHigh = points.begin() + cornerIndex + 1;
    std::vector<geo::Vec2f>::iterator it_end = points.end();

    Eigen::VectorXf beta_hat1 ( 2 ), beta_hat2 ( 2 );
    bool validFit1, validFit2;

    float mean_error1 = fitLine ( points, beta_hat1, &it_start, &it_splitLow ); //
    float mean_error2 = fitLine ( points, beta_hat2, &it_splitHigh, &it_end );

    unsigned int nPointsForExtrema;
    minPointsLine % 2 == 0 ? nPointsForExtrema = minPointsLine / 2 : nPointsForExtrema = (minPointsLine - 1) / 2;

    geo::Vec2f pointLow, pointHigh;
    pointLow.x = 0.0;
    pointLow.y = 0.0;
    pointHigh.x = 0.0;
    pointHigh.y = 0.0;

    for (unsigned int iAverage = 0; iAverage < nPointsForExtrema; iAverage++) // averaging to be more robust for noise of extreme points
    {
            geo::Vec2f point = points[iAverage];
            pointLow += point;

            point = points[cornerIndex - 1 - iAverage];
            pointHigh += point;
    }

    pointLow /= nPointsForExtrema;
    pointHigh /= nPointsForExtrema;
    
    float x_start1 = pointLow.x; // Is this correct in combination with theta?
    float y_start1 = pointLow.y;

    // determine width and height
    float x_end = pointHigh.x;
    float y_end = pointHigh.y;
    
    float theta = atan2 ( beta_hat1 ( 1 ), 1 ); // TODO: angle on points low alone?
    float theta2 = atan2 ( beta_hat2 ( 1 ), 1 ) + M_PI_2;
    
    if(theta == theta)
    {
            unwrap( &theta2, theta, (float) M_PI );
    }

    float diff = std::fabs(theta2  - theta );
    
    if(diff < M_PI / 8 )
    {
        theta = 0.5*(theta + theta2); // take the average
    }
    else
    {
        // large difference, so rely on the part which has the most points
        if( points.size() - cornerIndex > 0.5*points.size() && theta2 == theta2 || theta != theta && theta2 == theta2)
        {
            theta = theta2;
        }        
    }
    
    pointLow.x = 0.0;
    pointLow.y = 0.0;
    pointHigh.x = 0.0;
    pointHigh.y = 0.0;

    for (unsigned int iAverage = 0; iAverage < nPointsForExtrema; iAverage++) // averaging to be more robust for noise of extreme points
    {
        geo::Vec2f point = points[cornerIndex + 1 + iAverage];
        pointLow += point;

        point = points[points.size() - 1 - iAverage];
        pointHigh += point;
    }

    pointLow /= nPointsForExtrema;
    pointHigh /= nPointsForExtrema;

    float x_start2 = pointLow.x; // Is this correct in combination with theta?
    float y_start2 = pointLow.y;

    // determine width and height
    float x_end2 = pointHigh.x;
    float y_end2 = pointHigh.y;
   
    float dx = x_end2 - x_start2;
    float dy = y_start2 - y_end2;
    float depth = sqrt ( dx*dx+dy*dy ); // minimal depth
    
    // As we might have partial detection in both width and depth direction, ensure that the dimension and positions are corrected according to this
    float centerWidth_x = 0.5* ( x_start1 + x_end );
    float centerWidth_y = 0.5* ( y_start1 + y_end );
    
    float centerDepth_x = 0.5* ( x_end2 + x_start2 );
    float centerDepth_y = 0.5* ( y_end2 + y_start2  );    
    
    geo::Vec2f p1Line, p2Line, point2Project;
    float ct = cos ( theta );
    float st = sin ( theta );
    
    p1Line.x = centerDepth_x - 0.5*depth*st;
    p1Line.y = centerDepth_y + 0.5*depth*ct;
    
    p2Line.x = centerDepth_x -(- 0.5*depth*st);
    p2Line.y = centerDepth_y -(+ 0.5*depth*ct);
    
    point2Project.x = centerWidth_x;
    point2Project.y = centerWidth_y;
    
    geo::Vec2f cornerPoint = projectPointOnLine( p1Line, p2Line, point2Project);

    float width = 2*( sqrt ( pow( centerWidth_x - cornerPoint.x, 2.0) + pow( centerWidth_y - cornerPoint.y, 2.0) ) ); 
    depth = 2*( sqrt ( pow( centerDepth_x - cornerPoint.x, 2.0) + pow( centerDepth_y - cornerPoint.y, 2.0) ) );
    
    float center_x = 0.5* ( x_start1 + x_end ) + 0.5* ( x_end2 - x_start2 ); // uncorrected
    float center_y = 0.5* ( y_start1 + y_end ) + 0.5* ( y_end2 - y_start2 );
    
    float center_x_correctedPos = centerDepth_x + 0.5*width*ct;
    float center_y_correctedPos = centerDepth_y + 0.5*width*st;
    
    float center_x_correctedNeg = centerDepth_x - 0.5*width*ct;
    float center_y_correctedNeg = centerDepth_y - 0.5*width*st;
    
    float dist2Pos = pow( center_x_correctedPos - center_x, 2.0) + pow( center_y_correctedPos - center_y, 2.0);
    float dist2Neg = pow( center_x_correctedNeg - center_x, 2.0) + pow( center_y_correctedNeg - center_y, 2.0);
    
    if( dist2Pos < dist2Neg )
    {
            center_x = center_x_correctedPos;
            center_y = center_y_correctedPos;
    }
    else
    {
            center_x = center_x_correctedNeg;
            center_y = center_y_correctedNeg;
    }
    
    float roll = 0.0, pitch = 0.0, yaw = theta;
    rectangle->setValues ( center_x, center_y, pose.getOrigin().getZ(), width, depth, ARBITRARY_HEIGHT, roll, pitch, yaw ); // Assumption: object-height identical to sensor-height

    unsigned int low_size = cornerIndex;
    unsigned int high_size = points.size() - cornerIndex + 1;
    
    return ( (mean_error1*low_size+mean_error2*high_size ) / ( low_size + high_size ) ); // weighted average of error
}

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *IDs, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end, float minDistCornerDetection, unsigned int minPointsLine )
{
//std::cout << "FindPossibleCorner " << std::endl;
    float maxDistance = 0.0;
    unsigned int ID = std::numeric_limits<unsigned int>::quiet_NaN();

    geo::Vec2f startPoint = **it_start;
    geo::Vec2f endPoint = * ( *it_end - 1 );

    unsigned int nPointsForExtrema;
    minPointsLine % 2 == 0 ? nPointsForExtrema = minPointsLine / 2 : nPointsForExtrema = (minPointsLine - 1) / 2;

    geo::Vec2f pointLow, pointHigh;
    pointLow.x = 0.0;
    pointLow.y = 0.0;
    pointHigh.x = 0.0;
    pointHigh.y = 0.0;

    for (unsigned int iAverage = 0; iAverage < nPointsForExtrema; iAverage++) // averaging to be more robust for noise of extreme points
    {
            geo::Vec2f point = *(*it_start + iAverage);
            pointLow += point;
            
            point = *(*it_end - 1 - iAverage);
            pointHigh += point;
    }

    pointLow /= nPointsForExtrema;
    pointHigh /= nPointsForExtrema;
    
    float length = sqrt ( pow ( pointHigh.y - pointLow.y,2.0 ) + pow ( pointHigh.x - pointLow.x,2.0 ) );    
    
    for ( std::vector<geo::Vec2f>::iterator it = *it_start + 1; it != *it_end - 1; ++it ) 
    {
        geo::Vec2f point = *it;
        
        float x0 = point.x;
        float y0 = point.y;
        
        float distance = fabs ( (pointHigh.y - pointLow.y)*x0 - (pointHigh.x - pointLow.x)*y0 + pointHigh.x*pointLow.y - pointHigh.y*pointLow.x ) / length;

        if ( distance > maxDistance ) 
        {
            maxDistance = distance;
            ID = std::distance ( points.begin(), it );
        }
    }
    
    if ( maxDistance >  minDistCornerDetection ) 
    {
        IDs->push_back ( ID );
        return true;
    } 
    else 
    {
        return false;
    }

}

bool findPossibleCorners ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *cornerIndices, float minDistCornerDetection, unsigned int minPointsLine )
{
    // Check in section if a corner is present. If that is the case, split the data at this corner, and check for both parts if another corner is present.
    std::vector<laserSegments> segments;

    std::vector<geo::Vec2f>::iterator it_start = points.begin();
    std::vector<geo::Vec2f>::iterator it_end = points.end();

    if ( findPossibleCorner ( points, cornerIndices, &it_start, &it_end, minDistCornerDetection, minPointsLine ) )
    { // -1 because std::vector::end returns an iterators to one-past-the-end of the container. The element just before is then the last element in the vector.
        laserSegments segmentToAdd;
        segmentToAdd.begin = points.begin();
        segmentToAdd.end = points.begin() + cornerIndices->back();

        if ( segmentToAdd.end - segmentToAdd.begin > minPointsLine ) 
        {
            segments.push_back ( segmentToAdd );
        }

        segmentToAdd.begin = points.begin() + cornerIndices->back();
        segmentToAdd.end = points.end() - 1;

        if ( segmentToAdd.end - segmentToAdd.begin > minPointsLine ) 
        {
            segments.push_back ( segmentToAdd );
        }

        for ( unsigned int ii = 0; ii < segments.size(); ++ii ) 
        {
            laserSegments laserSegment = segments[ii];
            geo::Vec2f pointEnd =  *laserSegment.end;

            bool test = findPossibleCorner ( points, cornerIndices, &laserSegment.begin, &laserSegment.end, minDistCornerDetection , minPointsLine);

            if ( test ) 
            {
                segmentToAdd.begin = laserSegment.begin;
                segmentToAdd.end = points.begin() + cornerIndices->back();

                if ( segmentToAdd.end - segmentToAdd.begin > minPointsLine ) 
                {
                    segments.push_back ( segmentToAdd );
                }

                segmentToAdd.begin = points.begin() + cornerIndices->back();
                segmentToAdd.end = laserSegment.end;

                if ( segmentToAdd.end - segmentToAdd.begin > minPointsLine ) 
                {
                    segments.push_back ( segmentToAdd );
                }
            }
        }

        std::sort ( cornerIndices->begin(), cornerIndices->end(), greater() );
        
        return true;
    } 
    else 
    {
        return false;
    }
}

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end )
{
     float mean_errorOriginal = fitLineLeastSq ( points, beta_hat, it_start, it_end ) ;   
     
     std::vector<geo::Vec2f> pointsTranspose;
     for(std::vector<geo::Vec2f>::iterator it = *it_start; it != *it_end; ++it)
     {
        geo::Vec2f point = *it;
        geo::Vec2f pointTranspose;
        
        pointTranspose.x = point.y;
        pointTranspose.y = point.x;
        
        pointsTranspose.push_back(pointTranspose);
     }
     
     Eigen::VectorXf beta_hatTranspose(2);   
     
    std::vector<geo::Vec2f>::iterator it_startTranspose = pointsTranspose.begin();
    std::vector<geo::Vec2f>::iterator it_endTranspose = pointsTranspose.end();

     float mean_errorTranspose = fitLineLeastSq ( pointsTranspose, beta_hatTranspose, &it_startTranspose, &it_endTranspose ) ; 
     
     if(mean_errorTranspose == mean_errorTranspose && mean_errorTranspose < mean_errorOriginal)
     {
             beta_hat(0) = -beta_hatTranspose(0) / beta_hatTranspose(1); //rewrite x = b+a*y -> y = 1/a*x - b/a
             beta_hat(1) = 1.0 / beta_hatTranspose(1);
             
             return mean_errorTranspose;
     }
     else if(mean_errorOriginal == mean_errorOriginal )
     {
             return mean_errorOriginal;
     }
     else
     {
             return std::numeric_limits< float >::infinity();
     }     
}

float fitLineLeastSq ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end )
{
    // Least squares method: http://home.isr.uc.pt/~cpremebida/files_cp/Segmentation%20and%20Geometric%20Primitives%20Extraction%20from%202D%20Laser%20Range%20Data%20for%20Mobile%20Robot%20Applications.pdf
    unsigned int size = std::distance ( *it_start, *it_end );
    Eigen::MatrixXf m ( size, 2 );
    Eigen::MatrixXf mtest ( size, 2 );
    Eigen::VectorXf y ( size );
    Eigen::VectorXf ytest ( size );
    unsigned int counter = 0;
    
    std::vector<geo::Vec2f>::iterator it = *it_start;
    geo::Vec2f point_start = *it;
    it = *it_end; it--;
    geo::Vec2f point_end = *it;

    for ( std::vector<geo::Vec2f>::iterator it = *it_start; it != *it_end; ++it ) 
    {
        geo::Vec2f point = *it;
        m ( counter, 0 ) = ( double ) 1.0;
        m ( counter, 1 ) = ( double ) point.x;

        y ( counter ) = ( double ) point.y;
        ytest ( counter ) = ( double ) point.y;
        counter++;
    }

    Eigen::MatrixXf mt ( size, 2 );
    mt = m.transpose();

    beta_hat = ( mt*m ).inverse() * mt * y;// Numerically not stable!!!!
    float error, sum = 0.0;

    counter = 0;
    for ( std::vector<geo::Vec2f>::iterator it = *it_start; it != *it_end; ++it ) 
    {
        // Distance of each point to line
        geo::Vec2f point = *it;
        error = fabs ( -beta_hat ( 1 ) * point.x+point.y - beta_hat ( 0 ) ) /sqrt ( beta_hat ( 1 ) *beta_hat ( 1 ) + 1 ); // distance of a point to a line, see https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        float absError = sqrt( pow( error, 2.0 ) );
        sum += absError;
        counter ++;
    }

    return sum/counter;
}

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, ed::tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose, unsigned int minPointsLine )
{
    Eigen::VectorXf beta_hat ( 2 );
    float averageError = fitLine ( points, beta_hat, it_low, it_high ) ;

    float theta = atan2 ( beta_hat ( 1 ), 1 );

    unsigned int ii_start = std::distance ( points.begin(), *it_low );
    unsigned int ii_end = std::distance ( points.begin(), *it_high ) - 1;

    unsigned int nPointsForExtrema;
    minPointsLine % 2 == 0 ? nPointsForExtrema = minPointsLine / 2 : nPointsForExtrema = (minPointsLine - 1) / 2;
    
    geo::Vec2f pointLow, pointHigh;
    pointLow.x = 0.0;
    pointLow.y = 0.0;
    pointHigh.x = 0.0;
    pointHigh.y = 0.0;

    for (unsigned int iAverage = 0; iAverage < nPointsForExtrema; iAverage++) // averaging to be more robust for noise of extreme points
    {

    geo::Vec2f point = points[ii_start + iAverage];
    pointLow += point;

    point = points[ii_end - iAverage];
    pointHigh += point;
   }

    pointLow /= nPointsForExtrema;
    pointHigh /= nPointsForExtrema;
    
    float x_start = pointLow.x;
    float y_start = pointLow.y; 

    float x_end = pointHigh.x; 
    float y_end = pointHigh.y;

    float dx = x_end - x_start;
    float dy = y_start - y_end;
    float width = sqrt ( dx*dx+dy*dy );

    float center_x = 0.5* ( x_start + x_end );
    float center_y = 0.5* ( y_start + y_end );

    float roll = 0.0, pitch = 0.0, yaw = theta;
    
    rectangle->setValues ( center_x, center_y, sensor_pose.getOrigin().getZ(), width, ARBITRARY_DEPTH, ARBITRARY_HEIGHT, roll, pitch, yaw ); // Assumption: object-height identical to sensor-height

    return averageError;
}

Rectangle::Rectangle()
{
    float notANumber = 0.0/0.0;
    P_.setIdentity( 6, 6 ); 
    Pdim_.setIdentity( 2, 2 ); 
    this->setValues( notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber ); // Produces NaN values, meaning that the properties are not initialized yet
    xVel_   = 0.0;
    yVel_   = 0.0;
    yawVel_ = 0.0;
}

void Rectangle::setValues ( float x, float y, float z, float w, float d, float h, float roll, float pitch, float yaw )
{
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
    d_ = d;
    h_ = h;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

void Rectangle::printProperties ( )
{
    std::cout << "Rect prop = " ;
    std::cout << "x_ = "      << x_;
    std::cout << " y_ = "     << y_;
    std::cout << " z_ = "     << z_;
    std::cout << " w_ = "     << w_;
    std::cout << " d_ = "     << d_;
    std::cout << " h_ = "     << h_;
    std::cout << " xVel_ = "  << xVel_;
    std::cout << " yVel_ = "  << yVel_;
    std::cout << " yawVel_ = "<< yawVel_;
    std::cout << " roll_ = "  << roll_;
    std::cout << " pitch_ = " << pitch_;
    std::cout << " yaw_ = "   << yaw_;
}

float Rectangle::predictX( float dt )
{
        return x_ + dt * xVel_;
}

float Rectangle::predictY( float dt )
{
        return y_ + dt * yVel_;
}

float Rectangle::predictYaw( float dt )
{
        return yaw_ + dt * yawVel_;
}

void Rectangle::predictPos( float* predictedX, float* predictedY, float* predictedYaw, float dt )
{
        *predictedX = predictX( dt );
        *predictedY = predictY( dt );
        *predictedYaw = predictYaw( dt );
}

void Rectangle::predictAndUpdatePos( float dt )
{
        x_ = predictX( dt );
        y_ = predictY( dt );
        yaw_ = predictYaw( dt );
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color, std::string ns )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll_, pitch_, yaw_ );
    marker.scale.x = w_;
    marker.scale.y = d_;
    marker.scale.z = 0.1;
    marker.color = color;
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
   std_msgs::ColorRGBA color;
   color.a = 0.5;
   color.r = 0.0;
   color.g = 1.0;
   color.b = 0.0;
   
   this->setMarker ( marker, ID, color ); 
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color)
{
        this->setMarker ( marker, ID, color, "Position Marker" ); 
}

void Rectangle::setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Translational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    
    float rollTranslationalVel = 0.0;
    float pitchTranslationalVel = 0.0;
    float yawTranslationalVel = std::atan2( yVel_, xVel_ );
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollTranslationalVel, pitchTranslationalVel, yawTranslationalVel );

   // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
   // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    marker.scale.x = std::sqrt( std::pow(xVel_, 2.0) + std::pow(yVel_, 2.0) );
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void Rectangle::setRotationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    // At the first corner, place an indicator about the rotational vel
    std::vector<geo::Vec2f> corners = determineCorners( 0.0 );
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Rotational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = corners[0].x;
    marker.pose.position.y = corners[0].y;
    marker.pose.position.z = z_;

    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    float rollRotVel = 0.0;
    float pitchRotVel = 0.0;
    float yawRotVel = atan2(d_, w_) - M_PI_2;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollRotVel, pitchRotVel, yaw_ + yawRotVel );
    
    marker.scale.x = ( pow(w_, 2.0) + pow(d_, 2.0) )*yawVel_; // Velocity of the cornerpoint, so scaled with the distance from the center.
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;  
    
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

std::vector<geo::Vec2f> Rectangle::determineCorners ( float associationDistance )
{
    float dx = 0.5* ( w_ + associationDistance ); // blow up for associations
    float dy = 0.5* ( d_ + associationDistance );

    float ct = cos ( yaw_ );
    float st = sin ( yaw_ );

    geo::Vec2f originCorner ( x_ + ct*-dx + st* dy, y_ + st*-dx + ct*-dy ); // Rotation matrix @ -x, -y
    geo::Vec2f corner1 (      x_ + ct* dx + st* dy, y_ + st* dx + ct*-dy ); // @ +x, -y
    geo::Vec2f corner2 (      x_ + ct* dx - st* dy, y_ + st* dx + ct* dy ); // @ +x, +y
    geo::Vec2f corner3 (      x_ + ct*-dx - st* dy, y_ + st*-dx + ct* dy ); // @ -x, +y

    std::vector< geo::Vec2f > corners;
    corners.push_back ( originCorner );
    corners.push_back ( corner1 );
    corners.push_back ( corner2 );
    corners.push_back ( corner3 );

    return corners;
}

std::vector<geo::Vec2f> Rectangle::determineCenterpointsOfEdges ( )
{
        float rotWidth = yaw_ + M_PI_4;
        float ct = cos ( rotWidth );
        float st = sin ( rotWidth );
        
        float length = 0.5*d_;
        
        geo::Vec2f originCorner ( x_ + ct* length + st* 0, y_ + st* length + ct*0 );
        geo::Vec2f corner1 (      x_ + ct*-length + st* 0, y_ + st*-length + ct*0 );
        
        length = 0.5*w_;
        geo::Vec2f corner2 (      x_ + ct* 0 - st* length, y_ + st* 0 + ct* length );
        geo::Vec2f corner3 (      x_ + ct* 0 - st*-length, y_ + st* 0 + ct* -length );

        std::vector< geo::Vec2f > corners;
        corners.push_back ( originCorner );
        corners.push_back ( corner1 );
        corners.push_back ( corner2 );
        corners.push_back ( corner3 );

        return corners;  
}

bool Rectangle::switchDimensions( float measuredYaw)
{
        return std::fabs( std::fabs( yaw_ - measuredYaw )- M_PI_2 ) < MARGIN_RECTANGLE_INTERCHANGE;
}
void Rectangle::interchangeRectangleFeatures()
{
        float widthOld = w_;
        w_ = d_;
        d_ = widthOld;
        
       yaw_ += M_PI_2;
       
       float P_depthOld = Pdim_ ( 1, 1 );
       Pdim_( 1, 1) = Pdim_( 0, 0);
       Pdim_( 0, 0) = P_depthOld;
}

Eigen::VectorXf Rectangle::setState(float posX, float posY, float posYaw, float xVel, float yVel, float yawVel, float width, float depth)
{
         Eigen::MatrixXf state( 8, 1 );
         state << posX, posY, posYaw, xVel, yVel, yawVel, width, depth;
         
         return state;   
}

bool FeatureProbabilities::setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleDiameter, float typicalCorridorWidth )
{
        // TODO improve
    if ( !std::isinf ( errorRectangleSquared ) || !std::isinf ( errorCircleSquared ) )
    {
        float probabilityScaling = 1.0;
        if ( circleDiameter > 0.5*typicalCorridorWidth )
        {
            // Circles with a very large radius could have a smaller error compared to fitting a line. For the type of environment, this is very unlikely.
            // Therefore, the probability of large circles (diameter > typicalCorridorWidth) is reduced in an exponential fashion
            probabilityScaling = std::exp ( -1 / ( 0.5*typicalCorridorWidth ) * ( circleDiameter -0.5*typicalCorridorWidth ) );
        }

        float sum = errorRectangleSquared + errorCircleSquared;
        float pCircle = probabilityScaling * errorRectangleSquared/sum;
	if(pCircle < MIN_PROB_OBJECT) // smooth out prob such that recovery is easier
	{
		pCircle = MIN_PROB_OBJECT;
	}
	else if(pCircle > 1.0 - MIN_PROB_OBJECT)
	{
		pCircle = 1.0 - MIN_PROB_OBJECT;
	}


        float pRectangle =  1.0 - pCircle;  // Only 2 objects now, so the sum of it equals 1

        pmf_.setProbability ( "Rectangle", pRectangle );
        pmf_.setProbability ( "Circle", pCircle );
        return true;
    }
    else
    {
        // Infinity detected on both of the elements: this is the case when the number of points is too small to do a fit. Equal probability set.
        pmf_.setProbability ( "Rectangle", 0.5 );
        pmf_.setProbability ( "Circle", 0.5 );
    
            // TODO if there are enough points for a single fit (probably circle only), is this fit realistic?
            // Acatually, it should be measured if the object which is modelled is realistic by comparing the laser scan with the expected scan based on that object
            
            return false;
    }
}

void FeatureProbabilities::update ( float pRectangle_measured, float pCircle_measured )
{
    pbl::PMF pmf_measured = pmf_;

    pmf_measured.setProbability ( "Rectangle", pRectangle_measured );
    pmf_measured.setProbability ( "Circle", pCircle_measured );

    pmf_.update ( pmf_measured );
    
    float pCircle = pmf_.getProbability ( "Circle" );            
    
    if(pCircle < MIN_PROB_OBJECT) // smooth out prob such that recovery is easier
    {
            pCircle = MIN_PROB_OBJECT;
    }
    else if(pCircle > 1.0 - MIN_PROB_OBJECT)
    {
            pCircle = 1.0 - MIN_PROB_OBJECT;
    }

     float pRectangle =  1.0 - pCircle;  // Only 2 objects now, so the sum of it equals 1

     pmf_.setProbability ( "Rectangle", pRectangle );
     pmf_.setProbability ( "Circle", pCircle );
}

void FeatureProbabilities::update ( FeatureProbabilities& featureProbabilities_in )
{
    this->pmf_.update ( featureProbabilities_in.pmf_ );
}

void FeatureProperties::updateCircleFeatures ( Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::MatrixXf z_k, float dt )
// z = observation, dt is the time difference between the latest update and the new measurement
{
        unsigned int x_PosVelRef = 0, y_PosVelRef = 1, xVel_PosVelRef = 2, yVel_PosVelRef = 3;//, xAccel_PosVelRef = 4, yAccel_PosVelRef = 5;
        unsigned int r_dimRef = 0;      
        unsigned int x_zRef = 0, y_zRef = 1, radius_zRef = 2;
        
        Eigen::MatrixXf F_PosVel ( 4, 4 );
        F_PosVel << 1.0, 0.0, dt,  0.0,  // x 
                    0.0, 1.0, 0.0, dt,   // y 
                    0.0, 0.0, 1.0, 0.0,  // x vel 
                    0.0, 0.0, 0.0, 1.0;  // y vel

        float dt2 = std::pow(dt, 2.0);
        
/*        Eigen::MatrixXf F_PosVel ( 6, 6 );        
        F_PosVel << 1.0, 0.0, dt,  0.0, 0.5*dt2, 0.0,   // x 
                    0.0, 1.0, 0.0, dt,  0.0,     0.5*dt2,   // y 
                    0.0, 0.0, 1.0, 0.0, dt,      0.0,       // x vel 
                    0.0, 0.0, 0.0, 1.0, 0.0,     dt,       // y vel
                    0.0, 0.0, 0.0, 0.0, 1.0,     0.0,       // x double
                    0.0, 0.0, 0.0, 0.0, 0.0,     1.0;      // y double   */             
                    
        Eigen::MatrixXf Fdim ( 1, 1 );    
        Fdim <<     1.0;               // radius
                
//         Eigen::MatrixXf H_PosVel ( 2, 6 );
//         H_PosVel << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                     0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
                    
        Eigen::MatrixXf H_PosVel ( 2, 4 );
        H_PosVel << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0;
         
        Eigen::MatrixXf Hdim ( 1, 1 );
        Hdim.setIdentity( Hdim.rows(), Hdim.cols() );
        
        // First, update the dimensions
        Eigen::MatrixXf x_k_1_k_1_dim ( 1, 1 ), z_k_dim( 1, 1 );
        x_k_1_k_1_dim << circle_.get_radius();
        z_k_dim << z_k( radius_zRef );
        Eigen::MatrixXf Pdim = circle_.get_Pdim();
        
        Eigen::MatrixXf x_k_k_dim = kalmanUpdate(Fdim, Hdim, &Pdim, x_k_1_k_1_dim, z_k_dim, Q_k.block<1, 1>( 4, 4 ), R_k.block<1, 1>( 2, 2 ) );
        
        // After the position update for changed dimensions, update the dimensions
        Eigen::MatrixXf P_PosVel = circle_.get_P();
        Eigen::MatrixXf x_k_1_k_1_PosVel( 4, 1 ), z_k_posVel( 2, 1 );
        x_k_1_k_1_PosVel << circle_.get_x(), circle_.get_y(), circle_.get_xVel(), circle_.get_yVel();//, circle_.get_xAccel(), circle_.get_yAccel();
        z_k_posVel << z_k ( x_zRef ), z_k ( y_zRef );
        Eigen::MatrixXf x_k_k_PosVel = kalmanUpdate(F_PosVel, H_PosVel, &P_PosVel, x_k_1_k_1_PosVel, z_k_posVel, Q_k.block<4, 4>( 0, 0 ), R_k.block<2, 2>( 0, 0 ) );         
 
        circle_.set_x ( x_k_k_PosVel ( x_PosVelRef ) );
        circle_.set_y ( x_k_k_PosVel ( y_PosVelRef ) );
        circle_.set_xVel ( x_k_k_PosVel ( xVel_PosVelRef ) );
        circle_.set_yVel ( x_k_k_PosVel ( yVel_PosVelRef ) );
  //    circle_.set_xAccel ( x_k_k_PosVel ( xAccel_PosVelRef ) );
  //    circle_.set_yAccel ( x_k_k_PosVel ( yAccel_PosVelRef ) );
        circle_.set_radius ( x_k_k_dim( r_dimRef ) );
        circle_.set_P ( P_PosVel );
        circle_.set_Pdim ( Pdim );
}

void FeatureProperties::updateRectangleFeatures ( Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::VectorXf z_k, float dt, const geo::Pose3D& sensor_pose )
{
        // z = observation, dt is the time difference between the latest update and the new measurement
        // 2 stages: first determine the updated width en depth, then use this difference to update the position first in order to prevent ghost-velocities. 
        
        // conversion for general state to state for (1) the position and velocity state and (2) the dimension state
        unsigned int x_PosVelRef = 0, y_PosVelRef = 1, yaw_PosVelRef = 2, xVel_PosVelRef = 3, yVel_PosVelRef = 4, yawVel_PosVelRef = 5;
        unsigned int width_dimRef = 0, depth_dimRef = 1;
        unsigned int x_zRef = 0, y_zRef = 1, yaw_zRef = 2, width_zRef = 3, depth_zRef = 4;
        
        Eigen::MatrixXf F_PosVel ( 6, 6 );
        F_PosVel << 1.0, 0.0, 0.0, dt,  0.0, 0.0, // x 
                    0.0, 1.0, 0.0, 0.0, dt,  0.0, // y 
                    0.0, 0.0, 1.0, 0.0, 0.0, dt,  // orientation
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // x vel 
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // y vel 
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // rotational vel
                
        Eigen::MatrixXf Fdim ( 2, 2 );    
        Fdim <<     1.0, 0.0,               // width
                    0.0, 1.0;               // length
                
        Eigen::MatrixXf H_PosVel ( 3, 6 );
        H_PosVel << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
         
        Eigen::MatrixXf Hdim ( 2, 2 );
        Hdim.setIdentity( Hdim.rows(), Hdim.cols() ); 
                
        unwrap( &z_k( yaw_zRef ), rectangle_.get_yaw(), (float) M_PI );

        if( rectangle_.switchDimensions( z_k( yaw_zRef ) ) )
        {
                rectangle_.interchangeRectangleFeatures( );
                unwrap( &z_k( yaw_zRef ), rectangle_.get_yaw(), (float) M_PI ); 
                
                //ROS_WARN("Interchanged rectangular features");
        }
        
        Eigen::MatrixXf Pdim = rectangle_.get_Pdim();
        Eigen::MatrixXf x_k_1_k_1_dim( 2, 1 ), z_k_dim( 2, 1 );
        x_k_1_k_1_dim << rectangle_.get_w(), rectangle_.get_d();
        z_k_dim << z_k( width_zRef ), z_k( depth_zRef );
        
        // dim update
        Eigen::MatrixXf x_k_k_dim = kalmanUpdate(Fdim, Hdim, &Pdim, x_k_1_k_1_dim, z_k_dim, Q_k.block<2, 2>( 6, 6 ), R_k.block<2, 2>( 3, 3 ) );
        
        // Correct position for changes caused by a change in dimension
//         float deltaWidth = x_k_k_dim( width_dimRef ) - x_k_1_k_1_dim ( width_dimRef );
//         float deltaDepth = x_k_k_dim( depth_dimRef ) - x_k_1_k_1_dim ( depth_dimRef );
        
        float deltaWidth = x_k_1_k_1_dim ( width_dimRef ) - x_k_k_dim( width_dimRef );
        float deltaDepth = x_k_1_k_1_dim ( depth_dimRef ) - x_k_k_dim( depth_dimRef );
        
//        std::cout << "deltaWidth, deltaDepth model = " << deltaWidth << ", " << deltaDepth << std::endl;
        
      /* 
        float deltaX_dim, deltaY_dim;
        correctPosForDimDiff(deltaWidth, deltaDepth, &deltaX_dim, &deltaY_dim, dt );
        
        // What would the previous position of the measurement be given the velocities already estimated?
        float ZPrevX = z_k( x_zRef ) - rectangle_.get_xVel()*dt;
        float ZPrevY = z_k( y_zRef ) - rectangle_.get_yVel()*dt;
        
        // Determine the direction of the correction based on an estimation of the position of the measurement at the previous timestamp
        int signX = ( std::fabs( rectangle_.get_x() + deltaX_dim - ZPrevX ) < std::fabs( rectangle_.get_x() - deltaX_dim - ZPrevX ) ) ? 1 : -1;
        int signY = ( std::fabs( rectangle_.get_y() + deltaY_dim - ZPrevY ) < std::fabs( rectangle_.get_y() - deltaY_dim - ZPrevY ) ) ? 1 : -1;
        rectangle_.set_x ( rectangle_.get_x() + deltaX_dim );
        rectangle_.set_y ( rectangle_.get_y() + deltaY_dim );
       */
        
       float deltaX = 0.0, deltaY = 0.0;
       correctForDimensions( deltaWidth, deltaDepth, &deltaX, &deltaY, z_k( x_zRef ), z_k( y_zRef ), rectangle_.get_x(), rectangle_.get_y(), dt );
//        std::cout << "Delta x meas dim = " << deltaWidth << " Delta y dim = " << deltaDepth << " deltaX, Y = " << deltaX << ", " << deltaY << std::endl;
      
//        std::cout << "Rect pos before first update = " << rectangle_.get_x() << ", " << rectangle_.get_y() << std::endl;
//        std::cout << "Delta x dim = " << deltaX << "Delta y dim = " << deltaY << std::endl;
        
       // Correct position due to updated pos
        rectangle_.set_x ( rectangle_.get_x() + deltaX );
        rectangle_.set_y ( rectangle_.get_y() + deltaY );
  //      std::cout << "Rect pos after first update = " << rectangle_.get_x() << ", " << rectangle_.get_y() << std::endl;
  //      std::cout << "Rect dim before first update = " << rectangle_.get_w() << ", " << rectangle_.get_d() << std::endl;
        
        rectangle_.set_w ( x_k_k_dim ( width_dimRef ) );
        rectangle_.set_d ( x_k_k_dim ( depth_dimRef ) );
        
   //     std::cout << "Rect dim after first update = " << rectangle_.get_w() << ", " << rectangle_.get_d() << std::endl;
        
        // Correct measured position caused by differences in modelled and measured dimensions
        deltaWidth = z_k ( width_zRef ) - rectangle_.get_w();
        deltaDepth = z_k ( depth_zRef ) - rectangle_.get_d();
        
        // Correct measured pos due to diff in model & measurement
//        std::cout << "z_k( x_zRef ), z_k( y_zRef ) before = " << z_k( x_zRef ) << ", " << z_k( y_zRef );// << std::endl;
//        std::cout << " deltaWidth, deltaDepth = " << deltaWidth << ", " << deltaDepth << std::endl;
        
        deltaX = 0.0; deltaY = 0.0;
        correctForDimensions( deltaWidth, deltaDepth, &deltaX, &deltaY, z_k( x_zRef ), z_k( y_zRef ), rectangle_.get_x(), rectangle_.get_y(), dt );
//         std::cout << "Delta x meas dim = " << deltaWidth << " Delta y dim = " << deltaDepth << " deltaX, Y = " << deltaX << ", " << deltaY << std::endl;
 //       std::cout << "Out of func: deltaX, deltaY = " << deltaX << ", " << deltaY << std::endl;
        
        z_k( x_zRef ) = z_k( x_zRef ) - deltaX;
        z_k( y_zRef ) = z_k( y_zRef ) - deltaY;
 //       std::cout << "z_k( x_zRef ), z_k( y_zRef ) after = " << z_k( x_zRef ) << ", " << z_k( y_zRef ) << std::endl;
        
        
//         std::cout << "signWidth = " << signWidth << " signWidth = " << signDepth << std::endl;
//         std::cout << "deltaX_VelWidth = " << deltaX_VelWidth << " deltaX_VelWidth = " << deltaX_VelWidth << std::endl;
//         std::cout << "deltaX_VelDepth = " << deltaX_VelDepth << " deltaY_VelDepth = " << deltaY_VelDepth << std::endl;
        
        /*
        float deltaX_VelWidth, deltaY_VelWidth, deltaX_VelDepth, deltaY_VelDepth;
        correctPosForDimDiff(deltaWidth, 0, &deltaX_VelWidth, &deltaY_VelWidth, dt, z_k( yaw_zRef ) );
        correctPosForDimDiff(0, deltaDepth, &deltaX_VelDepth, &deltaY_VelDepth, dt, z_k( yaw_zRef ) );
        
        // Strategy previously tested: do a prediction of the position and check which (correction + measurement) is closest to the this prediction
        // Problem: wrong estimation of velocity can lead to measurements being corrected into the wrong direction, reflecting a position which can not be measured! (i.e., the position is closer
        // to the sensor than the measurement obtained )        
        float distPosWidth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) + deltaX_VelWidth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) + deltaY_VelWidth), 2.0 );
        float distNegWidth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) - deltaX_VelWidth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) - deltaY_VelWidth), 2.0 );
        
        float distPosDepth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) + deltaX_VelDepth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) + deltaY_VelDepth), 2.0 );
        float distNegDepth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) - deltaX_VelDepth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) - deltaY_VelDepth), 2.0 );
        
        bool largerDistanceDesiredWidth = deltaWidth > 0 ? 1 : 0 ;
        bool largerDistanceDesiredDepth = deltaDepth > 0 ? 1 : 0 ;
        
        int signWidth =  largerDistanceDesiredWidth && distPosWidth2 > distNegWidth2 || !largerDistanceDesiredWidth && distPosWidth2 <  distNegWidth2 ? 1 : -1;
        int signDepth =  largerDistanceDesiredDepth && distPosDepth2 > distNegDepth2 || !largerDistanceDesiredDepth && distPosDepth2 <  distNegDepth2 ? 1 : -1;
        
        z_k ( x_zRef ) += signWidth*deltaX_VelWidth + signDepth*deltaX_VelDepth;
        z_k ( y_zRef ) += signWidth*deltaY_VelWidth + signDepth*deltaY_VelDepth;
        /*
        /*std::cout << "Delta x meas dim = " << signWidth*deltaX_VelWidth + signDepth*deltaX_VelDepth << " Delta y dim = " << signWidth*deltaY_VelWidth + signDepth*deltaY_VelDepth<< std::endl;
        std::cout << "signWidth = " << signWidth << " signWidth = " << signDepth << std::endl;
        std::cout << "deltaX_VelWidth = " << deltaX_VelWidth << " deltaX_VelWidth = " << deltaX_VelWidth << std::endl;
        std::cout << "deltaX_VelDepth = " << deltaX_VelDepth << " deltaY_VelDepth = " << deltaY_VelDepth << std::endl;
        */
        
        Eigen::MatrixXf P = rectangle_.get_P();
        Eigen::MatrixXf x_k_1_k_1_PosVel( 6, 1 ), z_k_posVel( 3, 1 );
        x_k_1_k_1_PosVel << rectangle_.get_x(), rectangle_.get_y(), rectangle_.get_yaw(), rectangle_.get_xVel(), rectangle_.get_yVel(), rectangle_.get_yawVel();
        z_k_posVel <<       z_k ( x_zRef ),     z_k ( y_zRef ),     z_k ( yaw_zRef );
        
   //     std::cout << "x_k_1_k_1_PosVel = " << x_k_1_k_1_PosVel << " z_k_posVel = " << z_k_posVel << std::endl;
        
        Eigen::MatrixXf x_k_k_PosVel = kalmanUpdate(F_PosVel, H_PosVel, &P, x_k_1_k_1_PosVel, z_k_posVel, Q_k.block< 6, 6 >( 0, 0 ), R_k.block< 3, 3 >( 0, 0 ));
        
  //      std::cout << "x_k_k_PosVel = " << x_k_k_PosVel << std::endl;
        
        rectangle_.set_x ( x_k_k_PosVel ( x_PosVelRef ) );
        rectangle_.set_y ( x_k_k_PosVel ( y_PosVelRef ) );
        rectangle_.set_yaw ( x_k_k_PosVel ( yaw_PosVelRef ) );
        rectangle_.set_xVel ( x_k_k_PosVel ( xVel_PosVelRef ) );
        rectangle_.set_yVel ( x_k_k_PosVel ( yVel_PosVelRef ) );
        rectangle_.set_yawVel ( x_k_k_PosVel ( yawVel_PosVelRef ) );

        rectangle_.set_P ( P );
        rectangle_.set_Pdim( Pdim );
}

void FeatureProperties::correctForDimensions( float deltaWidth, float deltaDepth, float* xMeasured, float* yMeasured, float measuredPosX, float measuredPosY, float modelledPosX, float modelledPosY,  float dt )
{
        float deltaX_Width, deltaY_Width, deltaX_Depth, deltaY_Depth;
        correctPosForDimDiff(deltaWidth, 0, &deltaX_Width, &deltaY_Width, dt );
        correctPosForDimDiff(0, deltaDepth, &deltaX_Depth, &deltaY_Depth, dt );
        
        float distPosWidth2 = pow( modelledPosX + deltaX_Width - measuredPosX, 2.0 ) + pow( modelledPosY + deltaY_Width - measuredPosY, 2.0 );
        float distNegWidth2 = pow( modelledPosX - deltaX_Width - measuredPosX, 2.0 ) + pow( modelledPosY - deltaY_Width - measuredPosY, 2.0 );
        
        float distPosDepth2 = pow( modelledPosX + deltaX_Depth - measuredPosX, 2.0 ) + pow( modelledPosY + deltaY_Depth - measuredPosY, 2.0 );
        float distNegDepth2 = pow( modelledPosX - deltaX_Depth - measuredPosX, 2.0 ) + pow( modelledPosY - deltaY_Depth - measuredPosY, 2.0 );
        
        
        int signWidth =  distPosWidth2 < distNegWidth2 ? 1 : -1;
        int signDepth =  distPosDepth2 < distNegDepth2 ? 1 : -1;
        
        *xMeasured += (signWidth*deltaX_Width + signDepth*deltaX_Depth);
        *yMeasured += (signWidth*deltaY_Width + signDepth*deltaY_Depth);       
}

void FeatureProperties::correctPosForDimDiff(float deltaWidth, float deltaDepth, float *deltaX, float *deltaY, float dt)
{
        float thetaPred = rectangle_.get_yaw() + dt*rectangle_.get_yawVel();
        
    //    std::cout << "rectangle_.get_yaw() = " << rectangle_.get_yaw() << " thetaPred = " << thetaPred << std::endl;
        
        float pred_st = std::sin( thetaPred );
        float pred_ct = std::cos( thetaPred );
                
        // check in which direction the measured center-point falls in order to determine to right direction of correction in both x and y
        float rotatedX = deltaWidth * pred_ct - deltaDepth * pred_st;
        float rotatedY = deltaWidth * pred_st + deltaDepth * pred_ct;

        *deltaX = 0.5*rotatedX;
        *deltaY =  0.5*rotatedY;
}

void FeatureProperties::printProperties()
{
        std::cout << "\t";
        rectangle_.printProperties();
        circle_.printProperties();
        std::cout << "Probability circle = " ;
        std:: cout << featureProbabilities_.get_pCircle();
        std::cout << "Probability rectangle = " ;
        std::cout << featureProbabilities_.get_pRectangle() << std::endl;
}

}

}
