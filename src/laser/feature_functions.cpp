#include "feature_functions.h"

namespace tracking
{

tracking::FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine )
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
	    return tracking::NONE;	    
        }
        else if ( pointsRemoved && remainingSize >= minPointsLine ) 
        {
            *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
            return tracking::LINE;
        }
        else  
        { // we dit not remove points and a corner is present
            return tracking::RECTANGLE;
        }
    }
    else if(remainingSize >= minPointsLine)
    {
        return tracking::LINE;
    }
    else{
        return tracking::NONE;
    }
        return tracking::NONE;
}

float fitObject ( std::vector<geo::Vec2f>& points, int FITTINGMETHOD,  unsigned int* cornerIndex, tracking::Rectangle* rectangle, tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose, unsigned int minPointsLine )
{
    switch ( FITTINGMETHOD )
    {
    case tracking::NONE:
    {
        return std::numeric_limits<float>::infinity();
    }
    case tracking::LINE:
    {
        return setRectangularParametersForLine ( points,  it_low,  it_high, rectangle, sensor_pose, minPointsLine );
    }
    case tracking::CIRCLE:
    {
        return fitCircle ( points, circle, sensor_pose );
    }
    case tracking::RECTANGLE:
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

//Fast Line, Arc/Circle and Leg Detection from Laser Scan Data in a Player Driver: http://miarn.sourceforge.net/pdf/a1738b.pdf
float fitCircle ( std::vector<geo::Vec2f>& points, tracking::Circle* circle, const geo::Pose3D& pose )
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

    float roll = 0.0, pitch = 0.0, yaw = 0.0, xVel = 0.0, yVel = 0.0;
    circle->setProperties ( xc, yc, pose.getOrigin().getZ(), xVel, yVel,  roll, pitch, yaw, radius); // Assumption: object-height identical to sensor-height
    
    return sum/points.size();
}

float fitRectangle ( std::vector<geo::Vec2f>& points, tracking::Rectangle* rectangle, const geo::Pose3D& pose , unsigned int cornerIndex, unsigned int minPointsLine )
{

    std::vector<geo::Vec2f>::iterator it_start = points.begin();
    std::vector<geo::Vec2f>::iterator it_splitLow = points.begin() + cornerIndex - 1; // +1 and -1 to prevent problems when a corner is not visible due to an occlusion
    std::vector<geo::Vec2f>::iterator it_splitHigh = points.begin() + cornerIndex + 1;
    std::vector<geo::Vec2f>::iterator it_end = points.end();

    Eigen::VectorXf beta_hat1 ( 2 ), beta_hat2 ( 2 );
    
    if( std::distance(points.begin(), it_splitHigh ) > points.size() )
    {
             ROS_WARN("Potential problem in fitRectangle-function! std::distance(points.begin(), it_splitHigh ) > points.size(), std::distance(points.begin(), it_splitHigh )= %lu, points.size() = %lu", std::distance(points.begin(), it_splitHigh ), points.size());
    }

    float mean_error1 = fitLine ( points, beta_hat1, &it_start, &it_splitLow ); //
    float mean_error2 = fitLine ( points, beta_hat2, &it_splitHigh, &it_end );

    unsigned int nPointsForExtrema;
    minPointsLine % 2 == 0 ? nPointsForExtrema = minPointsLine / 2 : nPointsForExtrema = (minPointsLine - 1) / 2;

    if(nPointsForExtrema > points.size() )
    {
            ROS_WARN("Potential problem in fitRectangle-function! nPointsForExtrema > points.size(), nPointsForExtrema = %u minPointsLine = %u, points.size() = %lu", nPointsForExtrema, minPointsLine, points.size());
    }
    
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

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose, unsigned int minPointsLine )
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

bool splitSegmentsWhenGapDetected( std::vector< PointsInfo >& associatedPointsInfo, int min_gap_size_for_split,
                                   int min_segment_size_pixels, float dist_for_object_split, std::vector<float>& sensor_ranges, 
                                   const sensor_msgs::LaserScan::ConstPtr& scan)
{
        bool segmentSplitted = false;
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

                            float maxReference = std::max(avgRangeLow, avgRangeHigh);
                            unsigned int nLargerRanges = 0;

                            for(unsigned int iGap = IDs[idLow]; iGap < IDs[iIDs]; iGap++ )
                            {       
                                   if(  sensor_ranges[iGap] > maxReference ||  sensor_ranges[iGap] == 0.0 ) // as points associated to the world are set to 0
                                    {
                                            nLargerRanges++;
                                    }
                                    else
                                    {
                                            nLargerRanges = 0;
                                    }

                                    if(nLargerRanges >= min_gap_size_for_split)
                                    {
                                            PointsInfo splittedInfo;

                                            std::vector<geo::Vec2f> points = associatedPointsInfo[iList].points;
                                            std::copy( associatedPointsInfo[iList].laserIDs.begin() + iIDs, associatedPointsInfo[iList].laserIDs.end(), std::back_inserter(splittedInfo.laserIDs) );
                                            std::copy( associatedPointsInfo[iList].points.begin() + iIDs, associatedPointsInfo[iList].points.end(),  std::back_inserter(splittedInfo.points) );
                                            associatedPointsInfo.push_back( splittedInfo );
                                            associatedPointsInfo[iList].laserIDs.erase (associatedPointsInfo[iList].laserIDs.begin() + iIDs, associatedPointsInfo[iList].laserIDs.end() );
                                            associatedPointsInfo[iList].points.erase (associatedPointsInfo[iList].points.begin() + iIDs, associatedPointsInfo[iList].points.end() );
                                            
                                            ROS_WARN("Segment splitted based on min_gap_size_for_split-criterion. ");
                                            segmentSplitted = true;
                                            
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
                        segmentSplitted = true;
                        splitFound = true;
                        
                        ROS_WARN("Segment splitted based on dist_for_object_split-criterion (v1). ");
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
                                     
                                     ROS_WARN("Segment splitted based on dist_for_object_split-criterion (v2). ");
                                     segmentSplitted = true;
                                       
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
    return segmentSplitted;
}

std::vector<tracking::ScanSegment> determineSegments(std::vector<float> sensor_ranges, int maxGapSize, int minSegmentSize, 
                                                float segmentdepthThreshold, geo::LaserRangeFinder lrf_model, 
                                                double minClusterSize, double maxClusterSize, bool checkMinSizeCriteria)
{
        unsigned int num_beams = sensor_ranges.size();
        std::vector<tracking::ScanSegment> segments;
 
        // Find first valid value
        tracking::ScanSegment current_segment;
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

                if (    rs == 0 || 
                        (std::abs(rs - sensor_ranges[current_segment.back()]) > segmentdepthThreshold ) ||  // if there is a minimum number of points not associated
                        i == num_beams - 1)
                {
                        // Found a gap
                        ++gap_size;
                        gapRanges.push_back ( rs );

                        if (gap_size >= maxGapSize || i == num_beams - 1)
                        {
                                i = current_segment.back() + 1;

                                if (current_segment.size()  >= minSegmentSize || !checkMinSizeCriteria)
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
                                        if ( ( bb .x > minClusterSize || bb.y > minClusterSize || !checkMinSizeCriteria) && 
                                                bb.x < maxClusterSize && bb.y < maxClusterSize )
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

}