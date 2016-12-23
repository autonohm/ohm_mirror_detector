/**
* @file   lineFunctions.h
* @author Rainer Koch
* @date   28.01.2016
*
*
*/

#ifndef LINEFUNCTIONS_H_
#define LINEFUNCTIONS_H_

#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Time.h"

#include "obvision/reconstruct/grid/SensorPolar2D.h"

#include <opencv2/core/core.hpp>


/*
 * calculates the angle of the point
 * input:
 * origin     coordinates x,y
 * point      coordinates x,y
 *
 * return:
 * angle    [°]
 */
double getAngle(obvious::Matrix& origin, obvious::Matrix& point);

/*
 * calculates the angle of the point
 * input:
 * origin     coordinates x,y
 * point      coordinates x,y
 * corners    coordinates of c1 and c2 of the line
 *
 * return:
 * angle    [°]
 */
double getAngle2Normal(obvious::Matrix& origin, obvious::Matrix& point, obvious::Matrix& corners);

/*
 * calculates the length of the line between point 1 and point 2 -> checked
 */
double calcDistOfLine(obvious::Matrix& point1, obvious::Matrix& point2);

/*
 * calculates the length of a vector
 */
double calcDistOfLine(obvious::Matrix& vector);


/*
 * calculate the distance between point and intersection point of line (defined by their two corner points) -> checked
 */
double calcDistToLine(obvious::Matrix& point, obvious::Matrix& corners);

/*
 * calculate intersection point
 */
void calcIntersectionPoint(obvious::Matrix& corners, obvious::Matrix& point, obvious::Matrix& intersection);

/*
 * check if point is next to the mirror line.
 * => if point is close to the mirrorline, update the corners, if necessary
 */
bool checkPointToUpdate(obvious::Matrix* history, obvious::Matrix& corner_points, double th_mirrorline);

/*
 * calculates the line values rise and offset -> checked
 * Input:
 * points:    left and right corner point of mirror line
 *
 * Output:
 * t:         offset of line
 * m:         rise of line
 */
bool calcLineValues(obvious::Matrix& points, float& t, float& m);

/*
 * check if the direction of the vectors are the same -> checked
 * Input:
 * origin:    origin of both vectors
 * point_1:   end point of vector 1
 * point_2:   end point of vector 2
 *
 * Return:
 * true:      same direction
 * false:     not same direction
 */
bool checkDirection(obvious::Matrix* origin, obvious::Matrix* point_1, obvious::Matrix* point_2);

/*
 * check if point is on line and between corners of object
 *
 * Return:
 * true -> point is part of object
 * false -> point is not part of object
 */
bool checkPointOnObject(obvious::Matrix& point, obvious::Matrix& corners, double thres_line);

/*
 * check if point is on line and between corners of object
 *
 * Return:
 * true -> point is part behind object
 * false -> point is not behind object
 */
bool checkPointBehindObject(obvious::Matrix& position, obvious::Matrix& point, obvious::Matrix& corners, double thres_line, double thres_angle);

/*
 * get the corner points of the mirror line
 * Input:
 * scan:            scan with mirror points
 * line_points:     2 xy-points on the mirror line
 *
 * Output:
 * corner_points:   2 xy-points, edges of the mirror line (assumption: scanner runs clockwise => left corner has index 0, right corner has index 1)
 */
void getOuterLinePoints(std::vector<cv::Point2f> points, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corners, bool* mask_linepoints, float threshold);

/*
 * get the corner points of the mirror line
 * Input:
 * scan:            scan with mirror points
 * line_points:     2 xy-points on the mirror line
 *
 * Output:
 * corner_points:   2 xy-points, check which is the most further point on the vektor of the corners
 */
void getOuterLinePoints2(std::vector<cv::Point2f> points, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corners, bool* mask_linepoints, float threshold);

/*
 * calculates the perpendicular point to line -> checked
  * Input:
 * corners:     corner 1 and 2 of line
 * point:       point to get the of the distance of corner 1 to the perpendicular point (intersection point)
 *
 * Return:
 * distance to corner 1
 */
double distOnLine(obvious::Matrix* corners, obvious::Matrix* point);

/*
 * calculates the perpendicular point to line
  * Input:
 * corners:     corner 1 and 2 of line
 * point:       point to get the perpendicular to the line
 *
 * Output:
 * intersection:   perpendicular point to line
 */
void getPerpendicularPoint(obvious::Matrix* intersection, obvious::Matrix& corners, obvious::Matrix& point);

void replacePoints(obvious::Matrix& corners, obvious::Matrix& points, double* replaced_coords);


#endif /* LINEFUNCTIONS_H_ */
