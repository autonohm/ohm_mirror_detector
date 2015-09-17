/**
 * @file   ohm_mirror_postfilter.h
 * @author Rainer Koch
 * @date   07.09.2015
 *
 *
 */

#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <ohm_mirror_detector/ohm_sensor_msgs.h>
#include <ohm_mirror_detector/ohm_maskLaser_msgs.h>


#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"


/*
 * _typeID
 * 0 = POINT
 * 1 = MIRROR
 * 2 = ERROR_Mirror
 * 3 = TRANSPARENT
 * 4 = ERROR_TRANSPARENT
 */
void subscribermaskScan(const ohm_mirror_detector::ohm_maskLaser_msgs& maskScan);

void subscriberActivatePub(std_msgs::Bool activate);

void publisherFuncScan(std::vector<obvious::SensorPolar2D*> history, std::vector<obvious::SensorPolar2D*> history2, int historynr);
void pubPoints();
void pubLines();

/*
 * Main function of the ohm_mirror_postfilter
 */
void postFiltering(std::vector<obvious::SensorPolar2D*> scan_history, std::vector<obvious::Matrix*>& mHistory, std::vector<
    obvious::Matrix*>& mHistory_T, unsigned int history_count, unsigned int& mirror_count);

/*
 * getCornerPoints -> checked
 * returns the left and right corner of the mirror line
 * Input:
 * mirror_matrix:	matrix with xy-values of the mirror_line points
 *
 * Output:
 * corners			first value is left corner, second value is right corner
 *
 * Return:
 * true, if corners are valid
 */
bool getCornerPoints(obvious::Matrix mirror_matrix, obvious::Matrix& corners, bool* mask);

/*
 * Check if mirror/corners already exist
 *
 * Version 2:
 * checking if one of the corner points is more than threshold away
 * if so => check if point is still on the line of the old mirror threshold 2
 * => update corners or new mirror
 *
 * Input:
 * corner_points:	current corner points, which has to been checked
 * history:				history of old corner points
 * threshold:			max distance between two corner points
 * historysize:		size of mirror lines in the history (2 points equals one mirror line => half of points in history)
 *
 * Return:
 * 0 = points do not exist,
 * 1 = corner updated
 * 2 = points exist
 */
bool checkMirrorHistory(obvious::Matrix& corner_points, obvious::Matrix& T_corner_points, std::vector<obvious::Matrix*>& history, std::vector<
    obvious::Matrix*>& T_history, double th_new_mirror, double th_mirrorline, unsigned int& historysize);

/*
 * check if point is next to the mirror line.
 * => if point is close to the mirrorline, update the corners, if necessary
 */
bool checkPointToUpdate(obvious::Matrix* history, obvious::Matrix& corner_points, double th_mirrorline);

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
 * calculates the line values rise and offset -> checked
 * Input:
 * points:		left and right corner point of mirror line
 *
 * Output:
 * t:					offset of line
 * m:					rise of line
 */
bool calcLineValues(obvious::Matrix& points, float& t, float& m);

/*
 * calculates the length of the line between point 1 and point 2 -> checked
 */
double calcDistOfLine(obvious::Matrix& point1, obvious::Matrix& point2);

/*
 * calculate the distance between point and intersection point of line (defined by their two corner points) -> checked
 */
double calcDistToLine(obvious::Matrix& point, obvious::Matrix& corners);

/*
 * clean outgoing scans from affected points
 *
 * Input:
 * corners    left and right corner point of mirror line
 *
 * Output:
 * sensor     changes the object mask of the sensor
 */
void cleanScan(obvious::Matrix& corners, obvious::SensorPolar2D* sensor, double th_mirrorline, double th_angleThreshold);

void showTransformation(obvious::Matrix T);
