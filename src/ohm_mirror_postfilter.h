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
#include "ransac.h"
#include "reflectionIdentifier.h"


#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"



/*
 *
 * 0 = POINT
 * 1 = MIRROR
 * 2 = ERROR_Mirror
 * 3 = TRANSPARENT
 * 4 = ERROR_TRANSPARENT
 * 5 = error_mirror recalculated
 */

//void subscriberSensor(const ohm_mirror_detector::ohm_sensor_msgs& sensor);
void subscribermaskScan(const ohm_mirror_detector::ohm_maskLaser_msgs& maskScan);

void subscriberActivatePub(std_msgs::Bool activate);

/*
 * publish scan
 * Input:
 * history:     history of scans of echo 1
 * history2:    history of scans of echo 2
 * historynr    size of histories
 *
 * Return:
 * 1 => The scan has been send.
 * 0 => There was a mirror or transparent object -> Have to send the second part of the scan.
 *
 */
void publisherFuncScan(std::vector<obvious::SensorPolar2D*>& history, std::vector<obvious::SensorPolar2D*>& history2, std::vector<obvious::Matrix*>& cornerHistory, int historynr, int mirror_histCounter, int* object_type);
void pubPoints();
void pubLines();

/*
 * build the mirror history as well as a history of all corresponding poses
 * Input:
 * scan_history:    history of scan points
 * history_count:   amount of points in the scan_history
 *
 * Output:
 * mHistory:        history of mirror points (in real world coordinates!!!!), copied out of the scan_history
 * mHistory_T:      poses history
 * mirror_count:    amount of points in the mHistory
 * mHistoryIntens1  intensity 1 of mHistory
 * mHistoryIntens2  intensity 2 of mHistory
 *
 */
void buildHistory(std::vector<obvious::SensorPolar2D*>& scan_history, std::vector<obvious::SensorPolar2D*>& scan_history2, std::vector<obvious::Matrix*>& mHistory, std::vector<obvious::Matrix*>& mHistory2, std::vector<
    obvious::Matrix*>& mHistory_T, unsigned int history_count, unsigned int& mirror_count, std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2);

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
bool getCornerPoints2( std::vector<obvious::Matrix*>& mirror_pointsHistory, std::vector<obvious::Matrix*>& cornerHistory, unsigned int& counter_points, unsigned int& counter_corners, int ransac_points2fit, int ransac_iterations, double ransac_threshold, double th_mirrorline, int* objectIdentificationResult);

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
 * clean outgoing scans from affected points
 *
 * Input:
 * corners      left and right corner point of mirror line
 * object_type  type of the object described by the two corners
 * moved_mask   mask, if the point already was moved before
 *
 * Output:
 * sensor       changes points which are affected by the mirror -> displace them to original position
 */
bool replaceMirroredPoints(obvious::Matrix& corners, obvious::SensorPolar2D* sensor, double* replaced_coords, int object_type,  bool* moved_mask);

/*
 * clean outgoing scans from affected points
 *
 * Input:
 * corners            left and right corner point of mirror line
 * th_mirrorline      threshold to line of reflective object
 * th_angleThreshold  threshold to enlarge the corners
 * object_type        type of the object described by the two corners
 *
 * Output:
 * sensor     changes the object mask of the sensor
 */
void cleanScan(obvious::Matrix& corners, obvious::SensorPolar2D* sensor, double th_mirrorline, double th_angleThreshold, int object_type);


void showTransformation(obvious::Matrix T);

