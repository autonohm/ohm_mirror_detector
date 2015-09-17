/**
* @file   ohm_mirror_prefilter.h
* @author Rainer Koch
* @date   09.07.2015
*
*
*/

#include <sensor_msgs/LaserScan.h>
#include <ohm_mirror_detector/ohm_maskLaser_msgs.h>


#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * _typeID
 * 0 = POINT
 * 1 = MIRROR
 * 2 = ERROR_Mirror
 * 3 = TRANSPARENT
 * 4 = ERROR_TRANSPARENT
 */
/*
 * filter out single points from the scan
 * Input:
 * scan:        laser scan
 * intensity:   intensity of laser scan
 * threshold:   threshold value for max distance between valid points
 *
 * Output:
 * scan:        laser scan
 * intensity:   intensity of laser scan
 *
 * Version0:    only filters by checking the distance to the neighbor
 * Version1:    also check if the angle between to the next neighbor is not to big
 * Version2:    check previous and following two poins to validate, also check if the angle between to the next neighbor is not to big

 *
 */
void particleFilter(std::vector<float>& scan, std::vector<float>& intensity, float threshold);
void particleFilter1(std::vector<float>& scan, std::vector<float>& intensity, float threshold_dist, int threshold_angle_steps);

/*
 * locates the mirror line and get the corner points of the line
 * Input:
 * scan:                  laser scan
 * mask_line_points:      bool mask
 *
 * Output:
 * corner_line_points:    edges (xy-data) of mirror line
 */
bool findMirrorLine(std::vector<float> scan, std::vector<cv::Point2f>& corner_line_points, std::vector<bool*>& mask_line_points, float threshold);

/*
 * calcLine
 * Input:
 * vector<cv::Points2f>   points:     two points (xy-data) on the line
 *
 * Output:
 * float t:   y-offset of line
 * float m:   gradient of line
 *
 * Return:
 * bool       sucessful = 1/unsucessful = 0
 */
bool calcLineValues(std::vector<cv::Point2f> points, float& t, float& m);

/*
 * creates a vision cone to delete points which are not marked with 1 at the mask
 * Input:
 * mask:      bool mask
 *
 * Output:
 * scan:      laser scan
 */
void mirrorVisionCone(std::vector<float>& scan, std::vector<float>& intensity, bool* mask);
void mirrorVisionCone_multi(std::vector<float>& scan, std::vector<float>& intensity, std::vector<bool*>& mask);

/*
 * mirror affected points on the mirror lane to put them on their correct position
 * Input:
 * scan:                  laser scan
 * s_intens:              intensity values
 *
 * _mirror_line_points:   two xy-points on the mirror line
 *
 * Output:
 * recalculated_scan:     laser scan
 * rec_intens:            intensity values
 *
 */
void recalculateAffectedPoins(std::vector<float> scan, std::vector<float>& s_intens, std::vector<float>& recalculated_scan, std::vector<float>& rec_intens, std::vector<cv::Point2f> line_points);

/*
 * get the corner points of the mirror line
 * Input:
 * scan:            scan with mirror points
 * line_points:     2 xy-points on the mirror line
 *
 * Output:
 * corner_points:   2 xy-points, edges of the mirror line (assumption: scanner runs clockwise => left corner has index 0, right corner has index 1)
 */
void markMirrorArea(std::vector<float> scan, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corner_points, bool* mask_linepoints);
void markMirrorArea_multi(std::vector<float> scan, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corner_points, std::vector<bool*>& mask_linepoints);

/*
 * convert a laser scan to xy-data
 * Input:
 * scan:              laser scan
 * angle_increment:   angle between two scan points (rad)
 * scanangle:         scanangle of laser scanner (rad) 2*PI = 360°
 *
 * Output:
 * data:              xy-data
 */
void convertScan2xy(std::vector<float> scan, std::vector<cv::Point2f> &data, float angle_increment, float scanangle);

/*
 * convert xy-data to laser scan
 * Input:
 * data:              xy-data
 * angle_increment:   angle between two scan points (rad)
 * scanangle:         scanangle of laser scanner (rad) 2*PI = 360°
 *
 * Output:
 * scan:              laser scan
 */
void convertxy2scan(std::vector<cv::Point2f> data, std::vector<float> &scan, float angle_increment, float scanangle);

/*
 * set all points to 0
 * Input:
 * scan: laser scan
 */
void cleanUpScan(std::vector<float> &scan);

/*
 * check scan for points on the mirror and put them into the mirror point cloud:
 */
void sortingScanOnLineToMirror(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& mirror, std::vector<float>& m_intens, bool* mask_linepoints, std::vector<cv::Point2f> line_points, float threshold);
void sortingScanOnLineToMirror_multi(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& mirror, std::vector<float>& m_intens, std::vector<bool*>& mask_linepoints, std::vector<cv::Point2f> line_points, float threshold);

/*
 * take the masked points and put all data from the scan into the affected area, which are masked = true
 * Input:
 * scan:              laser scan
 * mask_linepoints:   bool mask
 *
 * Output:
 * affected:          laser scan
 */
//void scanRefilteredToAffected(std::vector<float> scan, std::vector<float>& affected, bool* mask_linepoints);
void sortingScanOnLineToAffected(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& affected, std::vector<float>& a_intens, bool* mask_linepoints);
void sortingScanOnLineToAffected_multi(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& affected, std::vector<float>& a_intens, std::vector<bool*>& mask_linepoints);

/*
 * subtract first and second echo depending on the threshold value
 * Input:
 * first    - first echo
 * last     - last echo
 *
 * Output:
 * good     - points defined as not affected by the threshold
 * mirror   - points defined as mirror plane
 * affected - points defined as affected by reflections
 *
 * Return:
 * true     - there was a difference between the scans -> subtraction was done
 * false    - both scans are identical
 */
bool subtractScans(sensor_msgs::LaserScan& first, sensor_msgs::LaserScan& last, sensor_msgs::LaserScan& good, sensor_msgs::LaserScan& mirror, sensor_msgs::LaserScan& affected, float threshold);

void pubPoints();
void pubLines();
void pubLineList();

/*
 * copy header of source scan to header of goal scan
 */
void writeHeader(sensor_msgs::LaserScan& goal, const sensor_msgs::LaserScan& source);


