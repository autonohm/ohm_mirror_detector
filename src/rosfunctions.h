/**
* @file   rosfunctions.cpp
* @author Rainer Koch
* @date   14.04.2015
*
*
*/


#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * converting a ros path to xy-data
 */
void convertPath2xyz(nav_msgs::Path& ros_path, std::vector<cv::Point3f>& coords_path, std::vector<cv::Vec4f>& path_orient);

/*
 * converting xy-data to ros a path
 */
void convertPath2ROS(std::vector<cv::Point3f>& coords_path, std::vector<cv::Vec4f>& path_orient, nav_msgs::Path& ros_path);
/*
 * converting xy-data to ros a occopancy grid map
 */
void convertMap2ROS(std::vector<cv::Point2f>& coords, std::vector<int>& mask, cv::Vec4f& map_orient, cv::Point3f& map_pos, nav_msgs::OccupancyGrid& map);

/*
 * converting a ros occopancy grid map to xy-data
 */
void convertMap2xy(nav_msgs::OccupancyGrid& map, std::vector<cv::Point2f>& coords, std::vector<int>& mask);

/*
 * convert a laser scan to xy-data and sort
 * Input:
 * coords:            xy-coords
 * angle_inc:         angle between two scan points (rad)
 * size:              amount of laser scan points
 *
 * Output:
 * distance:          laser scan
 */
void convertxy2RosSort(double* coords, double* distance, int size, double angle_inc);

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
void convertROS2xy(sensor_msgs::LaserScan &scan, std::vector<cv::Point2f> &data, float angle_increment, float scanangle);

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
void convertxy2ROS(std::vector<cv::Point2f> &data, sensor_msgs::LaserScan &scan, float angle_increment, float scanangle);

void copyHeaderScan(sensor_msgs::LaserScan scan_in, sensor_msgs::LaserScan &scan_out);

