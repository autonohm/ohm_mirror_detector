/**
* @file   ohm_mirror_prefilter
* @author Rainer Koch
* @date   07.09.2015
*
*
*/


#include "ohm_mirror_prefilter.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <ohm_mirror_detector/ohm_maskLaser_msgs.h>


#include <cmath>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "obcore/base/Time.h"

#include "ransac.h"
#include "rosfunctions.h"

#include <sstream>

// To write in file
#include <iostream>
#include <fstream>

#define _USE_MATH_DEFINES

using std::vector;
using namespace std;
using namespace cv;

unsigned int _scanSize            = 0;
unsigned int _sequenzNr           = 0;


bool _new_dataset_first           = false;
bool _new_dataset_last            = false;

float _substract_threshold_dist   = 0.0;
float _substract_threshold_int    = 0.0;
float _switchfactor_int           = 0.0;
float _switch_min_point           = 10;
bool _switch_reflectiontype       = false;
bool _multiline                   = false;
int _minMeasureDistance           = 23;
int _maxMeasureDistance           = 60000;

float _particlefilter_thres_dist_mirror   = 0.001;
int _particlefilter_threshold_angle       = 10;       // 1 step = 0.25°
float _particlefilter_thres_dist_affect   = 0.01;

int _ransac_iterations            = 100;               // max. number of iterations allowed in the ransac
float _ransac_threshold           = 0.05;            // threshold value for determining when a data point fits the model
int _ransac_points2fit            = 20;                // min. number of data values required to fit the model

float _angle_inc;
float _angle_diff;
bool _ransac_success              = 0;

float f                           = 0.0;

sensor_msgs::LaserScan _first_scan;
sensor_msgs::LaserScan _last_scan;
sensor_msgs::LaserScan _scan;
sensor_msgs::LaserScan _mirror;
sensor_msgs::LaserScan _transp;
sensor_msgs::LaserScan _affected_transp;
sensor_msgs::LaserScan _affected_mirror;
sensor_msgs::LaserScan _check;
sensor_msgs::PointCloud _cloud;

ohm_mirror_detector::ohm_maskLaser_msgs _maskLaser;

//mirror variables
vector<Point2f> _mirror_line_points;

//marker variables
visualization_msgs::Marker _marker_points;
visualization_msgs::Marker _marker_line_strip;
visualization_msgs::Marker _marker_line_list;

// ROS variables
ros::Publisher _pub_scan;
ros::Publisher _pub_mirror;
ros::Publisher _pub_transp;
ros::Publisher _pub_affected_mirror;
ros::Publisher _pub_affected_transp;
ros::Publisher _pub_maskScan;
ros::Publisher _pub_check;
ros::Publisher _pub_cloud;
ros::Publisher _pub_marker;

unsigned int _lifetime_marker = 1;  //ns  

/*
 * Subscriber & Publisher
 */
void subscriberFunc_first(const sensor_msgs::LaserScan& msg)
{
  if((!_new_dataset_first)) 
  {
    // copy msg information
    copyHeaderScan(msg, _first_scan);

    _angle_inc = _first_scan.angle_increment;
    _angle_diff = msg.angle_max - msg.angle_min;
    _scanSize = (unsigned int) ((_first_scan.angle_max - _first_scan.angle_min) / _first_scan.angle_increment);

    _first_scan.ranges.resize(_scanSize);
    _first_scan.intensities.resize(_scanSize);
    for (unsigned int i = 0; i < _scanSize; i++)
    {
      _first_scan.ranges[i] = msg.ranges[i];
      _first_scan.intensities[i] = msg.intensities[i];
    }
    if(_scanSize > 0)
    {
      _new_dataset_first = true;
    }
    /*
     * CleanScan: filter points, which are out of Disance range
     */
    distanceFilter(_first_scan.ranges, _first_scan.intensities, _minMeasureDistance, _maxMeasureDistance);
  }
}
void subscriberFunc_last(const sensor_msgs::LaserScan& msg)
{
  if(!_new_dataset_last) 
	{
    // copy msg information
    copyHeaderScan(msg, _last_scan);

    _last_scan.ranges.resize(_scanSize);
    _last_scan.intensities.resize(_scanSize);
    int tmp_size;
    tmp_size = (unsigned int) ((msg.angle_max - msg.angle_min) / msg.angle_increment);

    for (unsigned int i = 0; i < _scanSize; i++)
    {
      _last_scan.ranges[i] = msg.ranges[i];
      _last_scan.intensities[i] = msg.intensities[i];
    }
    if(_scanSize > 0)
    {
      _new_dataset_last = true;
    }
    /*
     * CleanScan: filter points, which are out of Disance range
     */
    distanceFilter(_last_scan.ranges, _last_scan.intensities, _minMeasureDistance, _maxMeasureDistance);
  }
}

void publisherFunc(void)
{
  _maskLaser.header.frame_id = _first_scan.header.frame_id;
  _maskLaser.header.stamp    = _first_scan.header.stamp;
  _maskLaser.header.seq      = _first_scan.header.seq;

  copyHeaderScan(_first_scan, _maskLaser.echo_1);
  copyHeaderScan(_first_scan, _maskLaser.echo_2);
  copyHeaderScan(_first_scan, _scan);
  copyHeaderScan(_first_scan, _mirror);
  copyHeaderScan(_first_scan, _transp);
  copyHeaderScan(_first_scan, _affected_mirror);
  copyHeaderScan(_first_scan, _affected_transp);
 
  _cloud.header.frame_id = _first_scan.header.frame_id;
  _cloud.header.stamp    = _first_scan.header.stamp;
  _cloud.header.seq      = _first_scan.header.seq;

   _maskLaser.echo_1.ranges.resize(_scanSize);
  _maskLaser.echo_1.intensities.resize(_scanSize);
  _maskLaser.echo_2.ranges.resize(_scanSize);
  _maskLaser.echo_2.intensities.resize(_scanSize);
  _maskLaser.object_mask.resize(_scanSize);

  for(int i=0; i < _scanSize; i++)
  {
    if(_scan.ranges[i] != 0) // POINT
    {
      _maskLaser.echo_1.ranges[i]       = _scan.ranges[i];
      _maskLaser.echo_1.intensities[i]  = _scan.intensities[i];
      _maskLaser.object_mask[i]         = 0;
    }
    else if(_mirror.ranges[i] != 0) // MIRROR
    {
      _maskLaser.echo_1.ranges[i]       = _mirror.ranges[i];
      _maskLaser.echo_1.intensities[i]  = _mirror.intensities[i];
      _maskLaser.echo_2.ranges[i]       = _affected_mirror.ranges[i];
      _maskLaser.echo_2.intensities[i]  = _affected_mirror.intensities[i];
      _maskLaser.object_mask[i]         = 1;
    }
    else if(_transp.ranges[i] != 0) // TRANSPARENT
    {
      _maskLaser.echo_1.ranges[i]       = _transp.ranges[i];
      _maskLaser.echo_1.intensities[i]  = _transp.intensities[i];
      _maskLaser.echo_2.ranges[i]       = _affected_transp.ranges[i];
      _maskLaser.echo_2.intensities[i]  = _affected_transp.intensities[i];
      _maskLaser.object_mask[i]         = 3;

    }
    else if(_affected_transp.ranges[i] != 0) // ERROR_TRANSPARENT
    {
      _maskLaser.echo_2.ranges[i]       = _affected_transp.ranges[i];
      _maskLaser.echo_2.intensities[i]  = _affected_transp.intensities[i];
      _maskLaser.object_mask[i]         = 4;
    }
    else if(_affected_mirror.ranges[i] != 0) // ERROR_MIRROR
    {
      _maskLaser.echo_2.ranges[i]       = _affected_mirror.ranges[i];
      _maskLaser.echo_2.intensities[i]  = _affected_mirror.intensities[i];
      _maskLaser.object_mask[i]         = 2;
    }
  }

  _pub_scan.publish(_scan);
  _pub_transp.publish(_transp);
  _pub_affected_mirror.publish(_affected_mirror);
  _pub_affected_transp.publish(_affected_transp);
  _pub_mirror.publish(_mirror);
  _pub_maskScan.publish(_maskLaser);

  /*
   * clean up old data
   */
   cleanUpScan(_scan.ranges);
   cleanUpScan(_mirror.ranges);
   cleanUpScan(_transp.ranges);
   cleanUpScan(_affected_transp.ranges);
   cleanUpScan(_affected_mirror.ranges);
   cleanUpScan(_maskLaser.echo_1.ranges);
   cleanUpScan(_maskLaser.echo_2.ranges);
   cleanUpScan(_check.ranges);

   cleanUpScan(_scan.intensities);
   cleanUpScan(_mirror.intensities);
   cleanUpScan(_transp.intensities);
   cleanUpScan(_affected_transp.intensities);
   cleanUpScan(_affected_mirror.intensities);
   cleanUpScan(_maskLaser.echo_1.intensities);
   cleanUpScan(_maskLaser.echo_2.intensities);
   cleanUpScan(_check.intensities);
}

void pubPoints()
{
  _marker_points.header.frame_id = "laser";
  _marker_points.header.stamp  = ros::Time();
  _marker_points.ns = "points";
  _marker_points.action = visualization_msgs::Marker::MODIFY;
  _marker_points.pose.orientation.w  = 1.0;
  _marker_points.type = visualization_msgs::Marker::POINTS;
  _marker_points.id = 0;
  _marker_points.lifetime = ros::Duration(0,1);

  _pub_marker.publish(_marker_points);

}
void pubLines()
{
  _marker_line_strip.header.frame_id = "laser";
  _marker_line_strip.header.stamp  = ros::Time();
  _marker_line_strip.ns = "lines";
  _marker_line_strip.action = visualization_msgs::Marker::MODIFY;
  _marker_line_strip.pose.orientation.w  = 1.0;
  _marker_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  _marker_line_strip.id = 1;
  _marker_line_strip.lifetime = ros::Duration(0,_lifetime_marker);

  _pub_marker.publish(_marker_line_strip);
}
void pubLineList()
{

  _marker_line_list.header.frame_id = "laser";
  _marker_line_list.header.stamp  = ros::Time();
  _marker_line_list.ns = "line list";
  _marker_line_list.action = visualization_msgs::Marker::MODIFY;
  _marker_line_list.pose.orientation.w  = 1.0;
  _marker_line_list.type = visualization_msgs::Marker::LINE_LIST;
  _marker_line_list.id = 2;
  _marker_line_list.lifetime = ros::Duration(0,_lifetime_marker);

  _pub_marker.publish(_marker_line_list);

}

/*
 * filter functions
 */
void distanceFilter(std::vector<float>&scan, std::vector<float>& intensities, int minDist, int maxDist)
{
  for(int i=0; i< sizeof(scan); i++)
  {
    if((scan[i] < minDist) or (scan[i] > maxDist))
    {
      scan[i] = 0;
      intensities[i] = 0;
    }
  }
}

void particleFilter(std::vector<float>& scan, std::vector<float>& intensity, float threshold)
{
  for (unsigned int i = 0; i < (scan.size()-1); i++)
  {
    if(abs(scan[i]-scan[i+1]) > threshold)
    {
        scan[i] = 0.0;
        intensity[i] = 0.0;
    }
  }
}
void particleFilter1(std::vector<float>& scan, std::vector<float>& intensity, float threshold_dist, int threshold_angle_steps)
{
  /*
   * check if the distance to the next point smaller than the threshold
   */
  for (unsigned int i = 0; i < (scan.size()-1); i++)
  {
    if(abs(scan[i]-scan[i+1]) > threshold_dist)
    {
        scan[i] = 0.0;
        intensity[i] = 0.0;
    }
  }

  /*
   * check if the next point is maximal angle step difference
   */
  for(unsigned int i = 0; i < scan.size(); i++)
  {
    if(scan[i] > 0)
    {
      for(unsigned int j = (i+1); j < scan.size(); j++)
      {
        if(scan[j] > 0)
        {
          if((j-i) < threshold_angle_steps)
          {
            j = scan.size();
          }
          else
          {
            scan[i] = 0.0;
            intensity[i] = 0.0;
            j = scan.size();
          }
        }
      }
    }
  }
}

void mirrorVisionCone(std::vector<float>& scan, std::vector<float>& intensity, bool* mask)
{
  // delete all points which have not a 1 at the mask
  for(unsigned int i = 0; i < scan.size(); i++)
  {
    if(!mask[i])
    {
      scan[i] = 0.0;
      intensity[i] = 0.0;
    }
  }
}
void mirrorVisionCone_multi(std::vector<float>& scan, std::vector<float>& intensity, std::vector<bool*>& mask)
{
  for(int i=0; i < mask.size(); i++)
  {
    mirrorVisionCone(scan, intensity, mask[i]);
  }
}

void markMirrorArea(std::vector<float> scan, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corner_points, bool* mask_linepoints)
{
  int i = 0;
  int first_1 = 0;
  int last_1 = 0;
  int first_2 = 0;
  int last_2 = 0;
  bool found = false;
  int max_inc = 2*M_PI / _angle_inc;         // 360° = max_inc

  /*
   * check the scan size
   */
  if(max_inc > scan.size())
  {
    max_inc = scan.size();
  }

/*
 * clean mask -> change mask = "true" for 0.0 values to "false"
 */
  for(int j=0; j < scan.size(); j++)
  {
    if(scan[j] == 0.0)
    {
      mask_linepoints[j] = false;
    }
  }
  while(i < scan.size())
  {
    if((mask_linepoints[i]) && (scan[i] != 0.0))
    {
      first_1 = i;
      i = scan.size();
      found = true;
    }
    i++;
  }
  if(found)
  {
    // first_1 + 180° or first_1 - 180°
    if(first_1 <= max_inc/2)
    {
      i = first_1 + max_inc/2;                              

      first_2 = first_1;                                    
      while(i < max_inc)
      {
        if((mask_linepoints[i]) && (scan[i] != 0.0))
        {
          first_2 = i;
          i = max_inc;
        }
        i++;
      }
      i = first_1 + max_inc/2;                             

      while(i >= first_1)
      {
        if((mask_linepoints[i]) && (scan[i] != 0.0))
        {
          last_2 = i;
          i = first_1;
        }
        i--;
      }
    }
    else
    {
      first_2 = first_1;

      i = max_inc;                              
      while(i > first_1)
      {
        if((mask_linepoints[i]) && (scan[i] != 0.0))
        {
          last_2 = i;
          i = first_1;
        }
        i--;
      }
    }
    // write corner points
    corner_points[0].x = -scan[first_2] * sin(_angle_inc*first_2+_angle_diff/2);
    corner_points[0].y = scan[first_2] * cos(_angle_inc*first_2+_angle_diff/2);
    corner_points[1].x = -scan[last_2] * sin(_angle_inc*last_2+_angle_diff/2);
    corner_points[1].y = scan[last_2] * cos(_angle_inc*last_2+_angle_diff/2);

    // mask all points between corners as true
    if(first_2 < last_2)
    {
      for(int j=first_2; j < last_2; j++)
      {
        mask_linepoints[j] = true;
      }
    }
    else
    {
      for(int j=last_2; j < first_2; j++)
      {
        mask_linepoints[j] = true;
      }
    }
  }
  else
  {
    //ROS_INFO("Error in finding Mirror plane");
  }
}
void markMirrorArea_multi(std::vector<float> scan, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corner_points, std::vector<bool*>& mask_linepoints)
{

  if(corner_points.size() < line_points.size())
  {
    corner_points.resize(line_points.size());
  }

  vector<Point2f> tmp_corner_points(2);
  vector<Point2f> tmp_line_points(2);

  for(int i=0; i < (line_points.size()/2); i++)
  {
    tmp_line_points[0].x    = line_points[2*i].x;
    tmp_line_points[0].y    = line_points[2*i].y;
    tmp_line_points[1].x    = line_points[2*i+1].x;
    tmp_line_points[1].y    = line_points[2*i+1].y;

    markMirrorArea(scan, tmp_line_points, tmp_corner_points, mask_linepoints[i]);
    corner_points[2*i].x    = tmp_corner_points[0].x;
    corner_points[2*i].y    = tmp_corner_points[0].y;
    corner_points[2*i+1].x  = tmp_corner_points[1].x;
    corner_points[2*i+1].y  = tmp_corner_points[1].y;
  }
}


/*
 * other functions
 */
void filter_echos(void)
{
  _scan.ranges.resize(_scanSize);
  _scan.intensities.resize(_scanSize);
  _mirror.ranges.resize(_scanSize);
  _mirror.intensities.resize(_scanSize);
  _transp.ranges.resize(_scanSize);
  _transp.intensities.resize(_scanSize);
  _affected_transp.ranges.resize(_scanSize);
  _affected_transp.intensities.resize(_scanSize);
  _affected_mirror.ranges.resize(_scanSize);
  _affected_mirror.intensities.resize(_scanSize);
  _check.ranges.resize(_scanSize);
  _check.intensities.resize(_scanSize);
  _cloud.points.resize(_scanSize);

  bool linefit_success = false;
  bool subtract_result = false;

 /*
 * Subtract scans
 */
  subtract_result = subtractScans(_first_scan, _last_scan, _scan, _mirror, _affected_mirror, _substract_threshold_dist);
  if(subtract_result)    // Only if true there are discrepancies in the scans
  {
    /*
     * clean up mirror from single points
     */
      particleFilter1(_mirror.ranges, _mirror.intensities, _particlefilter_thres_dist_mirror, _particlefilter_threshold_angle);
      std::vector<bool*> mask_line_points(1);
      mask_line_points[0] = new bool[_scanSize];

      /*
      * find mirror line, get the corner points of the line and delete points at _mirror.ranges and _affected_mirror.ranges, which are not behind the mirror
      */
      linefit_success = findMirrorLine(_mirror.ranges, _mirror_line_points, mask_line_points, _ransac_threshold);
      if(linefit_success)
      {
        /*
         * check scan for more points on the mirror plane
         */
        if(_multiline)
        {
          sortingScanOnLineToMirror_multi(_scan.ranges, _scan.intensities, _affected_mirror.ranges, _affected_mirror.intensities, mask_line_points, _mirror_line_points, _ransac_threshold);
        }
        else
        {
          sortingScanOnLineToMirror(_scan.ranges, _scan.intensities, _affected_mirror.ranges, _affected_mirror.intensities, mask_line_points[0], _mirror_line_points, _ransac_threshold);
        }
        /*
         * check scan for remaining affected points
         * (points, which are in the vision cone and behind the mirror line)
         */
        if(_multiline)
        {
          sortingScanOnLineToAffected_multi(_scan.ranges, _scan.intensities, _affected_mirror.ranges, _affected_mirror.intensities, mask_line_points);
        }
        else
        {
          sortingScanOnLineToAffected(_scan.ranges, _scan.intensities, _affected_mirror.ranges, _affected_mirror.intensities, mask_line_points[0]);
        }
        /*
         * check affected points, if they are on the mirror. If so, move them to the mirror
         */
        if(_multiline)
        {
          sortingScanOnLineToMirror_multi(_affected_mirror.ranges, _affected_mirror.intensities, _affected_mirror.ranges, _affected_mirror.intensities, mask_line_points, _mirror_line_points, _ransac_threshold);
        }
        else
        {
          sortingScanOnLineToMirror(_affected_mirror.ranges, _affected_mirror.intensities, _affected_mirror.ranges, _affected_mirror.intensities, mask_line_points[0], _mirror_line_points, _ransac_threshold);
        }
        /*
         * clean up mirror, delete all Points which are not on the mirror line
         */
        if(_multiline)
        {
          mirrorVisionCone_multi(_mirror.ranges, _mirror.intensities, mask_line_points);
        }
        else
        {
          mirrorVisionCone(_mirror.ranges, _mirror.intensities, mask_line_points[0]);
        }
         /*
          * delete points, which are not behind mirror area
          */
        if(_multiline)
        {
          mirrorVisionCone_multi(_affected_mirror.ranges, _affected_mirror.intensities, mask_line_points);
        }
        else
        {
          mirrorVisionCone(_affected_mirror.ranges, _affected_mirror.intensities, mask_line_points[0]);
        }
        /*
         * clean up affected from single points
         */
         particleFilter(_affected_mirror.ranges, _affected_mirror.intensities, _particlefilter_thres_dist_affect);

         if(_switch_reflectiontype)
         {
           analyzeReflectionType(_mirror, _affected_mirror, _transp, _affected_transp, _substract_threshold_int);
         }
      }
      else
      {
        /*
         * clean up old data
         */
         cleanUpScan(_mirror.ranges);
         cleanUpScan(_affected_mirror.ranges);

         cleanUpScan(_mirror.intensities);
         cleanUpScan(_affected_mirror.intensities);
      }
  }
  else
  {
    //ROS_INFO("No substraction result");
  }
  publisherFunc();
}

bool subtractScans(sensor_msgs::LaserScan& first, sensor_msgs::LaserScan& last, sensor_msgs::LaserScan& good, sensor_msgs::LaserScan& mirror, sensor_msgs::LaserScan& affected, float threshold)
{
  bool result = 0;
  for (unsigned int i = 0; i < _scanSize; i++)
  {
    // both scans are equal => point is ok
    if(abs((last.ranges[i] - first.ranges[i])) <= threshold)
    {
      good.ranges[i]           = first.ranges[i];
      good.intensities[i]      = first.intensities[i];
      mirror.ranges[i]         = 0.0;
      mirror.intensities[i]    = 0.0;
      affected.ranges[i]       = 0.0;
      affected.intensities[i]  = 0.0;
    }
    // last_affected_mirror is behind _first_affected_mirror => point behind mirror or transparent area
    else if((last.ranges[i] - first.ranges[i]) > threshold)
    {
      good.ranges[i]           = 0.0;
      good.intensities[i]      = 0.0;
      mirror.ranges[i]         = first.ranges[i];
      mirror.intensities[i]    = first.intensities[i];
      affected.ranges[i]       = last.ranges[i];
      affected.intensities[i]  = last.intensities[i];

      result = 1;
    }
  }
  return result;
}

bool findMirrorLine(std::vector<float> scan, std::vector<cv::Point2f>& corner_line_points, std::vector<bool*>& mask_line_points, float threshold)
{
  vector<Point2f> tmp_scan(scan.size());
  vector<Point2f> tmp_line_points(2);
  bool ransac_success = false;

  /*
   * convert scan in xy
   */
  convertScan2xy(scan, tmp_scan, _angle_inc, _angle_diff);

  Point2f* point_1 = new Point2f[1];                            // good model point 1
  Point2f* point_2 = new Point2f[1];                            // good model point 2

  /*
   * ransac to find line
   */
  bool ransac_result;

  if(_multiline)
  {
    ransac_success = ransac2D_multi(tmp_scan, tmp_line_points, _ransac_points2fit, _ransac_iterations, threshold, mask_line_points);
  }
  else
  {
    //obvious::Time start_time = obvious::Time::now();
    ransac_success = ransac2D(tmp_scan, tmp_line_points, _ransac_points2fit, _ransac_iterations, threshold, mask_line_points[0]);
  }


  if(ransac_success)
  {
    /*
     * check for corner points of mirror and mask all points between as true
     */
    if(_multiline)
    {
      markMirrorArea_multi(scan, tmp_line_points, corner_line_points, mask_line_points);
    }
    else
    {
      markMirrorArea(scan, tmp_line_points, corner_line_points, mask_line_points[0]);
    }
    ransac_success = false;
    return 1;
  }
  else
  {
    return 0;
  }

  delete [] point_1;
  delete [] point_2;

}


void analyzeReflectionType(sensor_msgs::LaserScan& mirror, sensor_msgs::LaserScan& affected_mirror, sensor_msgs::LaserScan& transp, sensor_msgs::LaserScan& affected_transp, float thres_int)
{
  float tmp_diff = 0;
  float tmp_sum_aff = 0;
  float tmp_sum_mirror = 0;
  float tmp_div = 0;

  int amount_trans = 0;
  int amount_total = 0;
  bool transparent = 0;
  bool result1 = 0;

  /************************************************
  * division of median echo 1 and echo 2
  ************************************************/
  for (unsigned int i = 0; i < _scanSize; i++)
  {
    if((mirror.ranges[i] > 0.0) && (affected_mirror.ranges[i] > 0.0))
    {
      tmp_sum_aff += affected_mirror.intensities[i];
      tmp_sum_mirror += mirror.intensities[i];
    }
  }
  tmp_div = tmp_sum_aff / tmp_sum_mirror;
  if(tmp_div > 3)
  {
    transparent = true;
  }
  else
  {
    transparent = false;
  }
}

void sortingScanOnLineToMirror(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& mirror, std::vector<float>& m_intens, bool* mask_linepoints, std::vector<cv::Point2f> line_points, float threshold)
{
  for(unsigned int i = 0; i < scan.size(); i++)
  {
    std::vector<cv::Point2f> data(scan.size());
    convertScan2xy(scan, data, _angle_inc, _angle_diff);
    if(mask_linepoints[i] && (scan[i] != 0))
    {
      //calc distance to line
      float m = 0;
      float t = 0;
      cv::Point2f intersection;
      float distance = 0;
      bool lineValues = false;
      lineValues = calcLineValues(line_points, t, m);
      if(lineValues)
      {
       /*
        * shortest point to the mirror line
        */
       intersection.x = ((data[i].y + data[i].x / m) - t) / (m +1/m);
       intersection.y = m * intersection.x +t;
       distance = sqrt(pow((data[i].x - intersection.x),2.0) + pow((data[i].y - intersection.y),2.0));

       if(distance < threshold)
       {
         mirror[i] = scan[i];
         m_intens[i] = s_intens[i];
         scan[i] = 0.0;
         s_intens[i] = 0.0;
       }
      }

    }
  }


}
void sortingScanOnLineToMirror_multi(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& mirror, std::vector<float>& m_intens, std::vector<bool*>& mask_linepoints, std::vector<cv::Point2f> line_points, float threshold)
{
  vector<Point2f> tmp_line_points(2);
  for(int i=0; i < mask_linepoints.size(); i++)
  {
    tmp_line_points[0].x    = line_points[2*i].x;
    tmp_line_points[0].y    = line_points[2*i].y;
    tmp_line_points[1].x    = line_points[2*i+1].x;
    tmp_line_points[1].y    = line_points[2*i+1].y;
    sortingScanOnLineToMirror(scan, s_intens, mirror, m_intens, mask_linepoints[i], tmp_line_points, threshold);
  }
}

void sortingScanOnLineToAffected(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& affected, std::vector<float>& a_intens, bool* mask_linepoints)
{
  for(unsigned int i = 0; i < scan.size(); i++)
  {
    if(mask_linepoints[i] && (scan[i] != 0))
    {
      affected[i] = scan[i];
      a_intens[i] = s_intens[i];

      scan[i]     = 0.0;
      s_intens[i] = 0.0;
    }
  }
}
void sortingScanOnLineToAffected_multi(std::vector<float>& scan, std::vector<float>& s_intens, std::vector<float>& affected, std::vector<float>& a_intens, std::vector<bool*>& mask_linepoints)
{
  for(int i=0; i < mask_linepoints.size(); i++)
  {
    sortingScanOnLineToAffected(scan, s_intens, affected, a_intens, mask_linepoints[i]);
  }
}

void recalculateAffectedPoins(std::vector<float> scan, std::vector<float>& s_intens, std::vector<float>& recalculated_scan, std::vector<float>& rec_intens, std::vector<cv::Point2f> line_points)
{
  float m_mirror = 0.0;             // gradient mirror line
  float t_mirror = 0.0;             // y at x=0
  float m_inv = 0.0;                // gradient mirror line
  float t_inv = 0.0;
  float m_p_i = 0.0;
  float m_normal = 0.0;
  float t_normal = 0.0;

  float distance_testpoint = 0.0;

  Point2f intersection_point_i;
  intersection_point_i.x        = 0.0;
  intersection_point_i.y        = 0.0;

  Point2f normal_point_i;
  normal_point_i.x              = 0.0;
  normal_point_i.y              = 0.0;

  Point2f vek_ip_i;             // vektor intersection point and p_i
  vek_ip_i.x                    = 0.0;
  vek_ip_i.y                    = 0.0;

  Point2f vek_pn_i;             // vektor normal_point_i and p_i
  vek_pn_i.x                    = 0.0;
  vek_pn_i.y                    = 0.0;

  int sizeData = scan.size();

  /*
   * line for mirror: y = mx + t
   * m = (y2-y1)/(x2-x1)
   * t = y - mx
   */
  m_mirror = (line_points[1].y - line_points[0].y) / (line_points[1].x - line_points[0].x);
  t_mirror = line_points[1].y - m_mirror * line_points[1].x;
  m_normal = -1/m_mirror;

  /*
   * Convert from Laser::scan into xy
   */
  vector<Point2f> vek_p_i(scan.size());                       // Points p_i
  vector<Point2f> tmp_recalculated(scan.size());
  convertScan2xy(scan, vek_p_i, _angle_inc, _angle_diff);

  /*
   * recalculate affected points
   */
  for(int i=0; i < (sizeData); i++)
  {
    if((vek_p_i[i].x != 0.0) && (vek_p_i[i].y != 0.0))
    {
      /*
       * intersection point with mirror line => vek_ip_i
       *
       * m_p_i = vek_p_i[i].y / vek_p_i[i].x
       *
       * y_mirror = m_mirror * x + t_mirror
       * y_p_i    = m_p_i * x
       *
       * y_mirror = y_p_i
       * x  = t_mirror / (m_p_i - m_mirror)
       */
      m_p_i = vek_p_i[i].y / vek_p_i[i].x;

      intersection_point_i.x = t_mirror / (m_p_i - m_mirror);
      intersection_point_i.y = m_p_i * intersection_point_i.x;

      vek_ip_i.x = vek_p_i[i].x - intersection_point_i.x;
      vek_ip_i.y = vek_p_i[i].y - intersection_point_i.y;
      /*
       * vek_pn_i
       *
       * m_pn_i = -1/m_mirror
       * t_inv = data[i].y - m_inv * data[i].x
       *
       */
      t_normal = vek_p_i[i].y - m_normal * vek_p_i[i].x;

      normal_point_i.x = (t_normal - t_mirror) / (m_mirror - m_normal);
      normal_point_i.y = m_normal * normal_point_i.x + t_normal;

      vek_pn_i.x = vek_p_i[i].x - normal_point_i.x;
      vek_pn_i.y = vek_p_i[i].y - normal_point_i.y;



      /*
       * original point location
       */
      tmp_recalculated[i].x = vek_p_i[i].x - 2* vek_pn_i.x;
      tmp_recalculated[i].y = vek_p_i[i].y - 2* vek_pn_i.y;
    }
  }

  cleanUpScan(scan);

  /*
   * convert from xy into Laser::scan
   */
  convertxy2scan(tmp_recalculated, recalculated_scan, _angle_inc, _angle_diff);

  /*
   * copy intensities
   */
  for(int i = 0; i < sizeof(s_intens); i++)
  {
    rec_intens[i] = s_intens[i];
  }
}


/*
 * helping functions
 */
void convertScan2xy(std::vector<float> scan, std::vector<cv::Point2f> &data, float angle_increment, float scanangle)
{
  cv::Point2f tmp_point;

  for (unsigned int i = 0; i < (scan.size()); i++)
  {
    tmp_point.x = -scan[i] * sin(angle_increment*i + scanangle/2);
    tmp_point.y = scan[i] * cos(angle_increment*i + scanangle/2);
    data[i] = tmp_point;
  }
}
void convertxy2scan(std::vector<cv::Point2f> data, std::vector<float> &scan, float angle_increment, float scanangle)
{

  int j = 0;
  int max_inc = 2*M_PI / angle_increment;         // 360° = max_inc

  // resize scan to get 360°
  scan.resize(max_inc);
  for (unsigned int i = 0; i < (data.size()); i++)
  {
    if(data[i].x != 0.0)
    {
      // calculate new angle (Book 02/04/15)
      j = (atan(data[i].y / data[i].x) / angle_increment) - (scanangle/6/angle_increment);
      if(j > max_inc)
      {
        j =  j - max_inc;
      }
      else if(j < 0)
      {
        j = j + max_inc;
      }
      scan[j] = sqrt(pow((data[i].x),2.0) + pow((data[i].y),2.0));
    }
    else
    {
      j = max_inc - (scanangle/6/angle_increment);
      scan[j] = data[i].y;
    }
  }
}

void cleanUpScan(std::vector<float> &scan)
{
  for(int i = 0; i < scan.size(); i++)
  {
    scan[i] = 0.0;
  }
}

bool calcLineValues(std::vector<cv::Point2f> points, float& t, float& m)
{
  if((points[1].x - points[0].x) != 0)
  {
    m = (points[1].y - points[0].y) / (points[1].x - points[0].x);
    t = points[1].y - m * points[0].x;
    return 1;
  }
  else
  {
    cout << "Can't calculate Line!" << endl;
    return 0;
  }
}

/*
 * Init
 */
void init()
{
  // config marker
    _mirror_line_points.resize(2);
    // POINTS markers use x and y scale for width/height respectively
    _marker_points.scale.x = 0.05;
    _marker_points.scale.y = 0.05;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    _marker_line_strip.scale.x = 0.02;
    _marker_line_list.scale.x = 0.02;
    // Points are light blue
    _marker_points.color.r = 0.0;
    _marker_points.color.g = 1.0;
    _marker_points.color.b = 1.0;
    _marker_points.color.a = 1.0;
    // Line strip is red
    _marker_line_strip.color.r = 1.0;
    _marker_line_strip.color.g = 0.0;
    _marker_line_strip.color.b = 0.0;
    _marker_line_strip.color.a = 1.0;
    // Line list is red
    _marker_line_list.color.r = 1.0;
    _marker_line_list.color.g = 0.0;
    _marker_line_list.color.b = 0.0;
    _marker_line_list.color.a = 1.0;
}

int main(int argc, char **argv)
{
  double dVar = 0;
  ros::init(argc, argv, "ohm_echo_filter");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");

  // Parameters for launch file
  std::string sub_first;
  std::string sub_last;

  std::string pub_scan;
  std::string pub_mirror;
  std::string pub_transp;
  std::string pub_affected_transp;
  std::string pub_affected_mirror;
  std::string pub_maskScan;
  std::string pub_check;
  std::string pub_cloud;

  std::string pub_marker;

  nh_sub.param<std::string>("sub_scan_first",           sub_first,                 "first");
  nh_sub.param<std::string>("sub_scan_last",            sub_last,                  "last");

  nh_sub.param<double>("substract_threshold_dist",                dVar,         1.0);                       // allowed distance between first and last scan
  _substract_threshold_dist = static_cast<float>(dVar);
  nh_sub.param<double>("substract_threshold_int",                 dVar,         200);                       // to check for mirror or transparent objects
  _substract_threshold_int = static_cast<float>(dVar);
  nh_sub.param<double>("switchfactor_int",                        dVar,         0.6);                       // min percentage of transparent points
  _switchfactor_int = static_cast<float>(dVar);
  nh_sub.param<double>("switch_min_point",                        dVar,         10);                        // min amount of points to differ between mirror or transparent object
  _switch_min_point = static_cast<float>(dVar);
  nh_sub.param<double>("particlefilter_threshold_dist_mirror",     dVar,        0.05);                      // allowed distance between two neighbor points
  _particlefilter_thres_dist_mirror = static_cast<float>(dVar);
  nh_sub.param<double>("particlefilter_threshold_dist_affected",   dVar,        0.1);                       // allowed distance between two neighbor points
  _particlefilter_thres_dist_affect = static_cast<float>(dVar);
  nh_sub.param<double>("ransac_threshold",                          dVar,       0.05);                      // Parameter for RANSAC
  _ransac_threshold = static_cast<float>(dVar);

  nh_sub.param<int>("particlefilter_threshold_angle",   _particlefilter_threshold_angle,    10);            // max distance between two neighbor points
  nh_sub.param<bool>("switch_reflectiontype",           _switch_reflectiontype, false);                      // true: differentation between mirror and glas, false: every reflection is mirror
  nh_sub.param<bool>("multiline",                       _multiline, false);                                 // true: recognise multiple mirror lines

  nh_sub.param<int>("ransac_iterations",                _ransac_iterations,        100);
  nh_sub.param<int>("ransac_points2fit",                _ransac_points2fit,        15);
  nh_sub.param<int>("minMeasureDistance",               _minMeasureDistance,        23);
  nh_sub.param<int>("maxMeasureDistance",               _maxMeasureDistance,        60000);

  nh_pub.param<std::string>("pub_scan",                 pub_scan,                 "scan");                  // filtered scan
  nh_pub.param<std::string>("pub_mirror",               pub_mirror,               "mirror");                // points assigned to mirror plane
  nh_pub.param<std::string>("pub_transp",               pub_transp,               "transparent");           // points assigned to transparent plane
  nh_pub.param<std::string>("pub_affected_transp",      pub_affected_transp,      "affected transparent");  // points affected by transparent object
  nh_pub.param<std::string>("pub_affected_mirror",      pub_affected_mirror,      "affected mirror");       // points affected by mirror
  nh_pub.param<std::string>("pub_maskScan",             pub_maskScan,             "maskScan");              // scan masked with point types

  nh_pub.param<std::string>("pub_check",                pub_check,                "check");                 // publish for analyzing
  nh_pub.param<std::string>("pub_cloud",                pub_cloud,                "cloud");                 // publish for analyzing

  nh_pub.param<std::string>("pub_marker",               pub_marker,               "marker");                // maker to show in rviz

  // initialize subscriber
  ros::Subscriber sub_1   = nh_sub.subscribe(sub_first, 2, subscriberFunc_first);
  ros::Subscriber sub_2   = nh_sub.subscribe(sub_last, 2, subscriberFunc_last);

  // initialize publisher
  _pub_scan               = nh_pub.advertise<sensor_msgs::LaserScan>(pub_scan, 1);
  _pub_mirror             = nh_pub.advertise<sensor_msgs::LaserScan>(pub_mirror, 1);
  _pub_transp             = nh_pub.advertise<sensor_msgs::LaserScan>(pub_transp, 1);
  _pub_affected_transp    = nh_pub.advertise<sensor_msgs::LaserScan>(pub_affected_transp, 1);
  _pub_affected_mirror    = nh_pub.advertise<sensor_msgs::LaserScan>(pub_affected_mirror, 1);
  _pub_maskScan           = nh_pub.advertise<ohm_mirror_detector::ohm_maskLaser_msgs>(pub_maskScan, 1);

  _pub_check              = nh_pub.advertise<sensor_msgs::LaserScan>(pub_check, 1);
  _pub_cloud              = nh_pub.advertise<sensor_msgs::PointCloud>(pub_cloud, 1);

  _pub_marker             = nh_pub.advertise<visualization_msgs::Marker>(pub_marker, 1);

  init();

  ros::Rate r(50);

  while(ros::ok())
  {
    if(_new_dataset_first && _new_dataset_last)
    {
      filter_echos();
      _new_dataset_first = false;
      _new_dataset_last = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down");

  return 0;
}
