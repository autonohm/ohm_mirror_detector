/**
 * @file   ohm_mirror_postfilter.cpp
 * @author Rainer Koch
 * @date   07.09.2015
 *
 *
 */

#include "ohm_mirror_postfilter.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <ohm_mirror_detector/ohm_maskLaser_msgs.h>
#include <ohm_mirror_detector/ohm_poseLaser_msgs.h>

#include <visualization_msgs/Marker.h>

#include "rosfunctions.h"
#include "lineFunctions.h"

#include <float.h>
#include <cmath>
#include "ransac.h"
#include "reflectionIdentifier.h"

#include <sstream>

#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Time.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

// To write in file
#include <iostream>
#include <fstream>

// ROS variables
ros::Publisher _pub_scan;
ros::Publisher _pub_error;
ros::Publisher _pub_mirror;
ros::Publisher _pub_transparent;
ros::Publisher _pub_additional;

ros::Publisher _pub_marker;
ros::Publisher _pub_path;

tf::TransformListener* _transformListener = NULL;

sensor_msgs::LaserScan _scan;
nav_msgs::Path _path;
visualization_msgs::Marker _marker_points;
visualization_msgs::Marker _marker_line_strip;

int newContainerElements = 100;                                                     // amount of new elements
std::vector<obvious::SensorPolar2D*> _sensorHistory(newContainerElements);					// save history of echo 1
std::vector<obvious::SensorPolar2D*> _sensorHistory2(newContainerElements);         // save history of echo 2
std::vector<obvious::Matrix*> _mirrorHistory1(newContainerElements);							  // save history of all points assigned to mirror (in Real-World-Coordinates
std::vector<obvious::Matrix*> _mirrorHistory2(newContainerElements);                // save history of all points assigned to mirror (in Real-World-Coordinates
std::vector<double*> _mirrorHistoryIntens1(newContainerElements);                   // save history of all intensity 1 of points assigned to mirror
std::vector<double*> _mirrorHistoryIntens2(newContainerElements);                   // save history of all intensity 2 of points assigned to mirror

std::vector<obvious::Matrix*> _mirrorHistory_T(newContainerElements);						    // save the position of the Mirror
std::vector<obvious::Matrix*> _cornerHistory(newContainerElements);                 // save history of all corner points (in Real-World-Coordinates
int* _corner_object_type = new int[newContainerElements];                           // displays type of object of all checks

std::vector<unsigned int> _seq(newContainerElements);

// data
bool _new_mirror = false;
double _th_mirrorline = 0.05;                   // [m] maximal distance from the mirror line, if closer the point counted as a mirror point
double _th_new_corner = 0.2;                    // [m] minimal distance between two mirrors
double _th_angleThreshold = 5;                  // [°] additional threshold angle at the mirror sector, makes the sector lightly bigger
double _th_openingAnglePrefilter = 1;           // [°] minimal angle to check for mirror points at an incoming scan

double _th_MinPointsForICP = 100;               // minimal amount of points to check with ICP for reflection
double _th_maxDiffDistICPTrans = 0.1;           // [m] maximal difference in distance between Transformations to identify as the same reflection
double _th_maxDiffAngleICPTrans = 5;            // [°] maximal difference between in angle Transformations to identify as the same reflection

double _norm_white_intensity = 10000;           // value of intensity value of white paper at 1m

double _max_intensityValue = 60000;             // maximal intensity of scanner
double _th_phongSpreding = 0.1;                 // +-0.1 = +-10%, max. distance between two intensity values of points next to each other
double _th_phongFactor = 0.8;                   // [0.8 = 80%] switching factor to identify mirror or glass


int _ransac_iterations            = 100;        // max. number of iterations allowed in the ransac
float _ransac_threshold           = 0.05;       // threshold value for determining when a data point fits the model
int _ransac_points2fit            = 20;         // min. number of data values required to fit the model

double _angle_increment = 0;
double _angle_diff = 0;
double _maxRange = 30;
double _minRange = 0;
int _posecounter = 0;

double _lowReflectivityRange = 0;
double _scanangle = 0;
std::string _frame_id = "";

unsigned int _sensorDataCounter = 0;
unsigned int _mirrorHistoryCounter = 0;
unsigned int _mirrorCornerCounter = 0;

bool _publishOn = false;
bool _finalBacktrace = false;
bool _poseReceived = false;
bool _scanReceived = false;

#define _USE_MATH_DEFINES

using std::vector;
using namespace std;
using namespace cv;
using namespace obvious;

//To test
bool _only_one_time = 0;
int _tmp_count = 0;

void subscribermaskScan(const ohm_mirror_detector::ohm_maskLaser_msgs& maskScan)
{
  /*
   * get position of the scan from tf
   */
  tf::StampedTransform transform;
  ros::Time laser_timestamp = maskScan.header.stamp;
  bool gotTransformation = true;
  try
  {
    //wait till transform is avaiable
    _transformListener->waitForTransform("/map", "/laser", ros::Time::now(), ros::Duration(1.0));
    (*_transformListener).lookupTransform("/map", "/laser", laser_timestamp, transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    gotTransformation = false;
  }
  if(gotTransformation)
  {
    /*
     * copy maskScan msg into obvious sensor
     */
      /*
       * copy scan 1 & 2
       * scan 1 includes points, which are supposed to be valid, mirror or transparent points
       * scan 2 includes points, which are already identified as erroneous points
       */
      obvious::SensorPolar2D* sensor = new obvious::SensorPolar2D(maskScan.echo_1.ranges.size(), maskScan.echo_1.angle_increment, maskScan.echo_1.angle_min, _maxRange, _minRange, _lowReflectivityRange);
      obvious::SensorPolar2D* sensor2 = new obvious::SensorPolar2D(maskScan.echo_2.ranges.size(), maskScan.echo_2.angle_increment, maskScan.echo_2.angle_min, _maxRange, _minRange, _lowReflectivityRange);

      unsigned int size = maskScan.echo_1.ranges.size();

      _frame_id = maskScan.header.frame_id;
      _scanangle = maskScan.echo_1.angle_max - maskScan.echo_1.angle_min;
      _angle_diff = _scanangle;
      _seq[_sensorDataCounter] = maskScan.header.seq;
      _angle_increment = maskScan.echo_1.angle_increment;

      double* data    = new double[size];
      double* intens    = new double[size];
      double* data2     = new double[size];
      double* intens2    = new double[size];
      int* object_mask  = new int[size];
      for(int i = 0; i < size; i++)
      {
        data[i]         = maskScan.echo_1.ranges[i];
        intens[i]       = maskScan.echo_1.intensities[i];
        data2[i]        = maskScan.echo_2.ranges[i];
        intens2[i]      = maskScan.echo_2.intensities[i];

        object_mask[i]  = maskScan.object_mask[i];

        // check for mirror or transparent object
        if(object_mask[i] != 0)
        {
          _new_mirror = true;
        }
      }
      sensor->resetMask();
      sensor2->resetMask();

      sensor->setRealMeasurementData(data, 1.0);
      sensor2->setRealMeasurementData(data2, 1.0);

      sensor->setRealMeasurementTypeID(object_mask);
      sensor2->setRealMeasurementTypeID(object_mask);
      sensor->setRealMeasurementAccuracy(intens);
      sensor2->setRealMeasurementAccuracy(intens2);

      /*
       * calculate the tranformation matrix
       */
      obvious::Quaternion q(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
      obvious::Matrix R = q.Quaternion::convertToMatrix();
      obvious::Matrix Tr(3,3);
      obvious::Matrix T = sensor->getTransformation();
      Tr(0,0) = 1;
      Tr(0,2) = transform.getOrigin().getX();
      Tr(1,1) = 1;
      Tr(1,2) = transform.getOrigin().getY();
      Tr(2,2) = 1;
      T = Tr*R;

      sensor->setTransformation(T);
      sensor2->setTransformation(T);

      _sensorHistory[_sensorDataCounter] = sensor;
      _sensorHistory2[_sensorDataCounter] = sensor2;
      _sensorDataCounter++;

      /*
       * copy sensor into history
       */
      //resize, if necessary
      if(_sensorHistory.size() == _sensorDataCounter)
      {
        _sensorHistory.resize(_sensorDataCounter + newContainerElements);
        _sensorHistory2.resize(_sensorDataCounter + newContainerElements);
        _seq.resize(_sensorDataCounter + newContainerElements);
      }
  
      delete data;
      delete data2;
      delete object_mask;
  }
}

void subscriberActivatePub(std_msgs::Bool activate)
{
  ROS_INFO("Publisher activated!");
  _publishOn = true;
}

void publisherFuncScan(std::vector<obvious::SensorPolar2D*>& history, std::vector<obvious::SensorPolar2D*>& history2, std::vector<obvious::Matrix*>& cornerHistory, int historynr, int mirror_histCounter, int* object_type)
{
  int size = history[0]->getRealMeasurementSize();
  double* tmp_data;
  double* tmp_data2;
  double* tmp_dataMirrored  = new double[size];
  double* tmp_coordMirrored  = new double[2*size];
  bool* tmp_mask = new bool[size];

  double* tmp_intens;
  double* tmp_intens2;
  double* tmp_intensMirrored = new double[size];
  int* tmp_object_mask;
  int* tmp_object_mask2;
  bool* tmp_beenMoved = new bool[size];

  bool scan_finished = true;
  bool additional_object = true;
  ros::Time now = ros::Time::now();
  bool mirrored_object = false;

  /*
   * create msg for refined scan
   */
  sensor_msgs::LaserScan msgCleanedScan;
  msgCleanedScan.header.frame_id = _frame_id;
  msgCleanedScan.angle_min = history[0]->getPhiMin();
  msgCleanedScan.angle_max = -history[0]->getPhiMin();
  msgCleanedScan.angle_increment = _angle_increment;
  msgCleanedScan.range_min = history[0]->getMinimumRange();
  msgCleanedScan.range_max = history[0]->getMaximumRange();
  msgCleanedScan.ranges.resize(size);
  msgCleanedScan.intensities.resize(size);
  msgCleanedScan.header.stamp = now;
  msgCleanedScan.header.seq = _seq[historynr];

  /*
  * create msg for mirror scan
  */
  sensor_msgs::LaserScan msgMirror;
  msgMirror.ranges.resize(size);
  msgMirror.intensities.resize(size);

  /*
  * create msg for transparent scan
  */
  sensor_msgs::LaserScan msgTransparent;
  msgTransparent.ranges.resize(size);
  msgTransparent.intensities.resize(size);

  /*
  * create msg for error scan
  */
  sensor_msgs::LaserScan msgError;
  msgError.ranges.resize(size);
  msgError.intensities.resize(size);

  /*
  * create msg for additional scan points to update the map
  */
  ohm_mirror_detector::ohm_poseLaser_msgs msgAdditional;
  msgAdditional.object.ranges.resize(size);
  msgAdditional.object.intensities.resize(size);
  msgAdditional.object_mask.resize(size);

  /*
  * clean up scan history
  */
  obvious::SensorPolar2D* tmp_sensor;
  obvious::SensorPolar2D* tmp_sensor2;

  obvious::Matrix* corners_tmp = new obvious::Matrix(2, 2);

  tmp_sensor          = history[historynr];
  tmp_sensor2         = history[historynr];

  /*
   * get data to publish
   */
  tmp_data            = tmp_sensor->getRealMeasurementData();
  tmp_data2           = tmp_sensor2->getRealMeasurementData();
  tmp_intens          = tmp_sensor->getRealMeasurementAccuracy();
  tmp_intens2         = tmp_sensor2->getRealMeasurementAccuracy();

  /*
   * clean up object mask and replace mirrored points
   */
  for(int i = 0; i < mirror_histCounter; i++)
  {
    (*corners_tmp)(0,0) = (*cornerHistory[i])(0,0);
    (*corners_tmp)(0,1) = (*cornerHistory[i])(0,1);
    (*corners_tmp)(1,0) = (*cornerHistory[i])(1,0);
    (*corners_tmp)(1,1) = (*cornerHistory[i])(1,1);
    // For visualization

    geometry_msgs::Point p;
    p.x = (*corners_tmp)(0, 0);
    p.y = (*corners_tmp)(0, 1);
    p.z = 0.0;
    _marker_points.points.push_back(p);
    _marker_line_strip.points.push_back(p);
    p.x = (*corners_tmp)(1, 0);
    p.y = (*corners_tmp)(1, 1);
    p.z = 0.0;
    _marker_points.points.push_back(p);
    _marker_line_strip.points.push_back(p);
    pubPoints();
    pubLines();

    /*
     * clean up scans: assign reflective objects, reflected points and erroneous points
     */
    cleanScan((*corners_tmp), tmp_sensor, _th_mirrorline, _th_angleThreshold, object_type[i]);
    cleanScan((*corners_tmp), tmp_sensor2, _th_mirrorline, _th_angleThreshold, (object_type[i]+1));  // object_type +  1 = error of object_type


    /*
     * only if object is a mirror
     * calculate original location of mirrored points and move them
     */
    if(object_type[i] == 1)
    {
      for(int j = 0; j < size; j++)
      {
        tmp_intensMirrored[j]     = tmp_intens[j];
      }
      mirrored_object = replaceMirroredPoints((*corners_tmp), tmp_sensor, tmp_coordMirrored, object_type[i], tmp_beenMoved);
    }
  }

  /*
   * moved convert coords to distance
   */
  if(mirrored_object)
  {
    convertxy2RosSort(tmp_coordMirrored, tmp_dataMirrored, size, _angle_increment);
    for(unsigned int i = 0; i <= size; i++)
    {
      if(tmp_dataMirrored[i] != 0)
        msgError.ranges[i] = tmp_dataMirrored[i];
      else
        msgError.ranges[i] = INFINITY;
    }
  }

  /*
   * get cleaned object masked
   */
  tmp_object_mask     = tmp_sensor->getRealMeasurementTypeID();
  tmp_object_mask2    = tmp_sensor2->getRealMeasurementTypeID();

  /*
   * fill laser msgs
   */
     for(unsigned int i = 0; i <= size; i++)
     {
       msgAdditional.object_mask[i] = tmp_object_mask[i];
       switch(tmp_object_mask[i]){
         case 0: // scan
           msgCleanedScan.ranges[i]      = tmp_data[i];
           msgCleanedScan.intensities[i] = tmp_intens[i];

           msgMirror.ranges[i]           = INFINITY;
           msgMirror.intensities[i]      = INFINITY;

           msgTransparent.ranges[i]      = INFINITY;
           msgTransparent.intensities[i] = INFINITY;

           msgAdditional.object.ranges[i]      = INFINITY;
           msgAdditional.object.intensities[i] = INFINITY;
           break;
         case 1: // mirror
           msgCleanedScan.ranges[i]      = tmp_data[i];
           msgCleanedScan.intensities[i] = tmp_intens[i];

           msgMirror.ranges[i]           = tmp_data[i];
           msgMirror.intensities[i]      = tmp_intens[i];

           msgTransparent.ranges[i]      = INFINITY;
           msgTransparent.intensities[i] = INFINITY;

           msgAdditional.object.ranges[i]      = tmp_data[i];
           msgAdditional.object.intensities[i] = tmp_intens[i];
           additional_object = true;
           break;
         case 2: // error_mirror
           msgCleanedScan.ranges[i]      = INFINITY;
           msgCleanedScan.intensities[i] = INFINITY;

           msgMirror.ranges[i]           = INFINITY;
           msgMirror.intensities[i]      = INFINITY;

           msgTransparent.ranges[i]      = INFINITY;
           msgTransparent.intensities[i] = INFINITY;

           msgAdditional.object.ranges[i]      = INFINITY;
           msgAdditional.object.intensities[i] = INFINITY;
           additional_object = true;
           break;
         case 3: // transparent
           msgCleanedScan.ranges[i]      = tmp_data[i];
           msgCleanedScan.intensities[i] = tmp_intens[i];

           msgMirror.ranges[i]           = INFINITY;
           msgMirror.intensities[i]      = INFINITY;

           msgTransparent.ranges[i]      = tmp_data[i];
           msgTransparent.intensities[i] = tmp_intens[i];

           msgAdditional.object.ranges[i]      = tmp_data[i];
           msgAdditional.object.intensities[i] = tmp_intens[i];
           additional_object = true;
           break;
         case 4: // error_transparent
           msgCleanedScan.ranges[i]      = tmp_data[i];
           msgCleanedScan.intensities[i] = tmp_intens[i];

           msgMirror.ranges[i]           = INFINITY;
           msgMirror.intensities[i]      = INFINITY;

           msgTransparent.ranges[i]      = tmp_data[i];
           msgTransparent.intensities[i] = tmp_intens[i];

           msgAdditional.object.ranges[i]      = INFINITY;
           msgAdditional.object.intensities[i] = INFINITY;
           break;
         case 5: //  error_mirror recalculated
           msgCleanedScan.ranges[i]      = INFINITY;
           msgCleanedScan.intensities[i] = INFINITY;

           break;
         default:
           msgCleanedScan.ranges[i]      = tmp_dataMirrored[i];
           msgCleanedScan.intensities[i] = 0;

           msgMirror.ranges[i]      = INFINITY;
           msgMirror.intensities[i] = INFINITY;

           msgTransparent.ranges[i]      = INFINITY;
           msgTransparent.intensities[i] = INFINITY;

           msgAdditional.object.ranges[i]      = tmp_dataMirrored[i];
       }
     }

     if(additional_object)
     {
       /*
       * create msg for mirror scan
       */
       msgMirror.header.frame_id = _frame_id;
       msgMirror.angle_min = history[0]->getPhiMin();
       msgMirror.angle_max = -history[0]->getPhiMin();
       msgMirror.angle_increment = _angle_increment;
       msgMirror.range_min = history[0]->getMinimumRange();
       msgMirror.range_max = history[0]->getMaximumRange();
       msgMirror.header.stamp = now;
       msgMirror.header.seq = _seq[historynr];

       /*
       * create msg for transparent scan
       */
       msgTransparent.header.frame_id = _frame_id;
       msgTransparent.angle_min = history[0]->getPhiMin();
       msgTransparent.angle_max = -history[0]->getPhiMin();
       msgTransparent.angle_increment = _angle_increment;
       msgTransparent.range_min = history[0]->getMinimumRange();
       msgTransparent.range_max = history[0]->getMaximumRange();
       msgTransparent.header.stamp = now;
       msgTransparent.header.seq = _seq[historynr];

       /*
       * create msg for error scan
       */
       msgError.header.frame_id = _frame_id;
       msgError.angle_min = history[0]->getPhiMin();
       msgError.angle_max = -history[0]->getPhiMin();
       msgError.angle_increment = _angle_increment;
       msgError.range_min = history[0]->getMinimumRange();
       msgError.range_max = history[0]->getMaximumRange();
       msgError.header.stamp = now;
       msgError.header.seq = _seq[historynr];

       /*
       * create msg for additional scan points to update the map
       */
       // Set scan
       msgAdditional.header.frame_id = _frame_id;
       msgAdditional.object.angle_min = history[0]->getPhiMin();
       msgAdditional.object.angle_max = -history[0]->getPhiMin();
       msgAdditional.object.angle_increment = _angle_increment;
       msgAdditional.object.range_min = history[0]->getMinimumRange();
       msgAdditional.object.range_max = history[0]->getMaximumRange();
       msgAdditional.object.header.stamp = now;
       msgAdditional.object.header.seq = _seq[historynr];

       // get position
       obvious::Matrix T = tmp_sensor->getTransformation();

       msgAdditional.pose.position.x = (T)(0, 2);
       msgAdditional.pose.position.y = (T)(1, 2);
       msgAdditional.pose.position.z = 0;

       // calc rotation
       tf::Quaternion quat;
       double curTheta          = 0.0;
       const double ARCSIN   = asin((T)(1,0));
       const double ARCSINEG = asin((T)(0,1));
       const double ARCOS    = acos((T)(0,0));
       if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
         curTheta = ARCOS;
       else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
         curTheta = 2.0 * M_PI - ARCOS;

       quat.setEuler(0.0, 0.0, curTheta);
       msgAdditional.pose.orientation.x = quat.x();
       msgAdditional.pose.orientation.y = quat.y();
       msgAdditional.pose.orientation.z = quat.z();
       msgAdditional.pose.orientation.w = quat.w();

       // publish scans
      _pub_error.publish(msgError);
      _pub_mirror.publish(msgMirror);
      _pub_transparent.publish(msgTransparent);
      _pub_additional.publish(msgAdditional);
     }
     _pub_scan.publish(msgCleanedScan);

  delete corners_tmp;
  delete tmp_dataMirrored;
  delete tmp_coordMirrored;
  delete tmp_mask;
}

void pubPoints()
{
  _marker_points.header.frame_id = "map";
  _marker_points.header.stamp = ros::Time();
  _marker_points.ns = "points";
  _marker_points.action = visualization_msgs::Marker::MODIFY;
  _marker_points.pose.orientation.w = 1.0;
  _marker_points.type = visualization_msgs::Marker::POINTS;
  _marker_points.id = 0;
  _marker_points.lifetime = ros::Duration(0, 1);

  _pub_marker.publish(_marker_points);

}
void pubLines()
{
  _marker_line_strip.header.frame_id = "map";
  _marker_line_strip.header.stamp = ros::Time();
  _marker_line_strip.ns = "lines";
  _marker_line_strip.action = visualization_msgs::Marker::MODIFY;
  _marker_line_strip.pose.orientation.w = 1.0;
  _marker_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  _marker_line_strip.id = 1;

  _pub_marker.publish(_marker_line_strip);
}

void buildHistory(std::vector<obvious::SensorPolar2D*>& scan_history, std::vector<obvious::SensorPolar2D*>& scan_history2, std::vector<obvious::Matrix*>& mHistory, std::vector<obvious::Matrix*>& mHistory2, std::vector<
    obvious::Matrix*>& mHistory_T, unsigned int history_count, unsigned int& mirror_count, std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2)
{
  /*
   * get mirror data from last sensordata
   */
  unsigned int h_count = history_count - 1;

  int size = scan_history[0]->getRealMeasurementSize();

  double* data = new double[2 * size];
  double* data2 = new double[2 * size];

  double* intens1 = new double[size];
  double* intens2 = new double[size];
  double* scan;
  bool found_mirror = 0;
  int count_valid  = 0;
  int m = 0;

  bool* data_validmask = new bool[size];
  int* object_mask;

  obvious::Matrix* mirror_corners = new obvious::Matrix(2, 2);
  obvious::Matrix* mirror_matrix = new obvious::Matrix(size, 2);
  obvious::Matrix* mirror_matrix2 = new obvious::Matrix(size, 2);
  obvious::Matrix moved_mirror_matrix(size, 2);
  obvious::Matrix moved_mirror_matrix2(size, 2);
  obvious::Matrix* transform = new obvious::Matrix(3, 3);

  scan_history[h_count]->dataToCartesianVectorMask(data, data_validmask);

  scan_history2[h_count]->dataToCartesianVectorMask(data2, data_validmask);
  scan = scan_history[h_count]->getRealMeasurementData();
  intens1 = scan_history[h_count]->getRealMeasurementAccuracy();
  intens2 = scan_history2[h_count]->getRealMeasurementAccuracy();

  object_mask = scan_history[h_count]->getRealMeasurementTypeID();

  for(int i = 0; i < size; i++)
  {
    if((object_mask[i] == 1))
    {
      if((data[i * 2] != 0) && (data[i * 2 + 1] != 0))
      {
        (*mirror_matrix)(i, 0) = data[i * 2];
        (*mirror_matrix)(i, 1) = data[i * 2 + 1];
        (*mirror_matrix2)(i, 0) = data2[i * 2];
        (*mirror_matrix2)(i, 1) = data2[i * 2 + 1];
        object_mask[i] = 1;

        count_valid++;
        found_mirror = 1;
      }
      else
      {
        object_mask[i] = 0;
      }
    }
    else
    {
      (*mirror_matrix)(i, 0) = NAN;
      (*mirror_matrix)(i, 1) = NAN;
      (*mirror_matrix2)(i, 0) = NAN;
      (*mirror_matrix2)(i, 1) = NAN;
    }
  }

  obvious::Matrix T = scan_history[h_count]->getTransformation();
  obvious::Matrix* T_scan = &T;
  for(int i=0; i < 3; i++)
  {
    for(int j= 0; j < 3; j++)
    {
      (*transform)(i,j) = T(i,j);
    }
  }

  moved_mirror_matrix = mirror_matrix->createTransform(*T_scan);
  moved_mirror_matrix2 = mirror_matrix2->createTransform(*T_scan);

  /*
   * store all moved mirror points
   * in one "mirror_count" are all mirror points from one scan stored
   */
  if(found_mirror)
  {
    obvious::Matrix* tmp_points = new obvious::Matrix((count_valid+1), 2);
    obvious::Matrix* tmp_points2 = new obvious::Matrix((count_valid+1), 2);
    double* tmp_intens1 = new double[count_valid+1];
    double* tmp_intens2 = new double[count_valid+1];

    for(int i= 0; i < size-1; i++)
    {
       if((!(isnan(moved_mirror_matrix(i,0)))) && (object_mask[i]))
       {
         (*tmp_points)(m,0) = (moved_mirror_matrix)(i,0);
         (*tmp_points)(m,1) = (moved_mirror_matrix)(i,1);
         (*tmp_points2)(m,0) = (moved_mirror_matrix2)(i,0);
         (*tmp_points2)(m,1) = (moved_mirror_matrix2)(i,1);
         tmp_intens1[m] = intens1[i];
         tmp_intens2[m] = intens2[i];
         m++;
       }
    }

    mHistory[mirror_count] = tmp_points;
    mHistory2[mirror_count] = tmp_points2;
    mHistoryIntens1[mirror_count] = tmp_intens1;
    mHistoryIntens2[mirror_count] = tmp_intens2;
    mHistory_T[mirror_count] = transform;

    mirror_count++;
  }

  delete data;
  delete data_validmask;
  delete mirror_matrix;
  delete mirror_matrix2;
}

bool getCornerPoints(obvious::Matrix mirror_matrix, obvious::Matrix& corners, bool* mask)
{
  int index_left_corner = 0;
  float angle_left = 0;
  int index_right_corner = 0;
  float angle_right = 0;
  float* angle = new float[mirror_matrix.getRows()];
  float* distance = new float[mirror_matrix.getRows()];

  int first_1 = 0;
  int last_1 = 0;
  int first_2 = 0;
  int last_2 = 0;
  bool found = false;
  int max_inc = 2 * M_PI / 0.25;         // 360° = max_inc  (angle resolution 0.25)

  /*
   * check for left corner (first point in the scan)
   */
  for(int i = 0; i < mirror_matrix.getRows(); i++)
  {
    if((!(isnan((mirror_matrix)(i, 0)))) && mask[i])
    {
      index_left_corner = i;
      i = mirror_matrix.getRows();
    }
  }

  /*
   * check for right corner (last point in the scan)
   */
  for(int i = mirror_matrix.getRows(); i > 0; i--)
  {
    if((!(isnan((mirror_matrix)(i - 1, 0)))) && mask[i - 1])
    {
      index_right_corner = i - 1;
      i = 0;
    }
  }
  // write corners
  (corners)(0, 0) = (mirror_matrix)(index_left_corner, 0);
  (corners)(0, 1) = (mirror_matrix)(index_left_corner, 1);
  (corners)(1, 0) = (mirror_matrix)(index_right_corner, 0);
  (corners)(1, 1) = (mirror_matrix)(index_right_corner, 1);

  delete angle;
  delete distance;

  if(isnan(corners(0, 0)) or isnan(corners(1, 0)))
    return 0;

  return 1;
}
bool getCornerPoints2(std::vector<obvious::Matrix*>& mirror_pointsHistory, std::vector<obvious::Matrix*>& cornerHistory, unsigned int& counter_points, unsigned int& counter_corners, int ransac_points2fit, int ransac_iterations, double ransac_threshold, double th_mirrorline, int* objectIdentificationResult)
{
  /*
   * copy whole matrix to a temp. array
   */
  std::vector<cv::Point2f> tmp_line_points(2);
  std::vector<cv::Point2f> corner_line_points(2);
  double angle_min = 360.0;
  double angle_max = 0.0;
  bool ransac_success = false;
  bool newCorner = 0;

  /*
   * Check amount of points in mirror_pointsHistory
   * create a temporary list of points
   */
  int points = 0;
  for(int i=0; i < (counter_points); i++)
  {
    points += mirror_pointsHistory[i]->getRows();
  }
  double* data = new double[points,points];

  std::vector<cv::Point2f> tmp_points(points);            // includes all points of miror_pointsHistory as cv-point (x,y)
  vector<bool*> mask_line_points(1);
  mask_line_points[0] = new bool[points];
  double* angle = new double[points];
  int m= 0;

  /*
   * copy mirror_pointsHistory points int cv-tmp_points (x,y)
   */
  for(int i=0; i < counter_points; i++)
  {
    //get one line in mirror_pointsHistory
    mirror_pointsHistory[i]->getData(data);
    // get amount of points in line and copy to tmp_points
    for(int j=0; j < mirror_pointsHistory[i]->getRows(); j++)
    {
      if((data[i,2*j] != 0) or (data[i,2*j+1] != 0))
      {
        tmp_points[m].x = data[i,2*j];
        tmp_points[m].y = data[i,2*j+1];
        m++;
      }
    }
  }

  /*
   * Create RANSAC to search for multible points
   */
  ransac_success = ransac2D_multi(tmp_points, tmp_line_points, ransac_points2fit, ransac_iterations, ransac_threshold, mask_line_points);

  if(ransac_success)
  {
    /*
     * check if line_points are already in the corner list
     */
    for(int l = 0; l < (tmp_line_points.size()/2); l++)
    {
      obvious::Matrix* tmp_line_point = new obvious::Matrix(2,2);
      if(counter_corners == 0)
      {
        newCorner = 1;
      }
      else
      {
        // check if corners already exist
        bool* mask_new_line_points = new bool[tmp_line_points.size()/2];
        newCorner = 1;
        (*tmp_line_point)(0,0) =  tmp_line_points[2*l].x;
        (*tmp_line_point)(0,1) =  tmp_line_points[2*l].y;
        (*tmp_line_point)(1,0) =  tmp_line_points[2*l+1].x;
        (*tmp_line_point)(1,1) =  tmp_line_points[2*l+1].y;
        for(int j=0; j < counter_corners; j++)
        {
          if(!checkPointToUpdate(cornerHistory[j],*tmp_line_point, th_mirrorline))
          {
            newCorner = 0;
          }
        }
       }

      // if new corner, then check outer areas
      if(newCorner)
      {
        std::vector<cv::Point2f> tmp_corner(2);
        tmp_corner[1].x = tmp_line_points[2*l].x;
        tmp_corner[1].y = tmp_line_points[2*l].y;
        tmp_corner[0].x = tmp_line_points[2*l+1].x;
        tmp_corner[0].y = tmp_line_points[2*l+1].y;

        getOuterLinePoints2(tmp_points, tmp_corner, corner_line_points, mask_line_points[l], th_mirrorline);

        obvious::Matrix* new_corner = new obvious::Matrix(2,2);

        (*new_corner)(0,0) =  corner_line_points[0].x;
        (*new_corner)(0,1) =  corner_line_points[0].y;
        (*new_corner)(1,0) =  corner_line_points[1].x;
        (*new_corner)(1,1) =  corner_line_points[1].y;

         // add to history
         cornerHistory[counter_corners] = new_corner;
         objectIdentificationResult[counter_corners] = 3;        // Set as a mirror
         counter_corners++;
      }
    }
    return 1;
  }
  return 0;
}

void cleanScan(obvious::Matrix& corners, obvious::SensorPolar2D* sensor, double th_mirrorline, double th_angleThreshold, int object_type)
{
   /*
    * point values
    */
   float d_point = 0;
   float angle_point = 0;
   int size = sensor->getRealMeasurementSize();
   obvious::Matrix* sensor_position = new obvious::Matrix(3, 3);
   obvious::Matrix* data_matrix = new obvious::Matrix(size, 2);
   obvious::Matrix* moved_data_matrix = new obvious::Matrix(size, 2);
   double* data = new double[2 * size];
   int* object_mask;
   bool* data_validmask = new bool[size];

   /*
    * mirror_line values
    */
   float m_mirror = 0;
   float t_mirror = 0;
   bool lineValues = false;
   float d_corner_1 = 0;
   float d_corner_2 = 0;
   float angle_corner_1 = 0;
   float angle_corner_2 = 0;

   lineValues = calcLineValues(corners, t_mirror, m_mirror);

   /*
    * intersection line values
    */
   float m_intersection = 0;
   float t_intersection = 0;
   float d_intersection = 0;
   bool intersectionValues = false;
   int angleCase = true;      // check configuration of angle C1 and angle C2 to clean correctly

   obvious::Matrix* intersection = new obvious::Matrix(1, 2);                // intersection point between mirror_corner_line and line between sensor position and test point
   obvious::Matrix* intersectionLinePoints = new obvious::Matrix(2, 2);      // first point is position of sensor, second point is point to test
   obvious::Matrix* testpoint = new obvious::Matrix(1, 2);                // intersection point between mirror_corner_line and line between sensor position and test point
   obvious::Matrix* origin = new obvious::Matrix(1,2);                // intersection point between mirror_corner_line and line between sensor position and test point
   obvious::Matrix* corner_1 = new obvious::Matrix(1,2);                // intersection point between mirror_corner_line and line between sensor position and test point
   obvious::Matrix* corner_2 = new obvious::Matrix(1,2);                // intersection point between mirror_corner_line and line between sensor position and test point


   int* object_mask_1;
   int* object_mask_2;

   int tmp_count = 0;

   (*corner_1)(0, 0) = (corners)(0,0);
   (*corner_1)(0, 1) = (corners)(0,1);
   (*corner_2)(0, 0) = (corners)(1,0);
   (*corner_2)(0, 1) = (corners)(1,1);

   bool once_to_test = false;

   object_mask = sensor->getRealMeasurementTypeID();
   sensor->dataToCartesianVectorMask(data, data_validmask);
   for(int i = 0; i < size; i++)
   {
     (*data_matrix)(i, 0) = data[2 * i];
     (*data_matrix)(i, 1) = data[2 * i + 1];
   }

   // get position of scan and bring it into coordinate system "Map"
   *sensor_position = sensor->getTransformation();
   *moved_data_matrix = data_matrix->createTransform(*sensor_position);

   // points to create the intersection line
   // point 1 = sensor position
   // point 2 = data point i
   if(!((*sensor_position)(1,0) == (*sensor_position)(1,1)))
   {
     (*intersectionLinePoints)(0,0) = (*sensor_position)(0,2);
     (*intersectionLinePoints)(0,1) = (*sensor_position)(1,2);
     (*origin)(0, 0) = (*intersectionLinePoints)(0,0);
     (*origin)(0, 1) = (*intersectionLinePoints)(0,1);
   }
   else
   {
     cout << __PRETTY_FUNCTION__ << "cleanScan: intersectionLinePoints division error" << endl;
     return;
   }

   d_corner_1 = sqrt(pow(((corners)(0,0) - (*intersectionLinePoints)(0,0)), 2.0) + pow(((corners)(0,1) - (*intersectionLinePoints)(0,1)), 2.0));
   d_corner_2 = sqrt(pow(((corners)(1,0) - (*intersectionLinePoints)(0, 0)), 2.0) + pow(((corners)(1,1) - (*intersectionLinePoints)(0,1)), 2.0));

   if((d_corner_1 != 0) && (d_corner_2 != 0))
   {
     if(!((corners)(0,1) == (*intersectionLinePoints)(0,1))
         and !((corners)(1,1) == (*intersectionLinePoints)(0,1)))
     {
       angle_corner_1 = getAngle(*origin, *corner_1);
       angle_corner_2 = getAngle(*origin, *corner_2);
       if((angle_corner_2 > angle_corner_1) and ((angle_corner_2 - angle_corner_1) < 180))
       {
         angleCase = 0;
       }
       else if((angle_corner_2 > angle_corner_1) and ((angle_corner_2 - angle_corner_1) > 180))
       {
         angleCase = 1;
       }
       if((angle_corner_2 < angle_corner_1) and ((angle_corner_1 - angle_corner_2) < 180))
       {
         angleCase = 2;
       }
       if((angle_corner_2 < angle_corner_1) and ((angle_corner_1 - angle_corner_2) > 180))
       {
         angleCase = 3;
       }

     }
     else
     {
       cout << __PRETTY_FUNCTION__ << "cleanScan: angle corners division error" << endl;
       return;
     }
     if((lineValues) && (abs(angle_corner_2 - angle_corner_1) >= th_angleThreshold))
     {
       for(int i = 0; i < size; i++)
       {
         /*
          * check only, if point is not identified as affected
          */
         if(object_mask[i] == 0)
         {
           /*
            * intersection point with point to the mirror line (book 20./21.04.15)
            */
           // create intersection point of mirror line and line between map point and path point
           (*intersectionLinePoints)(1, 0) = (*moved_data_matrix)(i, 0);
           (*intersectionLinePoints)(1, 1) = (*moved_data_matrix)(i, 1);
           (*testpoint)(0, 0) = (*moved_data_matrix)(i, 0);
           (*testpoint)(0, 1) = (*moved_data_matrix)(i, 1);

           intersectionValues = calcLineValues(*intersectionLinePoints, t_intersection, m_intersection);
           //Get intersection point between line_originToPoint and line_mirrorCorners
           if(!(m_mirror == m_intersection))
           {
             (*intersection)(0, 0) = (t_intersection - t_mirror) / (m_mirror - m_intersection);
             (*intersection)(0, 1) = m_intersection * ((*intersection)(0, 0)) + t_intersection;
           }
           else
           {
             cout << __PRETTY_FUNCTION__ << "cleanScan: intersection division error" << endl;
             return;
           }
           /*
            * Check 1: distance intersection < distance point => point behind mirror line
            *
            * d_intersection = distance between sensor position and intersection point
            * d_point        = distance between sensor position and point[i]
            */
           //distance between sensor position and point to check
           d_point = sqrt(pow(((*intersectionLinePoints)(1, 0) - (*intersectionLinePoints)(0, 0)), 2.0) + pow(((*intersectionLinePoints)(1, 1) - (*intersectionLinePoints)(0, 1)), 2.0));
           d_intersection = sqrt(pow(((*intersection)(0, 0) - (*intersectionLinePoints)(0, 0)), 2.0) + pow(((*intersection)(0, 1) - (*intersectionLinePoints)(0, 1)), 2.0));

           if(d_point != 0)
           {
             if(!((*intersectionLinePoints)(1, 1) == (*intersectionLinePoints)(0, 1)))
             {
               angle_point = getAngle(*origin, *testpoint);

                 switch(angleCase){
                   case 0:
                     if((angle_point < (angle_corner_2 + th_angleThreshold)) and (angle_point > (angle_corner_1 - th_angleThreshold)))
                     {
                       //Check direction of vector to point and vector to mirror
                       if(checkDirection(origin, testpoint, intersection))
                       {
                         if(d_point > (d_intersection - th_mirrorline))
                         {
                           if(d_point < (d_intersection + th_mirrorline))
                           {
                             object_mask[i] = object_type; // mirror
                           }
                           else
                           {
                             object_mask[i] = object_type+1; // error
                           }
                         }
                       }
                     }
                     break;
                   case 1:
                     if((angle_point > (angle_corner_2 - th_angleThreshold)) or (angle_point < (angle_corner_1 + th_angleThreshold)))
                     {
                       //Check direction of vector to point and vector to mirror
                       if(checkDirection(origin, testpoint, intersection))
                       {
                         if(d_point > (d_intersection - th_mirrorline))
                         {
                           if(d_point < (d_intersection + th_mirrorline))
                           {
                             object_mask[i] = object_type; // mirror
                           }
                           else
                           {
                             object_mask[i] = object_type+1; // error
                           }
                         }
                       }
                     }
                   break;
                   case 2:
                     if((angle_point > (angle_corner_2 - th_angleThreshold)) and (angle_point < (angle_corner_1 + th_angleThreshold)))
                     {
                       //Check direction of vector to point and vector to mirror
                       if(checkDirection(origin, testpoint, intersection))
                       {
                         if(d_point > (d_intersection - th_mirrorline))
                         {
                           if(d_point < (d_intersection + th_mirrorline))
                           {
                             object_mask[i] = object_type; // mirror
                           }
                           else
                           {
                             object_mask[i] = object_type+1; // error
                           }
                         }
                       }
                     }
                     break;
                   case 3:
                     if((angle_point < (angle_corner_2 - th_angleThreshold)) or (angle_point > (angle_corner_1 + th_angleThreshold)))
                     {
                       //Check direction of vector to point and vector to mirror
                       if(checkDirection(origin, testpoint, intersection))
                       {
                         if(d_point > (d_intersection - th_mirrorline))
                         {
                           if(d_point < (d_intersection + th_mirrorline))
                           {
                             object_mask[i] = object_type; // mirror
                           }
                           else
                           {
                             object_mask[i] = object_type+1; // error
                           }
                         }
                       }
                     }
                     break;
                 }
             }
             else
             {
               cout << __PRETTY_FUNCTION__ << "cleanScan: angle_point division error" << endl;
               return;
             }
           }
         }
       }
       sensor->setRealMeasurementTypeID(object_mask);
     }
   }

  delete sensor_position;
  delete data_matrix;
  delete moved_data_matrix;
  delete data;
  delete data_validmask;
  delete intersection;
  delete intersectionLinePoints;
  delete testpoint;
  delete origin;
  delete corner_1;
  delete corner_2;
}

bool checkMirrorHistory(obvious::Matrix& corner_points, obvious::Matrix& T_corner_points, std::vector<obvious::Matrix*>& history, std::vector<
    obvious::Matrix*>& T_history, double th_new_mirror, double th_mirrorline, unsigned int& historysize)
{
  /*
   * check for points at origin or with NAN
   */
  if(((corner_points)(0, 0) == 0) && ((corner_points)(0, 1) == 0))
    return 0;
  if(((corner_points)(1, 0) == 0) && ((corner_points)(1, 1) == 0))
    return 0;
  if(isnan((corner_points)(1, 0)) && isnan((corner_points)(1, 1)))
    return 0;
  if(isnan((corner_points)(1, 0)) && isnan((corner_points)(1, 1)))
    return 0;

  // check history for similar mirror
  double distance_r1 = 0.0;
  double distance_r2 = 0.0;
  bool foundBetterCorner = 0;
  bool newCorner = 0;
  bool lineValues = 0;
  obvious::Matrix hist_tmp(2, 2);
  bool getNewMirror = false;

  for(int i = 0; i < historysize; i++)
  {
    //cout << "check history -> " << i << endl;
    // if reach last point in history stop searching
    if(((*history[i])(0, 0) != 0) && ((*history[i])(0, 0) != 0))
    {
      distance_r1 = sqrt(pow((corner_points(0, 0) - (*history[i])(0, 0)), 2.0)
          + pow((corner_points(0, 1) - (*history[i])(0, 1)), 2.0));
      distance_r2 = sqrt(pow((corner_points(1, 0) - (*history[i])(1, 0)), 2.0)
          + pow((corner_points(1, 1) - (*history[i])(1, 1)), 2.0));

      if((distance_r1 > th_new_mirror) and (distance_r2 > th_new_mirror))
      {
        // points are to far of history points away, if not found later => add a new Mirror;
        cout << "New corners to far from history" << endl;
        getNewMirror = true;
      }
      else
      {
        // check if mirror corners need update and update
        hist_tmp = *history[i];
        foundBetterCorner = checkPointToUpdate(&hist_tmp, corner_points, th_mirrorline);

        if(foundBetterCorner)
        {
          *history[i] = hist_tmp;

          getNewMirror = false;
          i = historysize;
        }
      }
    }
    else
    {
      // end search, because end of history;
      getNewMirror = true;
      i = historysize;
    }
  }

  if(getNewMirror)
  {
    // new mirror
    cout << "Write a new mirror to the history" << endl;

    history[historysize] = &corner_points;
    T_history[historysize] = &T_corner_points;
    historysize++;

    newCorner = 1;
  }

  if(foundBetterCorner or newCorner)
    return true;
  else
    return false;
}


/*
 * Init
 */
void init()
{
  // config marker
  // POINTS markers use x and y scale for width/height respectively
  _marker_points.scale.x = 0.1;
  _marker_points.scale.y = 0.1;
  // Points are light blue
  _marker_points.color.r = 0.0;
  _marker_points.color.g = 1.0;
  _marker_points.color.b = 1.0;
  _marker_points.color.a = 1.0;
  // Line strip is red
  _marker_line_strip.scale.x = 0.02;
  _marker_line_strip.color.r = 1.0;
  _marker_line_strip.color.g = 0.0;
  _marker_line_strip.color.b = 0.0;
  _marker_line_strip.color.a = 1.0;

  ROS_INFO("Init mirror postfilter");

}

int main(int argc, char **argv)
{
  unsigned int historynr = 0;

  ros::init(argc, argv, "ohm_mirror_postfilter");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");

  // Parameters for launch file
  std::string sub_sensor;
  std::string sub_maskScan;
  //std::string sub_pose;

  std::string sub_activatePub;

  std::string pub_scan;
  std::string pub_error;
  std::string pub_mirror;
  std::string pub_transparent;
  std::string pub_additional;

  std::string pub_marker;
  double dVar = 0;

  nh_sub.param < std::string > ("sub_maskScan", sub_maskScan, "maskScan");
  nh_sub.param < std::string > ("sub_activatePub", sub_activatePub, "activatePub");

  nh_pub.param < std::string > ("pub_scan", pub_scan, "scan_corrected");
  nh_pub.param < std::string > ("pub_error", pub_error, "scan_error");
  nh_pub.param < std::string > ("pub_mirror", pub_mirror, "mirror_empty");
  nh_pub.param < std::string > ("pub_transparent", pub_transparent, "mirror_transparent");
  nh_pub.param < std::string > ("pub_additionalPoints", pub_additional, "additional_scan_points");

  nh_pub.param < std::string > ("pub_marker", pub_marker, "marker");              				 // maker to show in rviz

  nh_pub.param<double>("thres_mirrorcorner", dVar, 0.5);                      // [m] minimal distance between two mirrors
  _th_new_corner = static_cast<float>(dVar);
  nh_pub.param<double>("thres_mirrorline", dVar, 0.1);                        // [m] maximal distance from the mirror line, if closer the point counted as a mirror point
  _th_mirrorline = static_cast<float>(dVar);
  nh_pub.param<double>("thres_openingAnglePrefilter", dVar, 1);            // [°] additional threshold angle at the mirror sector, makes the sector lightly bigger
  _th_openingAnglePrefilter = static_cast<float>(dVar);
   nh_pub.param<double>("thres_angleThreshold", dVar, 5);                     // [°] minimal angle to check for mirror points at an incoming scan
  _th_angleThreshold = static_cast<float>(dVar);

  // Variables for icp
  nh_pub.param<double>("thres_MinPointsICP", dVar, 100);                     // minimal amount of points to check with ICP for reflection
  _th_MinPointsForICP = static_cast<float>(dVar);
  nh_pub.param<double>("thres_MaxDistICPTrans", dVar, 0.1);                  // [m] maximal difference in distance between Transformations to identify as the same reflection
  _th_maxDiffDistICPTrans = static_cast<float>(dVar);
  nh_pub.param<double>("thres_MaxAngleICPTrans", dVar, 5);                   // [°] maximal difference between in angle Transformations to identify as the same reflection
  _th_maxDiffAngleICPTrans = static_cast<float>(dVar);

  // Variables for normalized intensity
  nh_pub.param<double>("normed_white_Intensity", dVar, 10000);                   // value of intensity value of white paper at 1m
  _norm_white_intensity = static_cast<float>(dVar);

  // Variables for Phong curve
  nh_pub.param<double>("max_Intensity", dVar, 60000);                     // maximal intensity Value
  _max_intensityValue = static_cast<float>(dVar);
  nh_pub.param<double>("thres_phongSpreding", dVar, 0.1);                 // +-0.1 = +-10%, max. distance between two intensity values of points next to each other
  _th_phongSpreding = static_cast<float>(dVar);
  nh_pub.param<double>("thres_phongFactor", dVar, 0.8);                   // [0.8 = 80%] switching factor to identify mirror or glass
  _th_phongFactor = static_cast<float>(dVar);


  nh_pub.param<double>("min_range", dVar, 0.01);                        // [m]
  _minRange = static_cast<float>(dVar);
  nh_pub.param<double>("max_range", dVar, 30);                          // [m]
  _maxRange = static_cast<float>(dVar);
   nh_pub.param<double>("low_reflectivity_range", dVar, 2.0);           //
  _lowReflectivityRange = static_cast<float>(dVar);

  nh_sub.param<double>("ransac_threshold",                          dVar,       0.05);                      // Parameter for RANSAC
  _ransac_threshold = static_cast<float>(dVar);
  nh_sub.param<int>("ransac_iterations",                _ransac_iterations,        100);
  nh_sub.param<int>("ransac_points2fit",                _ransac_points2fit,        20);

  // initialize subscriber
  ros::Subscriber sub1 = nh_sub.subscribe(sub_maskScan, 2, subscribermaskScan);
  ros::Subscriber sub3 = nh_sub.subscribe(sub_activatePub, 2, subscriberActivatePub);

  // initialize publisher
  _pub_scan = nh_pub.advertise<sensor_msgs::LaserScan>(pub_scan, 1);
  _pub_error = nh_pub.advertise<sensor_msgs::LaserScan>(pub_error, 1);
  _pub_mirror = nh_pub.advertise<sensor_msgs::LaserScan>(pub_mirror, 1);
  _pub_transparent = nh_pub.advertise<sensor_msgs::LaserScan>(pub_transparent, 1);
  _pub_marker = nh_pub.advertise<visualization_msgs::Marker>(pub_marker, 1);
  _pub_additional = nh_pub.advertise<ohm_mirror_detector::ohm_poseLaser_msgs>(pub_additional, 1);

  _transformListener = new (tf::TransformListener);

  init();

  bool corner_points = false;
  bool checkNewCorners = false;
  ros::Rate r(50);

  while(ros::ok())
  {
    /*
     * copy new mirror data into the mirror history
     */
    if(_new_mirror)
    {
       buildHistory(_sensorHistory, _sensorHistory2, _mirrorHistory1, _mirrorHistory2, _mirrorHistory_T, _sensorDataCounter, _mirrorHistoryCounter, _mirrorHistoryIntens1, _mirrorHistoryIntens2);
       checkNewCorners = true;
    }

    if(_publishOn)
    {
      /*
       * check for mirror corners, if
       * - no corner_points
       * - new mirror points appeared after the corners already had calculated
       */
      if((!corner_points) && checkNewCorners)
      {
        corner_points = getCornerPoints2(_mirrorHistory1, _cornerHistory, _mirrorHistoryCounter, _mirrorCornerCounter, _ransac_points2fit, _ransac_iterations, _ransac_threshold, _th_mirrorline, _corner_object_type);

        identifyReflectionType(_sensorHistory, _sensorHistory2, _sensorDataCounter, _mirrorHistoryIntens1, _mirrorHistoryIntens2, _mirrorHistory1, _mirrorHistory2, _mirrorHistoryCounter, _mirrorHistory_T, _cornerHistory, _mirrorCornerCounter, _th_mirrorline, _th_angleThreshold, _th_MinPointsForICP, _th_maxDiffDistICPTrans, _th_maxDiffAngleICPTrans, _norm_white_intensity, _max_intensityValue, _th_phongSpreding, _th_phongFactor, _corner_object_type);

        checkNewCorners = false;
      }

      /*
       * publish cleaned scans
       */
      if(historynr < _sensorDataCounter)
      {
        /*
         * if publisherFuncScan = 1 => The scan has been send.
         * if publisherFuncScan = 0 => There was a mirror or transparent object -> Have to send the second part of the scan.
         */
        publisherFuncScan(_sensorHistory, _sensorHistory2, _cornerHistory, historynr, _mirrorCornerCounter, _corner_object_type);
        historynr++;
      }
    }

    if(_mirrorHistory1.size() == _mirrorHistoryCounter)
    {
      _mirrorHistory1.resize(_mirrorHistoryCounter + newContainerElements);
      _mirrorHistory2.resize(_mirrorHistoryCounter + newContainerElements);
      _mirrorHistory_T.resize(_mirrorHistoryCounter + newContainerElements);
      _mirrorHistoryIntens1.resize(_mirrorHistoryCounter + newContainerElements);
      _mirrorHistoryIntens2.resize(_mirrorHistoryCounter + newContainerElements);
      _seq.resize(_mirrorHistoryCounter + newContainerElements);
    }
    _new_mirror = false;

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down");

  return 0;
}
