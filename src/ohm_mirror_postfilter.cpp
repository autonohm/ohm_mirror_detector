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
#include <ohm_mirror_detector/ohm_maskLaser_msgs.h>


#include <visualization_msgs/Marker.h>

#include "rosfunctions.h"

#include <float.h>
#include <cmath>

#include <sstream>

#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Time.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

// ROS variables
ros::Publisher _pub_scan;
ros::Publisher _pub_error;
ros::Publisher _pub_marker;
ros::Publisher _pub_path;

sensor_msgs::LaserScan _scan;
nav_msgs::Path _path;
visualization_msgs::Marker _marker_points;
visualization_msgs::Marker _marker_line_strip;

int newContainerElements = 1000;                                     // amount of new elements
std::vector<obvious::SensorPolar2D*> _sensorHistory(newContainerElements);					// save history of echo 1
std::vector<obvious::SensorPolar2D*> _sensorHistory2(newContainerElements);         // save history of echo 2
std::vector<obvious::Matrix*> _mirrorHistory(newContainerElements);							    // save mirror corners (left corner, right corner)
std::vector<obvious::Matrix*> _mirrorHistory_T(newContainerElements);						    // save the position of the Mirror

std::vector<unsigned int> _seq(newContainerElements);

// data
bool _new_mirror = false;
double _th_mirrorline = 0.05;                   // [m] maximal distance from the mirror line, if closer the point counted as a mirror point
double _th_new_corner = 0.2;                    // [m] minimal distance between two mirrors
double _th_angleThreshold = 5;                  // [°] additional threshold angle at the mirror sector, makes the sector lightly bigger
double _th_openingAnglePrefilter = 1;           // [°] minimal angle to check for mirror points at an incoming scan

double _angle_increment = 0;
double _maxRange = 30;
double _minRange = 0;
int _posecounter = 0;

double _lowReflectivityRange = 0;
double _scanangle = 0;
std::string _frame_id = "";

unsigned int _sensorDataCounter = 0;
unsigned int _mirrorHistoryCounter = 0;
bool _publishOn = false;
bool _finalBacktrace = false;
bool _poseReceived = false;
bool _scanReceived = false;

#define _USE_MATH_DEFINES

using std::vector;
using namespace std;
using namespace cv;
using namespace obvious;

void subscriberPose(const geometry_msgs::PoseStamped& pose)
{
 // cout << "Pose in: " << _sensorDataCounter << endl;
  _path.poses.resize(_sensorDataCounter+1);

  _path.header.seq          = pose.header.seq;
  _path.header.stamp        = pose.header.stamp;
  _path.header.frame_id     = pose.header.frame_id;

  _path.poses[_sensorDataCounter].header.seq          = pose.header.seq;
  _path.poses[_sensorDataCounter].header.stamp        = pose.header.stamp;
  _path.poses[_sensorDataCounter].header.frame_id     = pose.header.frame_id;

  _path.poses[_sensorDataCounter].pose.position.x     = pose.pose.position.x;
  _path.poses[_sensorDataCounter].pose.position.y     = pose.pose.position.y;
  _path.poses[_sensorDataCounter].pose.position.z     = pose.pose.position.z;

  _path.poses[_sensorDataCounter].pose.orientation.x  = pose.pose.orientation.x;
  _path.poses[_sensorDataCounter].pose.orientation.y  = pose.pose.orientation.y;
  _path.poses[_sensorDataCounter].pose.orientation.z  = pose.pose.orientation.z;
  _path.poses[_sensorDataCounter].pose.orientation.w  = pose.pose.orientation.w;
//  cout << "Nr: " << pose.pose.position.x  << "/" << pose.pose.position.y << "/" << pose.pose.position.z << endl;
//  cout << "Nr: " << pose.pose.orientation.x << "/" << pose.pose.orientation.y << "/" << pose.pose.orientation.z << "/" << pose.pose.orientation.w << endl;
//  cout << "Nr: " << _poseStamped.size() << "_" << _posecounter << ": " <<_poseStamped[_posecounter].pose.position.x  << "/" << _poseStamped[_posecounter].pose.position.y << "/" << _poseStamped[_posecounter].pose.position.z << endl;

  /*
   * calculate the tranformation matrix
   */
  obvious::Quaternion q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  obvious::Matrix R = q.Quaternion::convertToMatrix();
  obvious::Matrix Tr(3,3);
  obvious::Matrix T = _sensorHistory[_sensorDataCounter]->getTransformation();
  Tr(0,0) = 1;
  Tr(0,2) = pose.pose.position.x;
  Tr(1,1) = 1;
  Tr(1,2) = pose.pose.position.y;
  Tr(2,2) = 1;
  T = Tr*R;

//  cout << "Mirror-Detector - Postfilter pose in" << _posecounter << endl;
//  cout << pose.pose.position.x << "/" << pose.pose.position.y << "/" << pose.pose.position.z << endl;
//  cout << pose.pose.orientation.x << "/" << pose.pose.orientation.y << "/" << pose.pose.orientation.z << "/" << pose.pose.orientation.w << endl;

 _sensorHistory[_sensorDataCounter]->setTransformation(T);
 _sensorHistory2[_sensorDataCounter]->setTransformation(T);
 _poseReceived = true;
}

void subscribermaskScan(const ohm_mirror_detector::ohm_maskLaser_msgs& maskScan)
{
  /*
   * copy maskScan msg into obvious sensor
   */
  if(_poseReceived)
  {
    _sensorDataCounter++;
    _poseReceived = false;
    _scanReceived = false;
  }
  // take first occuring scan. Ignore all, till a pose received. Then first occuring scan and ignore again all scans till new pose is received
  if(!_scanReceived)
  {
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
    _seq[_sensorDataCounter] = maskScan.header.seq;
    _angle_increment = maskScan.echo_1.angle_increment;

    double* data = new double[size];
    double* data2 = new double[size];
    int* object_mask = new int[size];

    for(int i = 0; i < size; i++)
    {
      data[i] = maskScan.echo_1.ranges[i];
      data2[i] = maskScan.echo_2.ranges[i];
      object_mask[i] = maskScan.object_mask[i];

      // check for mirror
      if(object_mask[i] == 2)
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
    _sensorHistory[_sensorDataCounter] = sensor;
    _sensorHistory2[_sensorDataCounter] = sensor2;

    _scanReceived = true;

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

void publisherFuncScan(std::vector<obvious::SensorPolar2D*> history, std::vector<obvious::SensorPolar2D*> history2, int historynr)
{
  //ROS_INFO("Publish cleaned history");

  int size = history[0]->getRealMeasurementSize();
  double* tmp_data;
  double* tmp_data2;
  int* tmp_object_mask;
  int* tmp_object_mask2;
  int* tmp_object_mask_test;

  /*
   * create msg for refined scan
   */
  sensor_msgs::LaserScan msgCleanedScan;
  msgCleanedScan.header.frame_id = _frame_id;
  msgCleanedScan.angle_min = history[0]->getPhiMin();
  //TODO: Save real PhiMax in Sensor and make a return function
  msgCleanedScan.angle_max = -history[0]->getPhiMin();
  msgCleanedScan.angle_increment = _angle_increment;
  //	scan_out.time_increment  		=
  //	scan_out.scan_time        		=
  msgCleanedScan.range_min = history[0]->getMinimumRange();
  msgCleanedScan.range_max = history[0]->getMaximumRange();
  msgCleanedScan.ranges.resize(size);
  msgCleanedScan.intensities.resize(size);
  msgCleanedScan.header.stamp = ros::Time::now();
  msgCleanedScan.header.seq = _seq[historynr];

  /*
  * create msg for error scan
  */
  sensor_msgs::LaserScan msgError;
  msgError.header.frame_id = _frame_id;
  msgError.angle_min = history[0]->getPhiMin();
  //TODO: Save real PhiMax in Sensor and make a return function
  msgError.angle_max = -history[0]->getPhiMin();
  msgError.angle_increment = _angle_increment;
  //	scan_out.time_increment  		=
  //	scan_out.scan_time        		=
  msgError.range_min = history[0]->getMinimumRange();
  msgError.range_max = history[0]->getMaximumRange();
  msgError.ranges.resize(size);
  msgError.intensities.resize(size);
  msgError.header.stamp = ros::Time::now();
  msgError.header.seq = _seq[historynr];

  /*
  * clean up scan history
  */
  obvious::SensorPolar2D* tmp_sensor;
  obvious::SensorPolar2D* tmp_sensor2;
  obvious::SensorPolar2D* tmp_sensor_test;

  obvious::Matrix* corners_tmp = new obvious::Matrix(2, 2);

  tmp_sensor = history[historynr];
  tmp_sensor2 = history[historynr];
  tmp_sensor_test = history[historynr];

  for(int i = 0; i < _mirrorHistoryCounter; i++)//_mirrorHistoryCounter
  {
       (*corners_tmp)(0,0) = (*_mirrorHistory[i])(0,0);
       (*corners_tmp)(0,1) = (*_mirrorHistory[i])(0,1);
       (*corners_tmp)(1,0) = (*_mirrorHistory[i])(1,0);
       (*corners_tmp)(1,1) = (*_mirrorHistory[i])(1,1);

       // TODO: include here a fix Testpoint
//    (*corners_tmp)(0,0) = 2.0;
//    (*corners_tmp)(0,1) = 0.3;
//    (*corners_tmp)(1,0) = 1.8;
//    (*corners_tmp)(1,1) = -0.3;
//    cout << "Corners: " << endl;
//    corners_tmp->print();

//    // For visualisation
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

//    cout << "Check corner: " << endl;
//    corners_tmp->print();

//       cout << "Hist: " << (*corners_tmp)(0,0) << "/" << (*corners_tmp)(0,1) << endl;

//    cout << "mask before " << endl;
    //tmp_object_mask = tmp_sensor->getRealMeasurementTypeID();
    tmp_object_mask_test = tmp_sensor->getRealMeasurementTypeID();

//    for(int j = 0; j < size; j++)
//    {
//      cout << tmp_object_mask[j] << "/";
//    }
//    cout << endl;


    cleanScan((*corners_tmp), tmp_sensor, _th_mirrorline, _th_angleThreshold);

    //cout << "mask after: " << endl;
//    tmp_object_mask = tmp_sensor->getRealMeasurementTypeID();
//
//    for(int j = 0; j < size; j++)
//    {
//      if(tmp_object_mask[j] != tmp_object_mask_test[j])
//      cout << tmp_object_mask[j] << "/";
//    }
//    cout << endl;

  }

  tmp_data = tmp_sensor->getRealMeasurementData();
  tmp_data2 = tmp_sensor2->getRealMeasurementData();
  tmp_object_mask = tmp_sensor->getRealMeasurementTypeID();
  tmp_object_mask2 = tmp_sensor2->getRealMeasurementTypeID();


  /*
   * fill laser msgs
   */
  for(unsigned int i = 0; i < size; i++)
  {
    //TODO: Intensities
    switch(tmp_object_mask[i]){
      case 0: // scan
        msgCleanedScan.ranges[i] = tmp_data[i];
        msgError.ranges[i] = 0;
        break;
      case 1: // mirror
        msgCleanedScan.ranges[i] = tmp_data[i];
        msgError.ranges[i] = 0;
        break;
      case 2: // error_mirror
        msgCleanedScan.ranges[i] = 0;
        // TODO: Check in another way!
        // when case 1 or 3 applies, there are also points behind it. These can also be used
        if(tmp_data[i] != 0.0)
        {
          msgError.ranges[i] = tmp_data[i];
         // cout << msgError.ranges[i] << "/";
        }
        else
        {
          msgError.ranges[i] = tmp_data2[i];
        }
//        cout << tmp_data[i] << "-" << tmp_data2[i] << "/";
        break;
      case 3: // transparent
        msgCleanedScan.ranges[i] = tmp_data[i];
        msgError.ranges[i] = 0;
        break;
      case 4: // error_transparent
        msgCleanedScan.ranges[i] = 0;
        //TODO: Check in another way!
        if(tmp_data[i] != 0.0)
        {
          msgError.ranges[i] = tmp_data[i];
        }
        else
        {
          msgError.ranges[i] = tmp_data2[i];
        }
        break;
      default:
        msgCleanedScan.ranges[i] = 0;
        if(tmp_data[i] != 0.0)
        {
          msgError.ranges[i] = tmp_data[i];
        }
        else
        {
          msgError.ranges[i] = tmp_data2[i];
        }
    }
    //cout << msgCleanedScan.ranges[i] << "-" << tmp_object_mask[i] << "/";
     // cout <<  tmp_object_mask[i] << "/";

  }
//  cout << endl;

  _pub_error.publish(msgError);
  _pub_scan.publish(msgCleanedScan);
//  _pub_mirror.publish(_msgInitMirrorMsg);

  delete corners_tmp;
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


void postFiltering(std::vector<obvious::SensorPolar2D*> scan_history, std::vector<obvious::Matrix*>& mHistory, std::vector<
    obvious::Matrix*>& mHistory_T, unsigned int history_count, unsigned int& mirror_count)
{
  //ROS_INFO("Start to clean history");
  /*
   * get mirror data from last sensordata
   */
  unsigned int h_count = history_count - 1;
  int size = scan_history[0]->getRealMeasurementSize();
  double* data = new double[2 * size];
  double* scan;

  bool* data_validmask = new bool[size];
  int* object_mask;

  obvious::Matrix* mirror_corners = new obvious::Matrix(2, 2);
  obvious::Matrix* mirror_matrix = new obvious::Matrix(size, 2);

  obvious::Matrix moved_mirror_matrix(size, 2);

  scan_history[h_count]->dataToCartesianVectorMask(data, data_validmask);
  scan = scan_history[h_count]->getRealMeasurementData();

  object_mask = scan_history[h_count]->getRealMeasurementTypeID();
  for(int i = 0; i < size; i++)
  {
    if((object_mask[i] == 1))
    {
      if((data[i * 2] != 0) && (data[i * 2 + 1] != 0))
      {
        (*mirror_matrix)(i, 0) = data[i * 2];
        (*mirror_matrix)(i, 1) = data[i * 2 + 1];
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
    }
  }
  // get position and bring Mirror into coordinate system "Map"
  obvious::Matrix T = scan_history[h_count]->getTransformation();
  obvious::Matrix* T_scan = &T;

  moved_mirror_matrix = mirror_matrix->createTransform(*T_scan);

  /*
   * analyze last scan, with mirror to get mirror corners
   */
  // get points
  bool corner_points;
  corner_points = getCornerPoints(moved_mirror_matrix, *mirror_corners, data_validmask);

  /*
   * check if mirror already was found before
   */
  bool new_mirror = false;
  if((mirror_count == 0) && corner_points)
  {
    // set first corner points
    mHistory[0] = mirror_corners;
    mHistory_T[0] = T_scan;

    mirror_count++;
    new_mirror = true;
  }
  else if((mirror_count == 1) && corner_points)
  {
      new_mirror = checkMirrorHistory(*mirror_corners, *T_scan, mHistory, mHistory_T, _th_new_corner, _th_mirrorline, mirror_count);
  }

  delete data;
  delete data_validmask;
  delete mirror_matrix;
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

//	cout << "New corner left: " << index_left_corner << "->" << (corners)(0,0) << "/" << (corners)(0,1) << endl;
//	cout << "New corner right: " << index_right_corner << "->" << (corners)(1,0) << "/" << (corners)(1,1) << endl;
//	cout << endl;

  delete angle;
  delete distance;

  if(isnan(corners(0, 0)) or isnan(corners(1, 0)))
    return 0;

  return 1;

//	cout << "Corners:" << endl;
//	cout << "left: " << (corners)(0,0) << "/" << (corners)(0,1) << endl;
//	cout << "right: " << (corners)(1,0) << "/" << (corners)(1,1) << endl;
}

bool checkMirrorHistory(obvious::Matrix& corner_points, obvious::Matrix& T_corner_points, std::vector<obvious::Matrix*>& history, std::vector<
    obvious::Matrix*>& T_history, double th_new_mirror, double th_mirrorline, unsigned int& historysize)
{
  //see book 05/12/15
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

//	cout << "----------------Fkt checkMirrorHistory----------------" << endl;
//	cout << "New corner points to check:" << endl;
//	cout << "corner 1: " << (corner_points)(0,0) <<  "/" << (corner_points)(0,1) << endl;
//	cout << "corner 2: " << (corner_points)(1,0) <<  "/" << (corner_points)(1,1) << endl;
//	cout << endl;
//
//	cout << " This is old history -> size = " << historysize << endl;
//	for(int i=0; i<historysize; i++)
//	{
//			history[i]->print();
//	}
//	cout << endl;
//	cout << endl;

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

      //	cout << "T hist:" << endl;
//			//	T_history[i]->print();
//				cout << "hist 1: " << (*history[i])(0,0) <<  "/" << (*history[i])(0,1) << " - " << distance_r1 << endl;
//				cout << "hist 2: " << (*history[i])(1,0) <<  "/" << (*history[i])(1,1) << " - " << distance_r2 <<  endl;
      if((distance_r1 > th_new_mirror) and (distance_r2 > th_new_mirror))
      {
        // points are to far of history points away, if not found later => add a new Mirror;
        cout << "New corners to far from history" << endl;
        getNewMirror = true;
      }
      else
      {
        // check if mirror corners need update and update
 //       cout << "Check to update old history" << endl;
        hist_tmp = *history[i];
//							cout << "This goes in" << endl;
//							hist_tmp->print();
//							cout << endl;
        foundBetterCorner = checkPointToUpdate(&hist_tmp, corner_points, th_mirrorline);

//						  cout << "History after: " << endl;
//						   hist_tmp.print();

//						  for(int i=0; i < mirror_count; i++)
//						  {
//							  hist[i]->print();
//						  }

        if(foundBetterCorner)
        {
//          cout << "old point" << endl;
//          cout << (*history[i])(0,0) << "/" << (*history[i])(0,1) << endl;
//          cout << (*history[i])(1,0) << "/" << (*history[i])(1,1) << endl;
//
//          cout << endl;
//          cout << "new point" << endl;
//          cout << hist_tmp(0,0) << "/" << hist_tmp(0,1) << endl;
//          cout << hist_tmp(1,0) << "/" << hist_tmp(1,1) << endl;

          *history[i] = hist_tmp;
//
//          history[i](0,0) = hist_tmp(0,0);
//          history[i](0,1) = hist_tmp(0,1);
//          history[i](1,0) = hist_tmp(1,0);
//          history[i](1,1) = hist_tmp(1,1);
//          cout << "Update done" << endl;
//          history[i]->print();
//          cout << endl;


          //T_history[0] = T_corner_points;

//									cout << "this comes out" << endl;
//									hist_tmp->print();
//									cout << endl;
//									history[i]->print();
//									cout << endl;
//									cout << endl;

          getNewMirror = false;
          i = historysize;
        }
      }
    }
    else
    {
      // end search, because end of history;
      //	cout << "reach end of history -> put a new mirror in" << endl;
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

    //cout << "The new mirror nr is " << historysize << endl;
//		for(int i=0; i<historysize; i++)
//		{
//			history[i]->print();
//		}
//		cout << endl;

    newCorner = 1;
  }

 // cout << "------------------End Fkt checkMirrorHistory_2----------------" << endl;

  if(foundBetterCorner or newCorner)
    return true;
  else
    return false;
}

bool checkPointToUpdate(obvious::Matrix* history, obvious::Matrix& corner_points, double th_mirrorline)
{
//  cout << "-------------------------"<< endl;
//  cout << "checkPointToUpdate"<< endl;
  double d_cpoint1toLine = 0.0;
  double d_cpoint2toLine = 0.0;
  double d_cpoint2ToHist1 = 0.0;						// distance between corner point 1 to history corner 2
  double d_cpoint1ToHist2 = 0.0;						// distance between corner point 2 to history corner 1
  double d_hist = 0.0;											// distance between history corners

  obvious::Matrix cpoint_1(1, 2);
  obvious::Matrix cpoint_2(1, 2);

  cpoint_1(0, 0) = corner_points(0, 0);
  cpoint_1(0, 1) = corner_points(0, 1);
  cpoint_2(0, 0) = corner_points(1, 0);
  cpoint_2(0, 1) = corner_points(1, 1);

  obvious::Matrix hpoint_1(1, 2);
  obvious::Matrix hpoint_2(1, 2);

  hpoint_1(0, 0) = (*history)(0, 0);
  hpoint_1(0, 1) = (*history)(0, 1);
  hpoint_2(0, 0) = (*history)(1, 0);
  hpoint_2(0, 1) = (*history)(1, 1);

  // calculate distance between new point and line
  d_cpoint1toLine = calcDistToLine(cpoint_1, *history);
  d_cpoint2toLine = calcDistToLine(cpoint_2, *history);

//    cout << "history" << endl;
//    history->print();
//    cout << "New corner" << endl;
//    corner_points.print();
//  cout << "points 1/2 to line" << d_cpoint1toLine << "/" << d_cpoint2toLine << endl;
  if((d_cpoint1toLine < th_mirrorline) && (d_cpoint2toLine < th_mirrorline))
  {
    // check if corner point are further outside
    // if -> update corner/s of mirror line
    d_cpoint1ToHist2 = calcDistOfLine(cpoint_1, hpoint_2);
    d_cpoint2ToHist1 = calcDistOfLine(cpoint_2, hpoint_1);
    d_hist = calcDistOfLine(hpoint_1, hpoint_2);
    // cout << " d_p1_2h2/d_p2_2h1/p_hist " << d_cpoint1ToHist2 << "/" << d_cpoint2ToHist1 << "/" << d_hist << endl;

    //cout << "Dist: " << d_hist << "/" << d_cpoint1ToHist2 << "/" << d_cpoint2ToHist1 << endl;
    if(d_cpoint1ToHist2 < d_hist)
    {
      (*history)(0, 0) = cpoint_1(0, 1);
      (*history)(0, 1) = cpoint_1(0, 1);
    }

    if(d_cpoint2ToHist1 > d_hist)
    {
      (*history)(1, 0) = cpoint_2(0, 1);
      (*history)(1, 1) = cpoint_2(0, 1);
    }
//    cout << "updated points" << endl;
//    history->print();
//    cout << "CheckPointToUpdate" << endl;
//    cout << "-----------------------" << endl;
    return 1;
  }

  return 0;
}

void cleanScan(obvious::Matrix& corners, obvious::SensorPolar2D* sensor, double th_mirrorline, double th_angleThreshold)
{
   //see book 05/19/15
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
   // point 1 = sensor position (recalculation see book 06/01/15)
   // point 2 = data point i
   if(!((*sensor_position)(1,0) == (*sensor_position)(1,1)))
   {
//     (*intersectionLinePoints)(0,1) = ((*sensor_position)(1,2) * (*sensor_position)(0,0))
//         - ((*sensor_position)(0,2) * (*sensor_position)(1,0));
//     (*intersectionLinePoints)(0,0) = ((*sensor_position)(0,2) / (*sensor_position)(0,0))
//         + (((*sensor_position)(1,0) / (*sensor_position)(0,0)) * (*intersectionLinePoints)(0,1));
     (*intersectionLinePoints)(0,0) = (*sensor_position)(0,2);
     (*intersectionLinePoints)(0,1) = (*sensor_position)(1,2);
     (*origin)(0, 0) = (*intersectionLinePoints)(0,0);
     (*origin)(0, 1) = (*intersectionLinePoints)(0,1);
    // origin->print();
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
       // Check the shorter distance between the two angles
       // see book 28.07.
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
     if((lineValues) && (abs(angle_corner_2 - angle_corner_1) >= _th_openingAnglePrefilter))
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
                             object_mask[i] = 1; // mirror
                           }
                           else
                           {
                             object_mask[i] = 2; // error
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
                             object_mask[i] = 1; // mirror
                           }
                           else
                           {
                             object_mask[i] = 2; // error
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
                             object_mask[i] = 1; // mirror
                           }
                           else
                           {
                             object_mask[i] = 2; // error
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
                             object_mask[i] = 1; // mirror
                           }
                           else
                           {
                             object_mask[i] = 2; // error
                           }
                         }
                       }
                     }
                     break;
                 }
             }
//                 }
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

double getAngle(obvious::Matrix& origin, obvious::Matrix& point)
{
  double angle = 0;
  double delta_x = (point)(0,0) - (origin)(0,0);
  double delta_y = (point)(0,1) - (origin)(0,1);
  double delta_hyp = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

  // number of quadrant
  int quadrant = 0;

  if(delta_hyp > 0)
  {
    angle = acos(delta_x / delta_hyp)*180/M_PI;

    if((delta_x > 0) && (delta_y > 0))
    {
      quadrant = 0;
    }
    else if((delta_x < 0) && (delta_y > 0))
    {
      quadrant = 1;
    }
    else if((delta_x < 0) && (delta_y < 0))
    {
      angle = 3;
      angle = 270 - angle;
    }
    else if((delta_x > 0) && (delta_y < 0))
    {
      quadrant = 4;
      angle = 360- angle;
    }
    else
      cout << __PRETTY_FUNCTION__ << "error when calculating angle" << endl;
  }

  return angle;
}

bool checkDirection(obvious::Matrix* origin, obvious::Matrix* point_1, obvious::Matrix *point_2)
{
  /*
   * same direction if,
   * 1. x1 and x2 are both pos or neg
   * and
   * 2. y1 and y2 are both pos or neg
   *
   */
  if(((((*point_1)(0,0) - (*origin)(0,0)) > 0 && ((*point_2)(0,0) - (*origin)(0,0)) > 0) and (((*point_1)(0,1) - (*origin)(0,1)) > 0 && ((*point_2)(0,1) - (*origin)(0,1)) > 0)) or
     ((((*point_1)(0,0) - (*origin)(0,0)) > 0 && ((*point_2)(0,0) - (*origin)(0,0)) > 0) and (((*point_1)(0,1) - (*origin)(0,1)) < 0 && ((*point_2)(0,1) - (*origin)(0,1)) < 0)) or
     ((((*point_1)(0,0) - (*origin)(0,0)) < 0 && ((*point_2)(0,0) - (*origin)(0,0)) < 0) and (((*point_1)(0,1) - (*origin)(0,1)) > 0 && ((*point_2)(0,1) - (*origin)(0,1)) > 0)) or
     ((((*point_1)(0,0) - (*origin)(0,0)) < 0 && ((*point_2)(0,0) - (*origin)(0,0)) < 0) and (((*point_1)(0,1) - (*origin)(0,1)) < 0 && ((*point_2)(0,1) - (*origin)(0,1)) < 0)))
  {
    return true;
  }
  else
  {
    return false;
  }
}

double calcDistToLine(obvious::Matrix& point, obvious::Matrix& corners)
{
  double distance = NAN;

  float t_exist = 0.0;
  float m_exist = 0.0;
  bool lineValues = false;

  float m_intersection = 0;
  float t_intersection = 0;
  obvious::Matrix* intersection = new obvious::Matrix(1, 2);
  bool intersectionValues = false;

  lineValues = calcLineValues(corners, t_exist, m_exist);
  if(lineValues and (m_exist != 0))
  {
    // calculate intersection point
    m_intersection = -1 / m_exist;
    t_intersection = (point)(0, 1) - m_intersection * (point)(0, 0);

    (*intersection)(0, 0) = (t_exist - t_intersection) / (m_intersection - m_exist);
    (*intersection)(0, 1) = m_intersection * ((*intersection)(0, 0)) + t_intersection;

    // calculate distance
    distance = calcDistOfLine(*intersection, point);
  }

  delete intersection;

  return distance;
}

double calcDistOfLine(obvious::Matrix& point1, obvious::Matrix& point2)
{
  return sqrt(pow(((point2)(0, 0) - (point1)(0, 0)), 2.0) + pow(((point2)(0, 1) - (point1)(0, 1)), 2.0));
}

bool calcLineValues(obvious::Matrix& points, float& t, float& m)
{
  if((((points)(1, 0)) - ((points)(0, 0))) != 0)
  {
    m = ((points)(1, 1) - (points)(0, 1)) / ((points)(1, 0) - (points)(0, 0));
    t = (points)(1, 1) - m * (points)(1, 0);

    return 1;
  }
  else
  {
    return 0;
  }
}

/*
 * Init
 */
void init()
{
  // config marker
  // POINTS markers use x and y scale for width/height respectively
  _marker_points.scale.x = 0.05;
  _marker_points.scale.y = 0.05;
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
  std::string sub_pose;

  std::string sub_activatePub;

  std::string pub_scan;
  std::string pub_error;
  std::string pub_mirror;
  std::string pub_marker;
  double dVar = 0;

  nh_sub.param < std::string > ("sub_maskScan", sub_maskScan, "maskScan");
  nh_sub.param < std::string > ("sub_pose", sub_pose, "pose");
  nh_sub.param < std::string > ("sub_activatePub", sub_activatePub, "activatePub");

  nh_pub.param < std::string > ("pub_scan", pub_scan, "scan_corrected");
  nh_pub.param < std::string > ("pub_error", pub_error, "scan_error");
  nh_pub.param < std::string > ("pub_mirror", pub_mirror, "mirror_empty");

  nh_pub.param < std::string > ("pub_marker", pub_marker, "marker");              				 // maker to show in rviz

  nh_pub.param<double>("thres_mirrorcorner", dVar, 0.5);                      // [m] minimal distance between two mirrors
  _th_new_corner = static_cast<float>(dVar);
  nh_pub.param<double>("thres_mirrorline", dVar, 0.1);                        // [m] maximal distance from the mirror line, if closer the point counted as a mirror point
  _th_mirrorline = static_cast<float>(dVar);
  nh_pub.param<double>("thres_openingAnglePrefilter", dVar, 1);            // [°] additional threshold angle at the mirror sector, makes the sector lightly bigger
  _th_openingAnglePrefilter = static_cast<float>(dVar);
   nh_pub.param<double>("thres_angleThreshold", dVar, 5);                     // [°] minimal angle to check for mirror points at an incoming scan
  _th_angleThreshold = static_cast<float>(dVar);

  nh_pub.param<double>("min_range", dVar, 0.01);                        // [m]
  _minRange = static_cast<float>(dVar);
  nh_pub.param<double>("max_range", dVar, 30);                          // [m]
  _maxRange = static_cast<float>(dVar);
   nh_pub.param<double>("low_reflectivity_range", dVar, 2.0);           //
  _lowReflectivityRange = static_cast<float>(dVar);

  // initialize subscriber
  ros::Subscriber sub1 = nh_sub.subscribe(sub_maskScan, 2, subscribermaskScan);
  ros::Subscriber sub2 = nh_sub.subscribe(sub_pose, 2, subscriberPose);
  ros::Subscriber sub3 = nh_sub.subscribe(sub_activatePub, 2, subscriberActivatePub);

  // initialize publisher
  _pub_scan = nh_pub.advertise<sensor_msgs::LaserScan>(pub_scan, 1);
  _pub_error = nh_pub.advertise<sensor_msgs::LaserScan>(pub_error, 1);

  _pub_marker = nh_pub.advertise<visualization_msgs::Marker>(pub_marker, 1);

  init();

  ros::Rate r(25);

  while(ros::ok())
  {
    if(_new_mirror)
    {
      if(_mirrorHistoryCounter < 45)
      {
        postFiltering(_sensorHistory, _mirrorHistory, _mirrorHistory_T, _sensorDataCounter, _mirrorHistoryCounter);
      }
    }

    if(_publishOn)
    {
      if(historynr < _sensorDataCounter)
      {
        publisherFuncScan(_sensorHistory, _sensorHistory2, historynr);
        historynr++;
      }
    }
    _new_mirror = false;

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down");

  return 0;
}
