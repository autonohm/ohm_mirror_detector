/**
* @file   Ransac.cpp
* @author Rainer Koch
* @date   28.01.2015
*
*/


#include "ransac.h"

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <iostream>
#include <algorithm>

using std::vector;
using namespace cv;
using namespace std;

bool ransac2D(vector<cv::Point2f>& data, vector<cv::Point2f>& linePoints, int minNumToFit, int iterations, float dist_threshold, bool* mask)
{
  int good_Points             = 0;        // the number of points fitting the model
  int trailcount              = 0;        // count how many trails had been done
  int sizeData                = 0;        // amount of data points
  bool* tmp_mask = NULL;
  int max_empty               = 20;       // max. amount of empty/not valid points between two points

  int idx_1                   = 0;                        // index for first sample point
  int idx_2                   = 0;                        // index for second sample point
  Point2f sample_point_1;           // point 1 for model line
  sample_point_1.x            = 0.0;
  sample_point_1.y            = 0.0;
  Point2f sample_point_2;           // point 2 for model line
  sample_point_2.x            = 0.0;
  sample_point_2.y            = 0.0;
  float m                     = 0.0;                          // gradient of model
  float m_inv                 = 0.0;                      // vertical gradient to model
  float t                     = 0.0;                          // y-intercept
  float t_inv                 = 0.0;                      // y-intercept
  Point2f intersection_point;       // intersection point of line trough test point and model line
  intersection_point.x        = 0.0;
  intersection_point.y        = 0.0;
  float distance_test_point   = 0;

  // to modify the required goal points based on the distance
  float d_1                   = 0.0;
  float d_2                   = 0.0;
  float d_middle              = 0.0;
  int minNumToFitModified     = 0;


  // Return:
  Point2f goodModel1;               // good model point 1
  goodModel1.x                 = 0.0;
  goodModel1.y                 = 0.0;
  Point2f goodModel2;               // good model point 2
  goodModel2.x                 = 0.0;
  goodModel2.y                 = 0.0;

  float m_goodModel           = 0.0;
  float t_goodModel           = 0.0;
  int goodModel_points        = 0;

  sizeData = data.size();
  if(sizeData < minNumToFit)
  {
    cout << "RANSAC ERROR: minNumToFit is to high!" << endl;
    return 0;
  }
  tmp_mask = new bool[data.size()];

  int emergency_stop = 0;           // stops while loop, when getting to many runs.
  while(trailcount < iterations)
  {
    good_Points = 0;

    // Select random subset -> hypothetical inliers
      idx_1 = rand() % sizeData;
      while((data[idx_1].x == 0) && (data[idx_1].y == 0) && emergency_stop < 100)
      {
        idx_1 = rand() % sizeData;
        emergency_stop++;
      }
      emergency_stop = 0;
      idx_2 = rand() % sizeData;

      while(((idx_1 == idx_2) or ((data[idx_2].x == 0) && (data[idx_2].y == 0))) && emergency_stop < 100)
      {
        idx_2 = rand() % sizeData;
        emergency_stop++;
      }

      sample_point_1.x = data[idx_1].x;
      sample_point_1.y = data[idx_1].y;
      sample_point_2.x = data[idx_2].x;
      sample_point_2.y = data[idx_2].y;

      // fit model to hypothetical inlier
      // line for model: y = mx + t
      //model.x = sample_point_2.x - sample_point_1.x;
      //model.y = sample_point_2.y - sample_point_1.y;
      // m = (y2-y1)/(x2-x1)
      m = (sample_point_2.y - sample_point_1.y) / (sample_point_2.x - sample_point_1.x);
      t = sample_point_2.y - m * sample_point_1.x;

    // test all other data -> consensus set
      // line for test point: y_testpoint = m_inv * x_testpoint + t_inv
      // m` = m_inv = -1/m =  (x2-x1) / (y2-y1) => m is vertical to m_inv
      m_inv = -1/m;
      for(int i=0; i < sizeData; i++)
      {
        // t_inv = data[i].y - m_inv * data[i].x = data[i].y + data[i].x / m;
        t_inv = data[i].y - m_inv * data[i].x;
        // intersection point of model and test line
        intersection_point.x = (t_inv - t) / (m - m_inv);
        intersection_point.y = m * intersection_point.x +t;

        // distance between intersection point and test point
        // vector d = data - intersection point
        // distance = |d| = sqrt(d.x² + d.y²)
        distance_test_point = sqrt( pow((data[i].x - intersection_point.x),2.0) + pow((data[i].y - intersection_point.y),2.0));
        if((distance_test_point <= dist_threshold) && ((data[i].x > 0.0) || (data[i].y > 0.0)))
        {
          tmp_mask[i] = true;
          good_Points++;
        }
        else
        {
          tmp_mask[i] = false;
        }
      }
    // check if model is reasonably

      /*
       * change minNumToFit based on the distance between object and scanner
       */
      // calculate middle distance of mirror points to scanner
      d_1 = sqrt( pow((sample_point_1.x),2.0) + pow((sample_point_1.y),2.0));
      d_2 = sqrt( pow((sample_point_2.x),2.0) + pow((sample_point_2.y),2.0));
      d_middle = (d_1 + d_2) / 2;

      //minNumToFitModified = d_middle * minNumToFit;
      minNumToFitModified = minNumToFit;

      // Max amount of "empty" points between, if more points between are not in the line then do not use the modle
      int int_occure1 = 0;
      int int_occure2 = 0;
      for(int i= 0; i < sizeData; i++)
      {
        if(tmp_mask[i])
        {
          int_occure2 = int_occure1;
          int_occure1 = i;
        }
      }


      // Go through all points and check if the distance between each point is smaller than the threshold
      // - sort them by x and y value
      // - check max. distance between two points next to each other in x and y
      // - if distance > thres_max_dist => points are to far away
      double d = 0;

      double thres_max_dist = 0.5;
      bool pointsClose = true;
      float tmp_x[good_Points];
      float tmp_y[good_Points];
      int m = 0;

      int test = 0;
      if((good_Points > goodModel_points) && (good_Points >= minNumToFitModified) && (int_occure2-int_occure1 < max_empty) && pointsClose)
      {
        goodModel1.x = sample_point_1.x;
        goodModel1.y = sample_point_1.y;
        goodModel2.x = sample_point_2.x;
        goodModel2.y = sample_point_2.y;
        m_goodModel = m;
        t_goodModel = t;
        goodModel_points = good_Points;
        for(int i= 0; i < data.size(); i++)
        {
          mask[i] = tmp_mask[i];
          test ++;
        }
      }
    test = 0;

      // improve model
      trailcount++;
  }

  if((goodModel1.x != 0.0) && (goodModel1.y != 0.0) && (goodModel2.x != 0.0) && (goodModel2.y != 0.0))
  {
    linePoints[0].x = goodModel1.x;
    linePoints[0].y = goodModel1.y;
    linePoints[1].x = goodModel2.x;
    linePoints[1].y = goodModel2.y;

    delete tmp_mask;
    return 1;
  }
  delete tmp_mask;
  return 0;
}

bool ransac2D_multi(vector<cv::Point2f>& data, vector<cv::Point2f>& linePoints, int minNumToFit, int iterations, float dist_threshold, vector<bool*>&  mask)
{
  int lines = 0;
  bool ransac_sucessful = true;
  bool linefound = false;

  vector<Point2f> tmp_data(data.size());
  vector<Point2f> tmp_linePoints(2);
  bool* tmp_mask = new bool[data.size()];
  for(int i=0; i < data.size(); i++)
  {
    tmp_data[i].x = data[i].x;
    tmp_data[i].y = data[i].y;
  }

  while(ransac_sucessful)
  {
    ransac_sucessful = ransac2D(tmp_data, tmp_linePoints, minNumToFit, iterations, dist_threshold, tmp_mask);

    if(ransac_sucessful)
    {
      linefound = true;
      if((linePoints.size()/2) < (lines+2))
      {
        linePoints.resize(lines*2+2);
        mask.resize(lines+1);
        mask[lines] = new bool[tmp_data.size()];
      }

      linePoints[2*lines].x = tmp_linePoints[0].x;
      linePoints[2*lines].y = tmp_linePoints[0].y;
      linePoints[2*lines+1].x = tmp_linePoints[1].x;
      linePoints[2*lines+1].y = tmp_linePoints[1].y;

      for(int i=0; i < data.size(); i++)
      {
        mask[lines][i] = tmp_mask[i];
        if(tmp_mask[i])
        {
          tmp_data[i].x = 0.0;
          tmp_data[i].y = 0.0;
          tmp_mask[i] = 0;
        }
      }
      lines ++;
    }
  }
  return linefound;
}

