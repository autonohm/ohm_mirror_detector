/**
* @file   lineFunctions.cpp
* @author Rainer Koch
* @date   28.01.2016
*
*
*/
#include "lineFunctions.h"


#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Time.h"
#include <math.h>

using namespace obvious;
using namespace cv;

#define PI 3.14159265

bool checkPointToUpdate(obvious::Matrix* history, obvious::Matrix& corner_points, double th_mirrorline)
{
  double d_cpoint1toLine = 0.0;
  double d_cpoint2toLine = 0.0;
  double d_cpoint2ToHist1 = 0.0;            // distance between corner point 1 to history corner 2
  double d_cpoint1ToHist2 = 0.0;            // distance between corner point 2 to history corner 1
  double d_hist = 0.0;                      // distance between history corners

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

  if((d_cpoint1toLine < th_mirrorline) && (d_cpoint2toLine < th_mirrorline))
  {
    // check if corner point are further outside
    // if -> update corner/s of mirror line
    d_cpoint1ToHist2 = calcDistOfLine(cpoint_1, hpoint_2);
    d_cpoint2ToHist1 = calcDistOfLine(cpoint_2, hpoint_1);
    d_hist = calcDistOfLine(hpoint_1, hpoint_2);
    return 0;
  }
  return 1;
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
  else
  {
   // cout << __PRETTY_FUNCTION__ << ": Can`t calculate distance to Line values!" << endl;
  }

  delete intersection;

  return distance;
}

void calcIntersectionPoint(obvious::Matrix& corners, obvious::Matrix& point, obvious::Matrix& intersection)
{

  float m_mirror = 0;
  float t_mirror = 0;
  float m_normal = 0;
  float m_point = 0;

  float t_intersection = 0;

  /*
   * line for mirror: y = mx + t
   * m = (y2-y1)/(x2-x1)
   * t = y - mx
   */
  m_mirror = ((corners)(1, 1) - (corners)(0, 1)) / ((corners)(1, 0) - (corners)(0, 0));
  t_mirror = (corners)(1, 1) - m_mirror * (corners)(1, 0);

  m_normal = -1/m_mirror;

  /*
   * intersection point with mirror line => vek_ip_i
   *
   * m_p_i = vek_p_point.y / vek_p_point.x
   *
   * y_mirror = m_mirror * x + t_mirror
   * y_p_i    = m_point * x
   *
   * y_mirror = y_point
   * x_p_i  = t_mirror / (m_point - m_mirror)
   */
  m_point = (point)(0, 1) / (point)(0, 0);

  (intersection)(0,0) = t_mirror / (m_point - m_mirror);
  (intersection)(0,1) = m_point * (intersection)(0,0);
}

double calcDistOfLine(obvious::Matrix& point1, obvious::Matrix& point2)
{
  return sqrt(pow(((point2)(0, 0) - (point1)(0, 0)), 2.0) + pow(((point2)(0, 1) - (point1)(0, 1)), 2.0));
}

double calcDistOfVector(obvious::Matrix& vector)
{
  return sqrt(pow((vector(0, 0)), 2.0) + pow((vector(0, 1)), 2.0));
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

    if((delta_x >= 0) && (delta_y >= 0))
    {
      quadrant = 0;
    }
    else if((delta_x <= 0) && (delta_y >= 0))
    {
      quadrant = 1;
    }
    else if((delta_x <= 0) && (delta_y <= 0))
    {
      angle = 3;
      angle = 270 - angle;
    }
    else if((delta_x >= 0) && (delta_y <= 0))
    {
      quadrant = 4;
      angle = 360- angle;
    }
    else
    {
      cout << __PRETTY_FUNCTION__ << "error when calculating angle" << endl;
    }
  }
  return angle;
}

double getAngle2Normal(obvious::Matrix& origin, obvious::Matrix& point, obvious::Matrix& corners)
{
  obvious::Matrix n(1,2);
  n(0,0) = -(corners(0,1) - corners(1,1));
  n(0,1) = corners(0,0) - corners(1,0);
  obvious::Matrix v(1,2);

  v(0,0) = point(0,0) - origin(0,0);
  v(0,1) = point(0,1) - origin(0,1);

  double d = acos((n(0,0) * v(0,0) + n(0,1) * point(0,1)) / (calcDistOfVector(n) * calcDistOfVector(v))) * 180 / PI;
  return d;
}

bool calcLineValues(obvious::Matrix& points, float& t, float& m)
{
  if((((points)(1, 0)) - ((points)(0, 0))) != 0)
  {
    // m = (y2-y1)/(x2-x1)
    m = ((points)(1, 1) - (points)(0, 1)) / ((points)(1, 0) - (points)(0, 0));
    // t = y - mx
    t = (points)(1, 1) - m * (points)(1, 0);
    return 1;
  }
  else
  {
    return 0;
  }
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

bool checkPointOnObject(obvious::Matrix& point, obvious::Matrix& corners, double thres_line)
{
  obvious::Matrix origin(2,2);
  origin(0,0) = 0.0;
  origin(0,1) = 0.0;
  origin(1,0) = 0.0;
  origin(1,1) = 0.0;

  obvious::Matrix c1(1,2);
  c1(0,0) = corners(0,0);
  c1(0,1) = corners(0,1);

  obvious::Matrix c2(1,2);
  c2(0,0) = corners(1,0);
  c2(0,1) = corners(1,1);

  double angle_point = getAngle(origin, point);
  double angle_c1 = getAngle(origin, c1);
  double angle_c2 = getAngle(origin, c2);
  // check if point is near the object line
  if(calcDistToLine(point, corners) < thres_line)
  {
    //check if point is between the two corners of the object
    if((angle_point >= angle_c1) and (angle_point <= angle_c2))
      return 1;
    else
      return 0;
  }
  else
    return 0;
}

bool checkPointBehindObject(obvious::Matrix& position, obvious::Matrix& point, obvious::Matrix& corners, double thres_line, double thres_angle)
{
  obvious::Matrix origin(1,2);
  origin(0,0) = position(0,2);
  origin(0,1) = position(1,2);

  obvious::Matrix c1(1,2);
  c1(0,0) = corners(0,0);
  c1(0,1) = corners(0,1);

  obvious::Matrix c2(1,2);
  c2(0,0) = corners(1,0);
  c2(0,1) = corners(1,1);

  obvious::Matrix intersection(1,2);
  intersection(0,0) = 0.0;
  intersection(0,1) = 0.0;

  double angle_point = getAngle(origin, point);
  double angle_c1 = getAngle(origin, c1);
  double angle_c2 = getAngle(origin, c2);

  if((angle_point > (angle_c1 + thres_angle)) and (angle_point < (angle_c2 - thres_angle)) or (angle_point < (angle_c1 - thres_angle)) and (angle_point > (angle_c2 + thres_angle)) )
  {
    calcIntersectionPoint(corners, point, intersection);

    double distance_intersection = calcDistOfLine(origin, intersection);
    double distance_point        = calcDistOfLine(origin, point);

    //check if point is between the two corners of the object
    if(distance_point > (distance_intersection+ thres_line))
    {
      return 1;
    }
    else
      return 0;
  }
  else
    return 0;

}

void getOuterLinePoints(std::vector<cv::Point2f> points, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corners, bool* mask_linepoints, float threshold)
{
  int size = 0;
  // check for amount of valid points
  for(int i=0; i < points.size(); i++)
  {
    if((points[i].x == 0) && (points[i].y == 0));
      size++;
  }

  // convert xy to distance and angle
  double* distance = new double[size];
  double* angle = new double[size];
  for(int i=0; i < size; i++)
  {
    distance[i] = sqrt(pow((points[i].x),2.0) + pow((points[i].y),2.0));
    angle[i] =  atan2(points[i].y,points[i].x)* 180/M_PI;
  }
  double dist_lp1 = sqrt(pow((line_points[0].x),2.0) + pow((line_points[0].y),2.0));;         // line point 1
  double angle_lp1 = atan2(line_points[0].y,line_points[0].x)* 180/M_PI;
  double dist_lp2 = sqrt(pow((line_points[1].x),2.0) + pow((line_points[1].y),2.0));;         // line point 2
  double angle_lp2 = atan2(line_points[1].y,line_points[1].x)* 180/M_PI;

  // find outer points
  double min_angle_value = 360.0;
  int min_position = 0;
  double max_angle_value = 0.0;
  int max_position = 0;
  for(int i=0; i < size; i++)
  {
      if((min_angle_value >= angle[i]) && (distance[i] != 0.0) && mask_linepoints[i])
      {
          min_angle_value = angle[i];
          min_position = i;
          cout << "i1:" << i << " " << min_angle_value << " " << angle[i] << " " << mask_linepoints[i] << endl;
      }
      if((max_angle_value <= angle[i]) && (distance[i] != 0.0) && mask_linepoints[i])
      {
          max_angle_value = angle[i];
          max_position = i;
          cout << "i2:" << i << " " << max_angle_value << " " << angle[i] <<  " " << mask_linepoints[i] << endl;
      }
  }
  // write new corners
  corners[0].x = points[min_position].x;
  corners[0].y = points[min_position].y;
  corners[1].x = points[max_position].x;
  corners[1].y = points[max_position].y;
}

void getOuterLinePoints2(std::vector<cv::Point2f> points, std::vector<cv::Point2f> line_points, std::vector<cv::Point2f>& corners, bool* mask_linepoints, float threshold)
{
  std::vector<cv::Point2f> p_0(1);
  p_0[0].x = line_points[0].x;
  p_0[0].y = line_points[0].y;

  std::vector<cv::Point2f> p_1(1);
  p_1[0].x = line_points[1].x;
  p_1[0].y = line_points[1].y;

  std::vector<cv::Point2f> v_p0_p1(1);
  v_p0_p1[0].x = p_1[0].x - p_0[0].x;
  v_p0_p1[0].y = p_1[0].y - p_0[0].y;

  double d_p0_p1 = sqrt(pow((v_p0_p1[0].x),2.0) + pow((v_p0_p1[0].y),2.0));

  std::vector<cv::Point2f> tmp_p_x0(1);
  std::vector<cv::Point2f> tmp_p_x1(1);

  std::vector<cv::Point2f> v_p0_pi(1);
  std::vector<cv::Point2f> v_p1_pi(1);

  std::vector<cv::Point2f> v_p0_px0(1);
  std::vector<cv::Point2f> v_p0_px1(1);

  std::vector<cv::Point2f> v_p1_px0(1);
  std::vector<cv::Point2f> v_p1_px1(1);

  double d_p0_px1 = 0;
  double d_p1_px0 = 0;

  double d_p0_pi = 0;
  double d_p1_pi = 0;

  bool check_other_point = false;
  double k = 0;
  bool new_p0 = false;
  bool new_p1 = false;

  // check points for further outer point
  for(int i=0; i < points.size(); i++)
  {
    if(((points[i].x != 0.0) or (points[i].y != 0)) and (mask_linepoints[i]))
    {
      // get vector from p0 to point xi
      v_p0_pi[0].x = points[i].x - p_0[0].x;
      v_p0_pi[0].y = points[i].y - p_0[0].y;
      // get length of the vector
      d_p0_pi = sqrt(pow((v_p0_pi[0].x),2.0) + pow((v_p0_pi[0].y),2.0));

      // check for point further right
      if(d_p0_p1 < d_p0_pi)
      {
        // k*v_po_p1 = v_p0_pxi
        k = v_p0_pi[0].x / v_p0_p1[0].x;

        if(k > 1)
        {
          if(d_p0_px1 < d_p0_pi)
          {
            // set point xi as new further right point
            tmp_p_x1[0].x = points[i].x;
            tmp_p_x1[0].y = points[i].y;
            d_p0_px1 = d_p0_pi;
            new_p1 = true;
          }
        }
        else
        {
          check_other_point = true;
        }
      }
      else
      {
        check_other_point = true;
      }
      k = 0;

      // check for point further left
      if(check_other_point)
      {
        // get vector from p0 to point xi

        v_p1_pi[0].x = points[i].x - p_1[0].x;
        v_p1_pi[0].y = points[i].y - p_1[0].y;
        // get length of the vector
        d_p1_pi = sqrt(pow((v_p1_pi[0].x),2.0) + pow((v_p1_pi[0].y),2.0));

        if(d_p0_p1 < d_p1_pi)
        {

          // k*(-v_po_p1) = v_p1_pxi (neg, because the vector real vektor is v_p1_p0)
          k = -v_p1_pi[0].x / v_p0_p1[0].x;
          if(k > 1)
          {

            if(d_p1_px0 < d_p1_pi)
            {

              // set point xi as new further right point
              tmp_p_x0[0].x = points[i].x;
              tmp_p_x0[0].y = points[i].y;
              d_p1_px0 = d_p1_pi;
              new_p0 = true;
            }
          }
        }
      }
    }
    k = 0;
    check_other_point = false;
  }

  // write new corners
  if(new_p0)
  {
    corners[0].x = tmp_p_x0[0].x;
    corners[0].y = tmp_p_x0[0].y;
  }
  else
  {
    corners[0].x = line_points[0].x;
    corners[0].y = line_points[0].y;
  }

  if(new_p1)
  {
    corners[1].x = tmp_p_x1[0].x;
    corners[1].y = tmp_p_x1[0].y;
  }
  else
  {
    corners[1].x = line_points[1].x;
    corners[1].y = line_points[1].y;
  }
}

void replacePoints(obvious::Matrix& corners, obvious::Matrix& points, double* replaced_coords)
{
    float m_mirror = 0.0;             // gradient mirror line
    float t_mirror = 0.0;             // y at x=0
    float m_inv = 0.0;                // gradient mirror line
    float t_inv = 0.0;
    float m_p_i = 0.0;
    float m_normal = 0.0;
    float t_normal = 0.0;
    float distance_testpoint = 0.0;

    int size = points.getRows();

  //  int* object_mask;
    double* data = new double[2 * size];

    bool* data_validmask  = new bool[size];
    bool points_replaced = false;

    obvious::Matrix* intersection_point_i = new obvious::Matrix(1, 2);                // intersection point between mirror_corner_line and line between sensor position and test point
    obvious::Matrix* sensor_position = new obvious::Matrix(3, 3);
    obvious::Matrix* data_matrix = new obvious::Matrix(size, 2);
    obvious::Matrix* vek_p_i = new obvious::Matrix(size, 2);    // vek_p_i: vektor scanner to testpoint p_i
    obvious::Matrix* vek_pn_i = new obvious::Matrix(1, 2);     // vektor normal_point_i and p_i
    obvious::Matrix* vek_ip_i = new obvious::Matrix(1, 2);     // vektor intersection point and p_i
    obvious::Matrix* normal_point_i = new obvious::Matrix(1, 2);
    obvious::Matrix* corner_1 = new obvious::Matrix(1,2);                     // intersection point between mirror_corner_line and line between sensor position and test point
    obvious::Matrix* corner_2 = new obvious::Matrix(1,2);                     // intersection point between mirror_corner_line and line between sensor position and test point

    (*corner_1)(0, 0) = (corners)(0,0);
    (*corner_1)(0, 1) = (corners)(0,1);
    (*corner_2)(0, 0) = (corners)(1,0);
    (*corner_2)(0, 1) = (corners)(1,1);

    for(int i = 0; i < size; i++)
    {
     (*vek_p_i)(i, 0) = (points)(i, 0);
     (*vek_p_i)(i, 1) = (points)(i, 1);
    }
    /*
     * line for mirror: y = mx + t
     * m = (y2-y1)/(x2-x1)
     * t = y - mx
     */
    m_mirror = ((*corner_2)(0, 1) - (*corner_1)(0, 1)) / ((*corner_2)(0, 1) - (*corner_1)(0, 0));
    t_mirror = (*corner_2)(0, 1) - m_mirror * (*corner_2)(0, 1);

    m_normal = -1/m_mirror;


    vector<Point2f> tmp_recalculated(size);

    /*
     * recalculate affected points
     */
    for(int i=0; i < (size); i++)
    {
      if(((*vek_p_i)(i, 0) != 0.0) && ((*vek_p_i)(i, 1) != 0.0) && !isnan((*vek_p_i)(i, 0)) && !isnan((*vek_p_i)(i, 1)))
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
        m_p_i = (*vek_p_i)(i, 1) / (*vek_p_i)(i, 0);

        (*intersection_point_i)(0,0) = t_mirror / (m_p_i - m_mirror);
        (*intersection_point_i)(0,1) = m_p_i * (*intersection_point_i)(0,0);

        (*vek_ip_i)(0, 0) = (*vek_p_i)(i, 0) - (*intersection_point_i)(0,0);
        (*vek_ip_i)(0, 1) = (*vek_p_i)(i, 1) - (*intersection_point_i)(0,1);
        /*
         * vek_pn_i
         *
         * m_pn_i = -1/m_mirror
         * t_inv = data[i].y - m_inv * data[i].x
         *
         */
        t_normal = (*vek_p_i)(i, 1) - m_normal * (*vek_p_i)(i, 0);

        (*normal_point_i)(0, 0)  = (t_normal - t_mirror) / (m_mirror - m_normal);
        (*normal_point_i)(0, 1) = m_normal * (*normal_point_i)(0, 0) + t_normal;

        (*vek_pn_i)(0, 0) = (*vek_p_i)(i, 0) - (*normal_point_i)(0, 0);
        (*vek_pn_i)(0, 1) = (*vek_p_i)(i, 1) - (*normal_point_i)(0, 1);

        /*
         * original point location
         */
        replaced_coords[2*i] = (*vek_p_i)(i, 0) -  2*(*vek_pn_i)(0, 0);
        replaced_coords[2*i+1] = (*vek_p_i)(i, 1) - 2*(*vek_pn_i)(0, 1);

        points_replaced = true;
      }
    }

    delete intersection_point_i;
    delete sensor_position;
    delete vek_p_i;
    delete vek_pn_i;
    delete vek_ip_i;
    delete normal_point_i;
    delete corner_1;
    delete corner_2;
}

double distOnLine(obvious::Matrix* corners, obvious::Matrix* point)
{
  obvious::Matrix tmp_point(1,2);
  tmp_point(0,0) = (*corners)(0,0);
  tmp_point(0,1) = (*corners)(0,1);

  return calcDistOfLine(tmp_point, *point);
}

void getPerpendicularPoint(obvious::Matrix* intersection, obvious::Matrix& corners, obvious::Matrix& point)
{
  double distance = NAN;

  float t_exist = 0.0;
  float m_exist = 0.0;
  bool lineValues = false;

  float m_intersection = 0;
  float t_intersection = 0;
  bool intersectionValues = false;

  lineValues = calcLineValues(corners, t_exist, m_exist);
  if(lineValues and (m_exist != 0))
  {
    // calculate intersection point
    m_intersection = -1 / m_exist;
    t_intersection = (point)(0, 1) - m_intersection * (point)(0, 0);

    (*intersection)(0, 0) = (t_exist - t_intersection) / (m_intersection - m_exist);
    (*intersection)(0, 1) = m_intersection * ((*intersection)(0, 0)) + t_intersection;
  }
  else
  {
    cout << __PRETTY_FUNCTION__ << ": Can`t calculate perpendicular point!" << endl;
  }

}
