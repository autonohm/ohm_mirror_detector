/**
* @file   rosfunctions.cpp
* @author Rainer Koch
* @date   14.04.2015
*
*
*/

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sstream>

using namespace std;

void convertPath2ROS(std::vector<cv::Point3f>& coords_path, std::vector<cv::Vec4f>& path_orient, nav_msgs::Path& ros_path)
{
  // resize
  if(ros_path.poses.size() < coords_path.size())
  {
    ros_path.poses.resize(coords_path.size());
  }
  // copy
  for(int i=0; i < coords_path.size(); i++)
  {
    ros_path.poses[i].header.seq          = ros_path.header.seq;
    ros_path.poses[i].header.stamp        = ros_path.header.stamp;
    ros_path.poses[i].header.frame_id     = ros_path.header.frame_id;

    ros_path.poses[i].pose.position.x = coords_path[i].x;
    ros_path.poses[i].pose.position.y = coords_path[i].y;
    ros_path.poses[i].pose.position.z = coords_path[i].z;

    ros_path.poses[i].pose.orientation.x = path_orient[i].val[0];
    ros_path.poses[i].pose.orientation.y = path_orient[i].val[1];
    ros_path.poses[i].pose.orientation.z = path_orient[i].val[2];
    ros_path.poses[i].pose.orientation.w = path_orient[i].val[3];
  }
}

void convertPath2xyz(nav_msgs::Path& ros_path, std::vector<cv::Point3f>& coords_path, std::vector<cv::Vec4f>& path_orient)
{
  // resize
  if(coords_path.size() < ros_path.poses.size())
  {
    coords_path.resize(ros_path.poses.size());
    path_orient.resize(ros_path.poses.size());
  }
  // copy
  for(int i=0; i < ros_path.poses.size(); i++)
  {
    coords_path[i].x     = ros_path.poses[i].pose.position.x;
    coords_path[i].y     = ros_path.poses[i].pose.position.y;
    coords_path[i].z     = ros_path.poses[i].pose.position.z;

    path_orient[i].val[0] = ros_path.poses[i].pose.orientation.x;
    path_orient[i].val[1] = ros_path.poses[i].pose.orientation.y;
    path_orient[i].val[2] = ros_path.poses[i].pose.orientation.z;
    path_orient[i].val[3] = ros_path.poses[i].pose.orientation.w;
  }
}

void convertMap2ROS(std::vector<cv::Point2f>& coords, std::vector<int>& mask, cv::Vec4f& map_orient, cv::Point3f& map_pos, nav_msgs::OccupancyGrid& map)
{
  // resize
  int mapsize = map.info.width * map.info.height;
  int width = map.info.width;
  int height = map.info.height;

  int validPointCounter = 0;

  map.data.resize(mapsize);
  int index = 0;
  int pos_x = 0;
  int pos_y = 0;

  map.info.origin.position.x = map_pos.x;
  map.info.origin.position.y = map_pos.y;
  map.info.origin.position.z = 0.0;

  map.info.origin.orientation.x = map_orient.val[0];
  map.info.origin.orientation.y = map_orient.val[1];
  map.info.origin.orientation.z = map_orient.val[2];
  map.info.origin.orientation.w = map_orient.val[3];

  // copy
  for(int i=0; i < mask.size(); i++)
  {
	  if(mask[i] != -1)
	  {
		  map.data[i] = 0;
	  }
	  else
	  {
		  map.data[i] = -1;
	  }
  }
  for(int i=0; i < (coords.size()); i++)
  {
	  if((coords[i].x != 0) && (coords[i].y != 0))
	  {
		  index = (((coords[i].x-map_pos.x) / map.info.resolution) + (((coords[i].y-map_pos.y) / map.info.resolution) * map.info.width));
		  map.data[index] = 100;
	  }
  }
}

void convertxy2RosSort(double* coords, double* distance, int size, double angle_inc)
{
  for (unsigned int i = 0; i < size; i++)
  {
    distance[i] = 0;
  }
  int j = 0;
  int max_inc = 2*M_PI / angle_inc;         // 360° = max_inc
  double scanangle = 2*M_PI*0.75; // :360*270 = *0.75
   // resize scan to get 360°
  for (unsigned int i = 0; i < size; i++)
  {
    if(!(isnan(coords[2*i+1])))
    {
      if(coords[2*i] != 0.0)
      {
        // calculate new angle 
        j = (atan(coords[2*i] / -coords[2*i+1]) / angle_inc);
        if(j > max_inc)
        {
          j =  j - max_inc;
        }
        else if(j < 0)
        {
          j = j + max_inc;
        }
        if(j > 540)
        {
          j = j - 540;
          distance[j] = sqrt(pow((coords[2*i+1]),2.0) + pow((coords[2*i]),2.0));
        }
      }
      else
      {
        j = max_inc;
        distance[j] = coords[2*i];
      }


    }
  }
}

void convertMap2xy(nav_msgs::OccupancyGrid& map, std::vector<cv::Point2f>& coords, std::vector<int>& mask)
{
  // resize
  unsigned int width = map.info.width;
  unsigned int height = map.info.height;


  int validPointCounter = 0;

  if(coords.size() < map.data.size())
  {
    coords.resize(map.data.size());
    mask.resize(map.data.size());
  }
  // copy
  int index = 0;

  for(unsigned int i=0; i < height; i++)
  {
    for(unsigned int j=0; j < width; j++)
    {
      mask[j+i*width] = map.data[j+i*width];
      if((map.data[j+i*width] != -1) && map.data[j+i*width] != 0)
      {
        coords[index].x = map.info.resolution * j + map.info.origin.position.x;
        coords[index].y = map.info.resolution * i + map.info.origin.position.y;
        index++;
      }
    }
  }
}

void convertROS2xy(sensor_msgs::LaserScan &scan, std::vector<cv::Point2f> &data, float angle_increment, float scanangle)
{
  cv::Point2f tmp_point;
  for (unsigned int i = 0; i < (scan.ranges.size()); i++)
  {
    tmp_point.x = -scan.ranges[i] * sin(angle_increment*i + scanangle/2);
    tmp_point.y = -scan.ranges[i] * cos(angle_increment*i + scanangle/2);
    data[i] = tmp_point;
  }
}

void convertxy2ROS(std::vector<cv::Point2f> &data, sensor_msgs::LaserScan &scan, float angle_increment, float scanangle)
{

  int j = 0;
  int max_inc = 2*M_PI / angle_increment;         // 360° = max_inc
   // resize scan to get 360°
  scan.ranges.resize(max_inc);
  for (unsigned int i = 0; i < (data.size()); i++)
  {
    if(data[i].x != 0.0)
    {
      // calculate new angle 
      j = (atan(data[i].y / data[i].x) / angle_increment) - (scanangle/6/angle_increment);
      if(j > max_inc)
      {
        j =  j - max_inc;
      }
      else if(j < 0)
      {
        j = j + max_inc;
      }
      scan.ranges[j] = sqrt(pow((data[i].x),2.0) + pow((data[i].y),2.0));
    }
    else
    {
      j = max_inc - (scanangle/6/angle_increment);
      scan.ranges[j] = data[i].y;
    }
  }
}

void copyHeaderScan(sensor_msgs::LaserScan scan_in, sensor_msgs::LaserScan &scan_out)
{
	scan_out.header.frame_id 	= scan_in.header.frame_id;
	scan_out.header.stamp 		= scan_in.header.stamp;
	scan_out.header.seq       	= scan_in.header.seq;

	scan_out.angle_min        	= scan_in.angle_min;
	scan_out.angle_max        	= scan_in.angle_max;
	scan_out.angle_increment  	= scan_in.angle_increment;
	scan_out.time_increment  	= scan_in.time_increment;
	scan_out.scan_time        	= scan_in.scan_time;
	scan_out.range_min        	= scan_in.range_min;
	scan_out.range_max        	= scan_in.range_max;
}

