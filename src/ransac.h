/**
* @file   ransac.h
* @author Rainer Koch
* @date   28.01.2015
*
*
*/


#ifndef RANSAC_H_
#define RANSAC_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

/**
  * Ransac - LineFit
  *
  * @param inputVariable variable to demonstrate variablenames
  * @return bool 1 = line, 0 = no lines or error
  * 
  * Returns multible lines
  */
  bool ransac2D(std::vector<cv::Point2f>& data, std::vector<cv::Point2f>& linePoints, int minNumToFit, int iterations, float dist_threshold, bool* mask);
  bool ransac2D_multi(std::vector<cv::Point2f>& data, std::vector<cv::Point2f>& linePoints, int minNumToFit, int iterations, float dist_threshold, std::vector<bool*>&  mask);

#endif /* RANSAC_H_ */
