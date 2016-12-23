/**
* @file   reflectionIdentifier.h
* @author Rainer Koch
* @date   25.01.2016
*
*
*/

#ifndef REFLECTIONIDENTIFIER_H_
#define REFLECTIONIDENTIFIER_H_

#include "obvision/reconstruct/grid/SensorPolar2D.h"

/*
 * function to identify the different reflection materials (transparent or mirror)
 *
 *
 *
 * 0: // scan
 * 1: // mirror
 * 2: // error_mirror
 * 3: // transparent
 * 4: // error_transparent
 * 5: // UNKNOWN Object
 * 6: // ERROR UNKNOWN Object
 *
 */
void identifyReflectionType(std::vector<obvious::SensorPolar2D*>& scan_history1, std::vector<obvious::SensorPolar2D*>& scan_history2, unsigned int scan_history_count, std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<obvious::Matrix*>& affected_point_history2, unsigned int& affHistCounter, std::vector<obvious::Matrix*>& mHistory_T, std::vector<obvious::Matrix*>& objectCornerHistory, unsigned int& amountOfObjects, double thres_line, double thres_angle, double thres_pointsForTrans, double thres_diffDistTrans, double thres_diffAngleTrans, double norm_white_intensity, double max_intensityValue, double thres_phongSpreding, double thres_phongFactor, int* objectIdentificationResult);

/*
 * function to check the Intensity Factor
 * - build the median intensities of echo 1 and echo 2 of one single scan
 * - divides the median of echo 2 and echo 2
 */

void checkIntensityFactor(std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<unsigned int*> tmp_maskObjectNr, unsigned int amountOfObjects, unsigned int& affectedHistCounter, int* res_medianIntensFactor);

/*
 * function to check if points behind the reflective object are mirrored in the scan
 */
void checkReflectedPoints(std::vector<obvious::SensorPolar2D*> scan_history1, std::vector<obvious::SensorPolar2D*> scan_history2, unsigned int scan_history_count, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<obvious::Matrix*>& affected_point_history2, unsigned int& affHistCounter, std::vector<obvious::Matrix*>& objectCornerHistory, std::vector<unsigned int*> tmp_maskObjectNr, unsigned int amountOfObjects, double thres_line, double thres_angle, double thres_pointsForTrans, double thres_diffDistTrans, double thres_diffAngleTrans, int* res_Reflect, std::vector<obvious::Matrix*>& trans_Mirror_Objects);

/*
 * function to check a single scan, if
 */
void checkPhong(std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<obvious::Matrix*>& affected_point_history2, std::vector<obvious::Matrix*>& mHistory_T, std::vector<unsigned int*> tmp_maskObjectNr, std::vector<obvious::Matrix*>& objectCornerHistory, unsigned int& amountOfObjects, unsigned int& affHistCounter, double max_intensityValue, double thres_phongSpreding, double thres_phongFactor, int* res_Phong);

/*
 * function to check, how many of the transformations are close together and returns this transformation.
 */
int checkLocationOfTrans(std::vector<obvious::Matrix*> transformation_hist, obvious::Matrix& most_possible_Trans, int amount_valid_Trans, double thres_diffDistTrans, double thres_diffAngleTrans);

#endif /* REFLECTIONIDENTIFIER_H_ */
