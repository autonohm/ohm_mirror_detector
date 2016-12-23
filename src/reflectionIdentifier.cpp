/**
* @file   reflectionIdentifier.cpp
* @author Rainer Koch
* @date   25.01.2016
*
*
*/


#include "reflectionIdentifier.h"
#include "lineFunctions.h"

#include <vector>

#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>

#include <opencv2/core/core.hpp>

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Timer.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/registration/icp/IcpMultiInitIterator.h"
#include "obvision/registration/icp/assign/FlannPairAssignment.h"

using std::vector;
using namespace std;
using namespace obvious;
using namespace cv;


/*
 * Object type:
 * 0: // scan
 * 1: // mirror
 * 2: // error_mirror
 * 3: // transparent
 * 4: // error_transparent
 * 5: // error_mirror recalculated
 *
 */
bool* tmp_mask;
bool only_once = false;

void identifyReflectionType(std::vector<obvious::SensorPolar2D*>& scan_history1, std::vector<obvious::SensorPolar2D*>& scan_history2, unsigned int scan_history_count, std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<obvious::Matrix*>& affected_point_history2, unsigned int& affectedHistCounter, std::vector<obvious::Matrix*>& mHistory_T, std::vector<obvious::Matrix*>& objectCornerHistory, unsigned int& amountOfObjects, double thres_line, double thres_angle, double thres_pointsForTrans, double thres_diffDistTrans, double thres_diffAngleTrans, double norm_white_intensity, double max_intensityValue, double thres_phongSpreding, double thres_phongFactor, int* objectIdentificationResult)
{
  obvious::Matrix* tmp_points1 = new obvious::Matrix(1,2);
  obvious::Matrix* tmp_points2 = new obvious::Matrix(1,2);

  obvious::Matrix tmp_corners(2,2);
  obvious::Matrix tmp_corners1(2,2);

  std::vector<obvious::Matrix*> trans_Mirror_objects(amountOfObjects);
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    trans_Mirror_objects[i] = new obvious::Matrix(3,3);
    (*trans_Mirror_objects[i])(0,0) = 1;
    (*trans_Mirror_objects[i])(1,1) = 1;
    (*trans_Mirror_objects[i])(2,2) = 1;
  }
  /*
   * mask data of object i
   */
  int size = 0;
  for(unsigned int i=0; i < affectedHistCounter; i++)
  {
    size += affected_point_history1[i]->getRows();
  }

  std::vector<unsigned int*> tmp_maskObjectNr(affectedHistCounter);

  for(unsigned int l=0; l < affectedHistCounter; l++)
  {
    tmp_maskObjectNr[l] = new unsigned int[affected_point_history1[l]->getRows()];
    for(unsigned int j=0; j < affected_point_history1[l]->getRows(); j++)
    {
      tmp_maskObjectNr[l][j] = NAN;
    }
  }

  double* data1 = new double[size,size];
  double* data2 = new double[size,size];

  int* medianIntensFactorResult = new int[amountOfObjects];                   // displays type of object of check intensity factor
  int* phongResult = new int[amountOfObjects];                                // displays type of object of check phong model
  int* reflectedPointResult = new int[amountOfObjects];                       // displays type of object of check refelcted points
  int* normIntensResult = new int[amountOfObjects];                                // displays type of object of check normed intensity

  /*
   * mask all points according to the object
   */
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    tmp_corners(0,0) = (*objectCornerHistory[i])(0,0);
    tmp_corners(0,1) = (*objectCornerHistory[i])(0,1);
    tmp_corners(1,0) = (*objectCornerHistory[i])(1,0);
    tmp_corners(1,1) = (*objectCornerHistory[i])(1,1);
    tmp_corners1(0,0) = (*objectCornerHistory[i])(1,0);
    tmp_corners1(0,1) = (*objectCornerHistory[i])(1,1);
    tmp_corners1(1,0) = (*objectCornerHistory[i])(0,0);
    tmp_corners1(1,1) = (*objectCornerHistory[i])(0,1);

    // check all scans
    for(unsigned int l=0; l < affectedHistCounter; l++)
    {
      affected_point_history1[l]->getData(data1);
      affected_point_history2[l]->getData(data2);
      // mirror_pointsHistory[i]->print();
      // get amount of points in line and copy to tmp_points
      for(unsigned int j=0; j < affected_point_history1[l]->getRows(); j++)
      {
        if(((data1[l,2*j] != 0) or (data1[l,2*j+1] != 0)) and (data1[l,2*j] != NAN) and ((data2[l,2*j] != 0) or (data2[l,2*j+1] != 0)) and (data2[l,2*j] != NAN))
        {
          (*tmp_points1)(0,0) = data1[l,2*j];
          (*tmp_points1)(0,1) = data1[l,2*j+1];
          (*tmp_points2)(0,0) = data2[l,2*j];
          (*tmp_points2)(0,1) = data2[l,2*j+1];

          if(checkPointOnObject(*tmp_points1, tmp_corners, thres_line) or checkPointOnObject(*tmp_points2, tmp_corners, thres_line) or checkPointOnObject(*tmp_points1, tmp_corners1, thres_line) or checkPointOnObject(*tmp_points2, tmp_corners1, thres_line))
          {
            tmp_maskObjectNr[l][j] = i;
          }
        }
      }
    }
  }

/*
 * analyze objects
 *
 */
  /*
   * Check 1: Median Intensity Factor
   */
  checkIntensityFactor(mHistoryIntens1, mHistoryIntens2, affected_point_history1, tmp_maskObjectNr, amountOfObjects, affectedHistCounter, medianIntensFactorResult);

  /*
   * Check 2: reflected points
   */
  checkReflectedPoints(scan_history1, scan_history2, scan_history_count, affected_point_history1, affected_point_history2, affectedHistCounter, objectCornerHistory, tmp_maskObjectNr, amountOfObjects, thres_line,  thres_angle, thres_pointsForTrans, thres_diffDistTrans, thres_diffAngleTrans, reflectedPointResult, trans_Mirror_objects);

  /*
   * Check 3:
   * check if a phong curve can be found
   */
  checkPhong(mHistoryIntens1, mHistoryIntens2, affected_point_history1, affected_point_history2, mHistory_T, tmp_maskObjectNr, objectCornerHistory, amountOfObjects, affectedHistCounter, max_intensityValue, thres_phongSpreding, thres_phongFactor, phongResult);

  /*
   * evaluate check 1-3 to identify object
   */
  for(int i=0; i < amountOfObjects; i++)
  {
    /*
     * the object with the most likelyhood is taken
     * e.g. test 1 = 1
     *      test 2 = 1
     *      test 3 = 3 => sum = 5 / 3 = 1,6   => mirror
     *
     * e.g. test 1 = 1
     *      test 2 = 3
     *      test 3 = 3 => sum = 7 / 3 = 2,3    => transparent
     *
     */
    if(((medianIntensFactorResult[i] + reflectedPointResult[i] + phongResult[i]) / 3.0) < 2.0)
    {
      objectIdentificationResult[i] = 1;    // mirror
      cout << "Final result: Mirror" << endl;
    }
    else
    {
      objectIdentificationResult[i] = 3;     // transparent
      cout << "Final result: Transparent" << endl;
    }
  }
}

void checkIntensityFactor(std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<unsigned int*> tmp_maskObjectNr, unsigned int amountOfObjects, unsigned int& affHistCounter, int* res_medianIntensFactor)
{
  /*
   * median_int = (sum of intensities of one scan)/amount of points of the scan
   * great_median = check how many points are over median_int 5
   */
  double* tmp_IntensScan1;
  double* tmp_IntensScan2;

  std::vector<double*> median_intens1(amountOfObjects);
  std::vector<double*> median_intens2(amountOfObjects);

  double* tmp_median_intens1 = new double[affHistCounter];
  double* tmp_median_intens2 = new double[affHistCounter];
  unsigned int amountOfAffectPoints = 0;
  unsigned int amountOfAffectScans = 0;

  unsigned int greatIntensFactor = 0;

  for(unsigned int i=0; i < amountOfObjects; i++)    // go through all objects
  {
    amountOfAffectScans = 0;

    greatIntensFactor = 0;
    // go through history of scans
    for(unsigned int l=0; l < affHistCounter; l++)   // go through one scan
    {
      amountOfAffectPoints = 0;
      tmp_IntensScan1 = mHistoryIntens1[l];
      tmp_IntensScan2 = mHistoryIntens2[l];
      tmp_median_intens1[l] = 0;
      tmp_median_intens1[l] = 0;
      // go through points of a scan
      for(unsigned int j=0; j < affected_point_history1[i]->getRows(); j++)    // go through each point
      {
        // build median value of intensitie of one scan
        if(tmp_maskObjectNr[l][j] == i)
        {
          tmp_median_intens1[l] += tmp_IntensScan1[j];
          tmp_median_intens2[l] += tmp_IntensScan2[j];
          amountOfAffectPoints++;
        }
      }

      // only do this step, if there are points of this scan belonging to object i
      if((tmp_median_intens1[l] != 0) && (tmp_median_intens2[l] != 0) && (amountOfAffectPoints != 0))
      {
        amountOfAffectScans++;

        tmp_median_intens1[l] = tmp_median_intens1[l]/amountOfAffectPoints;
        tmp_median_intens2[l] = tmp_median_intens2[l]/amountOfAffectPoints;

        if(tmp_median_intens2[l]/tmp_median_intens1[l] > 1)
          greatIntensFactor++;
     }
    }
    median_intens1[i] = tmp_median_intens1;
    median_intens2[i] = tmp_median_intens2;


    if(greatIntensFactor < (0.55*amountOfAffectScans))
    {
      res_medianIntensFactor[i] = 3;       // transparent
      cout << "object " << i << " check 1 -> transparent " << endl;// greatIntensFactor << "/" << amountOfAffectScans << endl;
    }
    else
    {
      res_medianIntensFactor[i] = 1;      // mirror
      cout << "object " << i << " check 1 -> mirror " << endl;//greatIntensFactor << "/" << amountOfAffectScans << endl;
    }
  }
}

void checkReflectedPoints(std::vector<obvious::SensorPolar2D*> scan_history1, std::vector<obvious::SensorPolar2D*> scan_history2, unsigned int scan_history_count, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<obvious::Matrix*>& affected_point_history2, unsigned int& affHistCounter, std::vector<obvious::Matrix*>& objectCornerHistory, std::vector<unsigned int*> tmp_maskObjectNr, unsigned int amountOfObjects, double thres_line, double thres_angle, double thres_pointsForTrans, double thres_diffDistTrans, double thres_diffAngleTrans, int* res_Reflect, std::vector<obvious::Matrix*>& trans_Mirror_Objects)
{
  int printb = 0;

  int size                            = scan_history1[0]->getRealMeasurementSize();

  double* data1                       = new double[size];
  int* objectMask1                    = new int[size];

  double* tmp_data_moved1             = new double[size];
  double* tmp_data_orig1              = new double[size];

  double* tmp_coord_moved             = new double[2*size];
  double* tmp_coord_orig              = new double[2*size];

  double* tmp_coord_moved_map          = new double[2*size];
  double* tmp_coord_orig_map           = new double[2*size];

  obvious::Matrix* moved_matrix       = new obvious::Matrix(size, 2);
  obvious::Matrix* orig_matrix        = new obvious::Matrix(size, 2);
  obvious::Matrix* check_moved_matrix = new obvious::Matrix(1, 2);

  obvious::Matrix* moved_matrix_map   = new obvious::Matrix(size, 2);
  obvious::Matrix* orig_matrix_map    = new obvious::Matrix(size, 2);

  obvious::Matrix* sensor_position    = new obvious::Matrix(3, 3);

  double* tmp_replaced_coords         = new double[2*size];
  bool* tmp_moved_mask                = new bool[size];

  int reflectionsValid    = 0;
  int amountOfAffectScans = 0;

  double lowReflectivityRange = INFINITY;

  obvious::Matrix tmp_corners(2,2);

  obvious::SensorPolar2D* tmp_sensor_moved1 = new obvious::SensorPolar2D(
      size,
      scan_history1[0]->getAngularResolution(),
      scan_history1[0]->getPhiMin(),
      scan_history1[0]->getMaximumRange(),
      scan_history1[0]->getMinimumRange(),
      scan_history1[0]->getLowReflectivityRange());

  obvious::SensorPolar2D* tmp_sensor_original1 = new obvious::SensorPolar2D(
      size,
      scan_history1[0]->getAngularResolution(),
      scan_history1[0]->getPhiMin(),
      scan_history1[0]->getMaximumRange(),
      scan_history1[0]->getMinimumRange(),
      scan_history1[0]->getLowReflectivityRange());

  int dim1 = 0;
  int dim2 = 0;
  bool pointsavaiable = false;

  // for ICP
  int iterations                 = 100;
  PairAssignment* assigner       = (PairAssignment*)  new FlannPairAssignment(2, 0.0, true);
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(2.0, 0.01, iterations);
  assigner->addPostFilter(filterD);
  IRigidEstimator* estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();
  Icp* icp = new Icp(assigner, estimator);

  double rms;
  unsigned int pairs;
  unsigned int it;
  int amount_valid_points;
  int amount_valid_Trans;


  //To visualize
  double* test_coord1         = new double[2*size];
  double* test_coord2         = new double[2*size];

  // Go through all objects
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    tmp_corners(0,0) = (*objectCornerHistory[i])(0,0);
    tmp_corners(0,1) = (*objectCornerHistory[i])(0,1);
    tmp_corners(1,0) = (*objectCornerHistory[i])(1,0);
    tmp_corners(1,1) = (*objectCornerHistory[i])(1,1);

    cout << "corner " << i << endl;
    cout << tmp_corners(0,0) << " " << tmp_corners(0,1) << endl;
    cout << tmp_corners(1,0) << " " << tmp_corners(1,1) << endl;

    // reset counters
    reflectionsValid    = 0;
    amountOfAffectScans = 0;
    amount_valid_Trans  = 0;

    std::vector<obvious::Matrix*> transformation_hist(scan_history_count);

    for(unsigned int j=0; j < scan_history_count; j++)    // check all scans
    {
      /*
       * create a temporal sensor objects
       */
      data1       = scan_history1[j]->getRealMeasurementData();
      objectMask1 = scan_history1[j]->getRealMeasurementTypeID();

      /*
       * check for invalid points
       * assume every masked point is a real scan point or a mirror point
       */
      for(int n=0; n <= size; n++)
      {
        if(data1[n] == 0)
          data1[n]  = NAN;
        if((objectMask1[n] == 2) or (objectMask1[n] == 3) or (objectMask1[n] == 4))
          objectMask1[n] = 1;
      }

      memcpy(tmp_data_orig1, data1, size*sizeof(double));
      memcpy(tmp_data_moved1, data1, size*sizeof(double));

      tmp_sensor_original1->setRealMeasurementData(tmp_data_orig1);
      tmp_sensor_moved1->setRealMeasurementData(tmp_data_moved1);

      tmp_sensor_original1->setRealMeasurementTypeID(objectMask1);
      tmp_sensor_moved1->setRealMeasurementTypeID(objectMask1);

      dim1 = tmp_sensor_original1->dataToCartesianVector(tmp_coord_orig);
      dim2 = tmp_sensor_moved1->dataToCartesianVector(tmp_coord_moved);


      int size_cleaned;
      int m = 0;

      if((dim1 != 0) && (dim2 != 0))
      {
        size_cleaned = 0;
        for(int n=0; n < size; n++)
        {
          (*moved_matrix)(n,0)  = tmp_coord_moved[2*n];
          (*moved_matrix)(n,1)  = tmp_coord_moved[2*n+1];
          (*orig_matrix)(n,0)   = tmp_coord_orig[2*n];
          (*orig_matrix)(n,1)   = tmp_coord_orig[2*n+1];
          if(!((((*orig_matrix)(n,0) == 0) and (*orig_matrix)(n,1) == 0) or isnan((*orig_matrix)(n,1))))
              size_cleaned++;
         }

        /*
         * move points into map coordinate system
         */
        *sensor_position  = scan_history1[j]->getTransformation();
        *moved_matrix_map = moved_matrix->createTransform(*sensor_position);
        *orig_matrix_map  = orig_matrix->createTransform(*sensor_position);

        obvious::Matrix* clean_orig_matrix_map    = new obvious::Matrix(size_cleaned, 2);
        obvious::Matrix* normals_orig_map         = new obvious::Matrix(size_cleaned, 2);
        // Create standard transformation
        obvious::Matrix* T = new obvious::Matrix(3,3);
        (*T)(0,0) = 1;
        (*T)(0,1) = 0;
        (*T)(0,2) = 0;
        (*T)(1,0) = 0;
        (*T)(1,1) = 1;
        (*T)(1,2) = 0;
        (*T)(2,0) = 0;
        (*T)(2,1) = 0;
        (*T)(2,2) = 1;

        /*
         * clean up original matrix to remove invalid points
         */
        m = 0;
        for(int n=0; n < size; n++)
        {
          if(!((((*orig_matrix_map)(n,0) == 0) and ((*orig_matrix_map)(n,1) == 0)) or isnan((*orig_matrix_map)(n,1))))
          {
            (*clean_orig_matrix_map)(m,0)  = (*orig_matrix_map)(n,0);
            (*clean_orig_matrix_map)(m,1)  = (*orig_matrix_map)(n,1);
            m++;
          }
         }

        /*
         * remove all points without the affected (behind the object)
         */
         for(int n=0; n < size; n++)
         {
           tmp_coord_orig_map[2*n]    = (*orig_matrix_map)(n,0);
           tmp_coord_orig_map[2*n+1]  = (*orig_matrix_map)(n,1);
           tmp_moved_mask[n] = 0;

           (*check_moved_matrix)(0,0)   = (*moved_matrix_map)(n,0);
           (*check_moved_matrix)(0,1)   = (*moved_matrix_map)(n,1);

           /*
            * Check if in check_moved_matrix are only the points behind the object
            */
           if(!checkPointBehindObject(*sensor_position, *check_moved_matrix, tmp_corners, thres_line, thres_angle))
           {
             (*moved_matrix_map)(n,0)   = NAN;
             (*moved_matrix_map)(n,1)   = NAN;
           }
           else
           {
             pointsavaiable = true;
           }
         }

         /*
          * only if points behind the object are avaiable in this scan
          */
         if(pointsavaiable)
         {
          /*
           * replace affected points
           */
           replacePoints(tmp_corners, *moved_matrix_map, tmp_replaced_coords);

           /*
            * calculate Normals model
            */
           // compute mean of components build by left and right neighbors
           for(int n=1; n < size_cleaned-1; n++)
           {
             double xleft  = (*clean_orig_matrix_map)(n, 0)   - (*clean_orig_matrix_map)(n-1, 0);
             double xright = (*clean_orig_matrix_map)(n+1, 0) - (*clean_orig_matrix_map)(n, 0);
             double yleft  = (*clean_orig_matrix_map)(n, 1)   - (*clean_orig_matrix_map)(n-1, 1);
             double yright = (*clean_orig_matrix_map)(n+1, 1) - (*clean_orig_matrix_map)(n, 1);

             // x-component of normal
             (*normals_orig_map)(n-1, 0) = -(yright+yleft)/2.0;

             // y-component of normal
             (*normals_orig_map)(n-1, 1) =  (xright+xleft)/2.0;
           }

           // left bound
           (*normals_orig_map)(0, 0) = -((*clean_orig_matrix_map)(1, 1) - (*clean_orig_matrix_map)(0, 1));
           (*normals_orig_map)(0, 1) =   (*clean_orig_matrix_map)(1, 0) - (*clean_orig_matrix_map)(0, 0);

           // right bound
           (*normals_orig_map)(size_cleaned-1, 0) = -((*clean_orig_matrix_map)(size_cleaned-1, 1) - (*clean_orig_matrix_map)(size_cleaned-2, 1));
           (*normals_orig_map)(size_cleaned-1, 1) =   (*clean_orig_matrix_map)(size_cleaned-1, 0) - (*clean_orig_matrix_map)(size_cleaned-2, 0);

           /*
            * check amount of replaced coordinates
            */
           amount_valid_points = 0;
           for(int n=0; n < size; n++)
           {
              if((tmp_replaced_coords[2*n] != 0) or (tmp_replaced_coords[2*n+1] != 0))
               amount_valid_points++;
           }

           if(amount_valid_points >= thres_pointsForTrans)
           {
             obvious::Matrix* scene_matrix_ICP   = new obvious::Matrix(amount_valid_points, 2);
             obvious::Matrix final_mirrored_matrix(amount_valid_points, 2);

             /*
              * copy only valid moved coordinates into scene matrix
              */
             m = 0;
             for(int n=0; n < size; n++)
             {
               if(!(tmp_replaced_coords[2*n] == 0) && !(tmp_replaced_coords[2*n+1] == 0))
               {
                 (*scene_matrix_ICP)(m,0) = tmp_replaced_coords[2*n];
                 (*scene_matrix_ICP)(m,1) = tmp_replaced_coords[2*n+1];
                 m++;
               }
             }

             /*
              * check for match with ICP
              */
              icp->reset();
              icp->setModel(clean_orig_matrix_map, normals_orig_map);
              icp->setScene(scene_matrix_ICP, NULL);
              icp->setMaxRMS(0.0);
              icp->setMaxIterations(iterations);
              icp->activateTrace();

              icp->iterate(&rms, &pairs, &it);
              *T = icp->getFinalTransformation();

              final_mirrored_matrix = scene_matrix_ICP->createTransform(*T);

              delete scene_matrix_ICP;
           }
           /*
             * check the result for this object, => if unit matrix -> no result
             */
           if((amount_valid_points >= thres_pointsForTrans) and !(((*T)(0,0) == 1) && (((*T)(1,1) == 1)) && (((*T)(2,2) == 1))) and !(((*T)(0,0) == 0) && (((*T)(1,1) == 0)) && (((*T)(2,2) == 0))))
           {
             transformation_hist[amount_valid_Trans] = T;
             amount_valid_Trans++;
           }
           amountOfAffectScans++;
         }

         pointsavaiable = false;
         delete clean_orig_matrix_map;
         delete normals_orig_map ;
      }
    }

    /*
      * check, if valid transformations are close together
      * since the scans are in map coordinate system and the mirror reflect them back to their "original" position the ICP should find for all back reflected points the same transformation.
      * The transformation removes the error which are caused by a not perfect detected mirror line.
      */
     obvious::Matrix most_possible_Trans(3,3);

     int amount_of_CloseTogether = checkLocationOfTrans(transformation_hist, most_possible_Trans, amount_valid_Trans, thres_diffDistTrans, thres_diffAngleTrans);

     // if enough transformations are calculated and if enough of these transformations are close together
     if(((double)amountOfAffectScans < (0.7*amount_valid_Trans)) && ((double)amount_of_CloseTogether < (0.7*amount_valid_Trans)))
     {
       res_Reflect[i] = 3;       // transparent
       cout << "object " << i << " check 2 -> transparent " << amount_valid_Trans << "/" << amountOfAffectScans << "/" << amount_of_CloseTogether << endl;
       (*trans_Mirror_Objects[i])(0,0) = 1;
       (*trans_Mirror_Objects[i])(0,1) = 0;
       (*trans_Mirror_Objects[i])(0,2) = 0;
       (*trans_Mirror_Objects[i])(1,0) = 0;
       (*trans_Mirror_Objects[i])(1,1) = 1;
       (*trans_Mirror_Objects[i])(1,2) = 0;
       (*trans_Mirror_Objects[i])(2,0) = 0;
       (*trans_Mirror_Objects[i])(2,1) = 0;
       (*trans_Mirror_Objects[i])(2,2) = 1;
     }
     else
     {
       res_Reflect[i] = 1;       // mirror
       cout << "object " << i << " check 2 -> mirror " << amount_valid_Trans << "/" << amountOfAffectScans << "/" << amount_of_CloseTogether << endl;
       (*trans_Mirror_Objects[i])(0,0) = most_possible_Trans(0,0);
       (*trans_Mirror_Objects[i])(0,1) = most_possible_Trans(0,1);
       (*trans_Mirror_Objects[i])(0,2) = most_possible_Trans(0,2);
       (*trans_Mirror_Objects[i])(1,0) = most_possible_Trans(1,0);
       (*trans_Mirror_Objects[i])(1,1) = most_possible_Trans(1,1);
       (*trans_Mirror_Objects[i])(1,2) = most_possible_Trans(1,2);
       (*trans_Mirror_Objects[i])(2,0) = most_possible_Trans(2,0);
       (*trans_Mirror_Objects[i])(2,1) = most_possible_Trans(2,1);
       (*trans_Mirror_Objects[i])(2,2) = most_possible_Trans(2,2);
     }
  }


  delete [] data1;

  delete [] tmp_data_moved1;
  delete [] tmp_data_orig1;

  delete tmp_sensor_moved1;
  delete tmp_sensor_original1;

  delete sensor_position;
  delete icp;
  delete estimator;
  delete assigner;
}

void checkPhong(std::vector<double*>& mHistoryIntens1, std::vector<double*>& mHistoryIntens2, std::vector<obvious::Matrix*>& affected_point_history1, std::vector<obvious::Matrix*>& affected_point_history2, std::vector<obvious::Matrix*>& mHistory_T, std::vector<unsigned int*> maskObjectNr, std::vector<obvious::Matrix*>& objectCornerHistory, unsigned int& amountOfObjects, unsigned int& affHistCounter, double max_intensityValue, double thres_phongSpreding, double thres_phongFactor, int* res_Phong)
{
  double phong_n_value = 0;

  double* tmp_IntensScan1;
  double* tmp_IntensScan2;

  std::vector<double*> median_intens1(amountOfObjects);
  std::vector<double*> median_intens2(amountOfObjects);

  unsigned int amountOfAffectPoints = 0;
  unsigned int amountOfAffectScans = 0;
  bool take_scan = false;
  unsigned int greatIntensFactor = 0;

  obvious::Matrix tmp_corners(2,2);
  obvious::Matrix* tmp_origin = new obvious::Matrix(1,2);
  obvious::Matrix* tmp_point = new obvious::Matrix(1,2);
  obvious::Matrix* tmp_point1 = new obvious::Matrix(1,2);
  obvious::Matrix* tmp_point2 = new obvious::Matrix(1,2);

  std::vector<double*> l_onLine(amountOfObjects);
  std::vector<double*> d_robot2line(amountOfObjects);
  std::vector<double*> angle(amountOfObjects);            // angle between incoming laser beam and normal of object

  std::vector<double*> intens_1(amountOfObjects);
  std::vector<double*> intens_2(amountOfObjects);

  std::vector<double*> l_onLine_sort(amountOfObjects);        // distance from robot to line
  std::vector<double*> d_robot2line_sort(amountOfObjects);
  std::vector<double*> angle_sort(amountOfObjects);
  std::vector<double*> intens_1_sort(amountOfObjects);
  std::vector<double*> intens_1_sort_norm(amountOfObjects);
  std::vector<double*> intens_2_sort(amountOfObjects);
  std::vector<double*> intens_2_sort_norm(amountOfObjects);
  std::vector<double> value;


  unsigned int steps = 100;
  int* counter1 = new int[steps];
  int* counter2 = new int[steps];

  unsigned int points_total   = 0;
  unsigned int points_scan    = 0;

  //get amount of points for each object
  int* counter_points = new int[amountOfObjects];
  int* jumping_points = new int[amountOfObjects];

  /*
   * Go through all objects and count the points on each object
   */
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    counter_points[i] = 0;
    // go through history of scans
    for(unsigned int l=0; l < affHistCounter; l++)
    {
      // go through points of a scan
      for(unsigned int j=0; j < affected_point_history1[l]->getRows(); j++)
      {
        // check if point is on object
        if(maskObjectNr[l][j] == i)
        {
          counter_points[i] ++;
        }
      }
    }
  }

  /*
   * create space for variables
   */
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    intens_1[i]           = new double[counter_points[i]];
    intens_2[i]           = new double[counter_points[i]];

    l_onLine[i]           = new double[counter_points[i]];
    d_robot2line[i]       = new double[counter_points[i]];            // distance from robot to line
    angle[i]              = new double[counter_points[i]];            // angle between incoming laser beam and normal of object

    l_onLine_sort[i]      = new double[counter_points[i]];
    d_robot2line_sort[i]  = new double[counter_points[i]];
    angle_sort[i]         = new double[counter_points[i]];
    intens_1_sort[i]      = new double[counter_points[i]];
    intens_1_sort_norm[i] = new double[counter_points[i]];
    intens_2_sort[i]      = new double[counter_points[i]];
    intens_2_sort_norm[i] = new double[counter_points[i]];
  }
  /*
   * copy points together
   */
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    /*
     * transform the points on the object surface
     */
    unsigned int points_total = 0;
    unsigned int points_scan = 0;

    /*
     * copy together all data of one object and normalize them
     * calculate distance of point to surface, position the surface, angle incoming laser beam to normal of object
     */
    // go through history of scans
    for(unsigned int l=0; l < affHistCounter; l++)
    {
      (*tmp_origin)(0,0)    = (*mHistory_T[l])(0,2);
      (*tmp_origin)(0,1)    = (*mHistory_T[l])(1,2);

      if(((*tmp_origin)(0,0) == 0) && ((*tmp_origin)(0,0) == 0.0))
      {
        (*tmp_origin)(0,0)    = (*mHistory_T[l-1])(0,2);
        (*tmp_origin)(0,1)    = (*mHistory_T[l-1])(1,2);
      }

      // go through points of one scan to copy together the points placed on the object
      for(unsigned int j=0; j < affected_point_history1[l]->getRows(); j++)
      {
        // check if point is on object
        if(maskObjectNr[l][j] == i)
        {
          (*tmp_point1)(0,0) = (*affected_point_history1[l])(j,0);
          (*tmp_point1)(0,1) = (*affected_point_history1[l])(j,1);
          (*tmp_point2)(0,0) = (*affected_point_history2[l])(j,0);
          (*tmp_point2)(0,1) = (*affected_point_history2[l])(j,1);

          // check which point is closer
          if(calcDistOfLine(*tmp_point1, *tmp_origin) >= calcDistOfLine(*tmp_point2, *tmp_origin))
          {
            (*tmp_point)(0,0) = (*affected_point_history1[l])(j,0);
            (*tmp_point)(0,1) = (*affected_point_history1[l])(j,1);

            d_robot2line[i][points_total]   = calcDistOfLine(*tmp_point, *tmp_origin);

            l_onLine[i][points_total] = distOnLine(objectCornerHistory[i], tmp_point);

            angle[i][points_total]          = getAngle2Normal(*tmp_origin, *tmp_point, *objectCornerHistory[i]);
            if(mHistoryIntens1[l][j] <= max_intensityValue)
            {
              intens_1[i][points_total]       = mHistoryIntens1[l][j];
            }
            else
            {
              intens_1[i][points_total]       = 0;
            }
            if(mHistoryIntens2[l][j] <= max_intensityValue)
            {
              intens_2[i][points_total]       = mHistoryIntens2[l][j];
            }
            else
            {
              intens_2[i][points_total]       = 0;
            }
          }
          else
          {
            (*tmp_point)(0,0) = (*affected_point_history2[l])(j,0);
            (*tmp_point)(0,1) = (*affected_point_history2[l])(j,1);

            d_robot2line[i][points_total]   = calcDistOfLine(*tmp_point, *tmp_origin);
            l_onLine[i][points_total] = distOnLine(objectCornerHistory[i], tmp_point);
            angle[i][points_total]          = getAngle2Normal(*tmp_origin, *tmp_point, *objectCornerHistory[i]);
            if(mHistoryIntens1[l][j] <= max_intensityValue)
            {
              intens_1[i][points_total]       = mHistoryIntens2[l][j];
            }
            else
            {
              intens_1[i][points_total]       = 0;
            }
            if(mHistoryIntens2[l][j] <= max_intensityValue)
            {
              intens_2[i][points_total]       = mHistoryIntens1[l][j];
            }
            else
            {
              intens_2[i][points_total]       = 0;
            }
          }

          points_scan++;
          points_total++;
        }
      }

      /*
       * Normalize points
       */
      double max_1 = 0.0;
      double max_2 = 0.0;
      // determine start- and end-position for this scan
      unsigned int startvalue = points_total-points_scan;
      unsigned int endvalue = points_total;

      // get max. values
      for(unsigned int n=startvalue; n <  endvalue; n++)
      {
        if(intens_1[i][n] > max_1)
          max_1 = intens_1[i][n];
        if(intens_2[i][n] > max_2)
          max_2 = intens_2[i][n];
      }
      if((max_1 > 1.0) && (max_2 > 1.0))
      {
        for(unsigned int n=startvalue; n <  endvalue; n++)
        {
          intens_1[i][n]   = intens_1[i][n] / max_1;
          intens_2[i][n]   = intens_2[i][n] / max_2;
        }
      }
      points_scan = 0;
    }

    /*
     * identify, if many discontinueties are included in the curve of the intensity
     */
    jumping_points[i] = 0;
    for(unsigned int n=1; n <= counter_points[i]; n++)
    {
      if((intens_1[i][n-1] > ((1.0+thres_phongSpreding)*intens_1[i][n])) or (intens_1[i][n-1] > ((1.0-thres_phongSpreding)*intens_1[i][n])))
        jumping_points[i]++;
    }

      if((double)jumping_points[i] >= (thres_phongFactor*counter_points[i]))
      {
        res_Phong[i] = 3;       // transparent
        cout << "object " << i << " check 1 -> transparent " << greatIntensFactor << "/" << amountOfAffectScans << endl;
      }
      else
      {
        res_Phong[i] = 1;      // mirror
        cout << "object " << i << " check 1 -> mirror " << greatIntensFactor << "/" << amountOfAffectScans << endl;
      }
  }

  /*
   * free space
   */
  for(unsigned int i=0; i < amountOfObjects; i++)
  {
    delete median_intens1[i];
    delete median_intens2[i];
    delete intens_1[i];
    delete intens_2[i];

    delete l_onLine[i];
    delete d_robot2line[i];
    delete angle[i];

    delete l_onLine_sort[i];
    delete d_robot2line_sort[i];
    delete angle_sort[i];
    delete intens_1_sort[i];
    delete intens_1_sort_norm[i];
    delete intens_2_sort[i];
    delete intens_2_sort_norm[i];
  }
}

bool replaceMirroredPoints(obvious::Matrix& corners, obvious::SensorPolar2D* sensor, double* replaced_coords, int object_type, bool* moved_mask)
{
    float m_mirror = 0.0;             // gradient mirror line
    float t_mirror = 0.0;             // y at x=0
    float m_inv = 0.0;                // gradient mirror line
    float t_inv = 0.0;
    float m_p_i = 0.0;
    float m_normal = 0.0;
    float t_normal = 0.0;
    float distance_testpoint = 0.0;

    int size = sensor->getRealMeasurementSize();

    int* object_mask;
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

    object_mask = sensor->getRealMeasurementTypeID();
    sensor->dataToCartesianVectorMask(data, data_validmask);

    for(int i = 0; i < size; i++)
    {
     (*vek_p_i)(i, 0) = data[2 * i];
     (*vek_p_i)(i, 1) = data[2 * i + 1];
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
      if((object_mask[i] == 2) && (!moved_mask[i]))  // only do if assigned as mirrored point
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
          moved_mask[i] = true;
        }
			}
    }
    return points_replaced;

    delete intersection_point_i;
    delete sensor_position;
    delete vek_p_i;
    delete vek_pn_i;
    delete vek_ip_i;
    delete normal_point_i;
    delete corner_1;
    delete corner_2;
}

int checkLocationOfTrans(std::vector<obvious::Matrix*> transformation_hist, obvious::Matrix& most_possible_Trans, int amount_valid_Trans, double thres_diffDistTrans, double thres_diffAngleTrans)
{
  /*
   * set all transformations as origin and compare how far the other ones are away
   */
  int amount_of_valid     = 0;
  int max_amount_of_valid = 0;
  double diff_x;
  double diff_y;
  double diff_angle;

  for(int i=0; i < amount_valid_Trans; i++)
  {
    obvious::Matrix origin(3,3);
    origin = *transformation_hist[i];
    amount_of_valid = 0;

    // compare the other ones
    for(int j=0; j < amount_valid_Trans; j++)
    {
      obvious::Matrix toTest(3,3);
      toTest = *transformation_hist[j];
      // if matrix is unit matrix, dont use it
      if(!(((*transformation_hist[j])(0,0) == 1) && (((*transformation_hist[j])(1,1) == 1)) && (((*transformation_hist[j])(2,2) == 1))) and !(((*transformation_hist[j])(0,0) == 0) && (((*transformation_hist[j])(1,1) == 0)) && (((*transformation_hist[j])(2,2) == 0))))
      {
        diff_x = abs((origin)(0,2) - (*transformation_hist[j])(0,2));
        diff_y = abs((origin)(1,2) - (*transformation_hist[j])(1,2));
        diff_angle = (abs(acos(origin(0,0)) - acos((*transformation_hist[j])(0,0))))/M_PI*180.0;

        if((diff_x <= thres_diffDistTrans) && (diff_y <= thres_diffDistTrans) && (diff_angle <= thres_diffAngleTrans))
          amount_of_valid++;
      }
    }
    if((amount_of_valid-1) > max_amount_of_valid)
    {
      max_amount_of_valid = amount_of_valid;
      most_possible_Trans = *transformation_hist[i];
    }
  }

  return max_amount_of_valid;
}
