/**
 * @brief  This header file implements aruco_detector class for detecting a single marker and estimating its pose w.r.t the camera cooridinate
 *         The class is built based on the sample code in OpenCV sample code in aruco package.
 *         KLT is added for recover detections that are missed by the detector due to blurry and bad viewing angle
 *
 * @author Yiming Wang (wangyimingkaren@gmail.com)
 * @date   13/04/2017
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ARUCO_DETECTOR_HPP_INCLUDED
#define ARUCO_DETECTOR_HPP_INCLUDED

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include <opencv2/aruco.hpp>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <algorithm>


#include "lk_util.hpp"
#include "mat_util.hpp"
#include "file_util.hpp"

namespace aruco_detection
{

/**
 * @brief read the detector parameters within a version wrapper to handle different OpenCV3 versions
 * @param the detector parameters that is read in type aruco::DetectorParameters
 * @param the path of the detector file in strings
*/
#if CV_MINOR_VERSION >= 2
bool  readDetectorParametersVersionWrapper(cv::Ptr<cv::aruco::DetectorParameters> &detectorParams, std::string abs_path_det);
#else
bool  readDetectorParametersVersionWrapper(cv::aruco::DetectorParameters &detectorParams, std::string abs_path_det);
#endif

/**
 * @brief class aruco_detector
*/
class aruco_detector
{
public:

    /**
     * @brief constructor
    */
    aruco_detector();

    /**
     * @brief set the target id
     * @param id to set
    */
    void setTargetID(int id);

    /**
     * @brief core function for detecting the target marker based on the both the detections provided with built-in marker detector function and KLT.
     * @param current image in rgb
     * @param current image in grey for KLT
     * @param previous image in grey for KLT
     * @param detected contours with point data type cv::Point2f from the built-in detector
     * @param detected ids from the built-in detector
    */
    void detectMarkerOnce(const cv::Mat &image, const cv::Mat &grey, const cv::Mat &prevGrey, const std::vector< std::vector< cv::Point2f > > &corners, const std::vector<int> &ids);

    /**
     * @brief computes the marker pose in camera coordinate
     * @param camera intrinsic matrix
     * @param camera distortion coefficients matrix
     * @param the length of the marker
    */
    void getPose(cv::Mat &camMatrix, cv::Mat &distCoeffs, float &markerLength);

    /**
     * @brief prepare the translation and quaternion to array type with the distance unit of meters
    */
    void preparePose4publish();

    /**
     * @brief transform the pose of marker in camera cooridnate to the pose of camera w.r.t to a reference cooridnate;
     * @param the rotation vector of the camera in the reference coordinate
     * @param the translation vector of the camera in the reference coordinate
    */
    void preparePose4TopView(cv::Vec3d &rvecRef, cv::Vec3d &tvecRef);

    int targetID; // the id of the marker

    /** KL tracker */
    bool trackPoint_;

    LucasKanade::LKtracker kl;
    RectTool rect_tool;

    bool has_KL;
    bool use_KL;
    cv::Rect KL_boundingbox;
    std::vector< std::vector< cv::Point > > KL_contour;
    double KL_cont_area;

    /** detector */

    int  num_detection;

    bool hasDetection; // has detection for markers
    bool hasTargetDection; // has the detection for the target marker

    std::vector< std::vector< cv::Point2f > > single_corners;

    // final detection related
    bool hasFinalDetection;
    std::vector< std::vector< cv::Point > > markerContours;

    // pose related
    cv::Vec3d rvec, tvec; // the tranform of detected marker with respect to the camera coordinate
    cv::Vec3d finalTvec,finalRvec; // the transform of detected marker with respect to the reference marker coordinate

    double trans_robot[3], quat_robot[4];

    bool usePreviousGuess; // used when estimating the pose

    int tilt_angle_count[91]; // record a histogram of the tilt angle of the detected rect ranging [0,90]
    double approx_tilt_angle;

}; //end of class

aruco_detector::aruco_detector()
{

    /**KL tracker related object**/
    trackPoint_ = true;

    has_KL = false;
    use_KL = false;

    hasDetection = false;
    hasTargetDection = false;
    hasFinalDetection = false;

    usePreviousGuess = false;

    // initialise the pose vecs
    rvec = cv::Vec3d(0.0,0.0,0.0);
    tvec= cv::Vec3d(0.0,0.0,0.0);

    for (int i = 0; i<91; i++)
    {
        tilt_angle_count[i] = 0;
    }

    approx_tilt_angle = 0.0;

}

void aruco_detector::setTargetID(int id)
{
    targetID = id;
}

void aruco_detector::detectMarkerOnce(const cv::Mat &image, const cv::Mat &grey, const cv::Mat &prevGrey, const std::vector< std::vector< cv::Point2f > > &corners, const std::vector<int> &ids)
{
    // only serve for one marker
    // cleared for each target

    single_corners.clear();
    markerContours.clear();
    KL_contour.clear();


    num_detection = (int)ids.size();
    hasDetection = num_detection>0;


    hasTargetDection = false;
    hasFinalDetection = false;
    has_KL = false;
    use_KL = false;

    // should be reset at each frame
    kl.noTracking = false;

    approx_tilt_angle = 0.0;

    if(hasDetection)
    {
        for (int i = 0; i<num_detection; i++)
        {
            // If the target ID is detected
            if (ids[i] == targetID)
            {

                hasTargetDection = true;
                hasFinalDetection = true;

                single_corners.push_back(corners[i]); // only have one detected corner in the vec

                approx_tilt_angle = rect_tool.approximateAngle(single_corners[0]);

                rect_tool.getContour(single_corners[0], markerContours);

                tilt_angle_count[(int)approx_tilt_angle]++;


                /** Initialize the feature points if necessary**/
                // get the bounding box
                if (trackPoint_)
                {
                    has_KL  = true;

                    kl.LK_tracking_ini_detection(grey, prevGrey,markerContours[0]);

                    kl.LK_bounding(KL_boundingbox);
                    kl.LK_getCornerPoints();

                    rect_tool.getContour(kl.KL_corner_points, KL_contour);
                    KL_cont_area = cv::contourArea(KL_contour[0]);
                } // end if trackPoints_
            }// end of has the target detection
        }// end of for the number of detections
    } // end of if has detection


    /** tracking the points when points are initialised */

    if ((!hasTargetDection)&&trackPoint_)
    {
        if ((!kl.hasOcclusion)&&kl.initialized&&(!kl.noTracking))
        {
            kl.LK_tracking(grey, prevGrey);
            // calculate the vector_variance
            kl.LK_checkOcclusion();

            if (!kl.hasOcclusion)
            {
                has_KL = true;

                kl.LK_bounding(KL_boundingbox);
                kl.LK_getCornerPoints();

                // update the corners
                single_corners.push_back(kl.points[0]);
                rect_tool.getContour(kl.KL_corner_points, KL_contour);
            }

        }// end of point tracking

        if (kl.hasOcclusion)
        {
            has_KL = false;
            /** can be further implemented when occlusion happens */
        }

    }// end of marker detection

    /** get the final marker detection */
    use_KL = has_KL&&(!hasTargetDection);

    if (use_KL)
    {
        hasFinalDetection = true;
        markerContours = KL_contour;
    }
}

void aruco_detector::getPose(cv::Mat &camMatrix, cv::Mat &distCoeffs, float &markerLength)
{
    std::vector<cv::Point3f> markerObjPoints;

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    markerObjPoints.push_back(cv::Point3f(-markerLength / 2.f, markerLength / 2.f, 0));
    markerObjPoints.push_back(cv::Point3f(markerLength / 2.f, markerLength / 2.f, 0));
    markerObjPoints.push_back(cv::Point3f(markerLength / 2.f, -markerLength / 2.f, 0));
    markerObjPoints.push_back(cv::Point3f(-markerLength / 2.f, -markerLength / 2.f, 0));

    if (usePreviousGuess)
    {
        cv::solvePnP(markerObjPoints,single_corners[0],camMatrix, distCoeffs,rvec, tvec,true);
    }
    else  //if not initialised
    {
        cv::solvePnP(markerObjPoints,single_corners[0],camMatrix, distCoeffs,rvec, tvec);
        usePreviousGuess = true;
    }

    // by default set the final transform with repect to the camera cooridinate
    finalRvec = rvec;
    finalTvec = tvec;
}

void aruco_detector::preparePose4publish(){
    double distance_ratio = 0.001;

    trans_robot[0] = finalTvec[0]*distance_ratio; //X (the right side from a camera view)
    trans_robot[1] = finalTvec[1]*distance_ratio; //Y (the down side from a camera view)
    trans_robot[2] = finalTvec[2]*distance_ratio; //Z (camera is looking down)

    rvec2quaternion(finalRvec,quat_robot);
}

void aruco_detector::preparePose4TopView(cv::Vec3d &rvecRef, cv::Vec3d &tvecRef){
    /* transform inverse to the actual marker */
    cv::composeRT( rvec, tvec, rvecRef, tvecRef, finalRvec, finalTvec);
}


#if CV_MINOR_VERSION >= 2

bool  readDetectorParametersVersionWrapper(cv::Ptr<cv::aruco::DetectorParameters> &detectorParams, std::string filename){
    detectorParams = cv::aruco::DetectorParameters::create();

    bool a = fileUtil::checkFileExistence(filename);
    std::cout<<"The detector file exists::"<<a<<std::endl;
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> detectorParams->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> detectorParams->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> detectorParams->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> detectorParams->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> detectorParams->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> detectorParams->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> detectorParams->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> detectorParams->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> detectorParams->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> detectorParams->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> detectorParams->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> detectorParams->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> detectorParams->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> detectorParams->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> detectorParams->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> detectorParams->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> detectorParams->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> detectorParams->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> detectorParams->minOtsuStdDev;
    fs["errorCorrectionRate"] >> detectorParams->errorCorrectionRate;

    detectorParams->doCornerRefinement = true;
    return true;
}

#else

bool  readDetectorParametersVersionWrapper(cv::aruco::DetectorParameters &detectorParams, std::string filename){

    bool a = fileUtil::checkFileExistence(filename);
    std::cout<<"The detector file exists::"<<a<<std::endl;
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
        return false;

    fs["adaptiveThreshWinSizeMin"] >> detectorParams.adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> detectorParams.adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> detectorParams.adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> detectorParams.adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> detectorParams.minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> detectorParams.maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> detectorParams.polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> detectorParams.minCornerDistanceRate;
    fs["minDistanceToBorder"] >> detectorParams.minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> detectorParams.minMarkerDistanceRate;
    fs["doCornerRefinement"] >> detectorParams.doCornerRefinement;
    fs["cornerRefinementWinSize"] >> detectorParams.cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> detectorParams.cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> detectorParams.cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> detectorParams.markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> detectorParams.perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> detectorParams.perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> detectorParams.maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> detectorParams.minOtsuStdDev;
    fs["errorCorrectionRate"] >> detectorParams.errorCorrectionRate;

    detectorParams.doCornerRefinement = true;

    return true;
}

#endif

}// end of namespace aruco_detection



#endif // ARUCO_DETECTOR_HPP_INCLUDED
