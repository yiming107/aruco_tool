/**
 * @brief  This header file implements a class for calibrating the camera pose
 *         w.r.t a reference marker which is considered as the world cooridnate.
 *
 * @author Yiming Wang (wangyimingkaren@gmail.com)
 * @date   20/05/2017
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

#ifndef DETECT_ARUCO_CALIBRATION_HPP_INCLUDED
#define DETECT_ARUCO_CALIBRATION_HPP_INCLUDED

#include <stdarg.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <resource_retriever/retriever.h>

#include "opencv2/opencv.hpp"
#include <opencv2/videoio.hpp> //for video read-write
#include <opencv2/core.hpp> // for mat
#include <opencv2/highgui.hpp> // for showing image
#include "opencv2/imgproc.hpp"

#include "aruco_detector.hpp"
#include "file_util.hpp"

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new cv_bridge API in Groovy
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_camera/ImageSync.h>
#else
// Fuerte support for cv_bridge will be deprecated
#if defined(__GNUC__)
#warning "Support for the old cv_bridge API (Fuerte) is derecated and will be removed when Hydro is released."
#endif
#include <cv_bridge/CvBridge.h>
#endif

#include <aruco_tool/ARMarkers.h> // message header


namespace aruco_detection
{

class ArucoCalibrator
{
public:
    /**
     * @brief ArucoCalibrator constructor
    */
    ArucoCalibrator (ros::NodeHandle & n);

    /**
     * @brief ArucoCalibrator destructor
    */
    ~ArucoCalibrator (void);

    /**
     * @brief initisation variables
    */
    int arucoInit ();

    /**
     * @brief write the computed camera pose into files
    */
    bool writeCameraParameters();


    int count; // record the number of frames with successful marker detection
    int maxCount; // the maximum number of marker detections, set by user in launch file with default vaule of 10

private:

    /**
     * @brief write the computed camera pose into files
     * @param message of type ImageSync
    */
    void getImageCallback (const cv_camera::ImageSync &msg);

    /**
     * @brief read the camera calibration data (intrinsics)
     * @param filename of the camera calibration parameters
    */
    bool readCameraParameters(string filename);

    /**
     * @brief compute the average of camera poses with respect to the reference marker (considered as the origin for world cooridnate)
    */
    void computeAverage();

    int selfID; // id of the machine that runs the calibration node
    int j; // sequence of the received ImageSync msg

    std::string package_path;
    ros::NodeHandle n_;
    ros::Subscriber cam_sub_; // the image message obj

    /** image related */
    cv_bridge::CvImagePtr capture_;
    cv::Mat grey, prevGrey, image, image2show; // for feature points
    int image_width, image_height;
    bool visualizeImage_;

    /** marker detector related */
    int targetID;
    float markerLength;

    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;

    aruco_detection::aruco_detector singleDetector; // for each target marker

#if CV_MINOR_VERSION >= 2
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
#else
    cv::aruco::Dictionary dictionary;
    cv::aruco::DetectorParameters detectorParams;
#endif

    std::string abs_path_cal;
    std::string abs_path_det;

    cv::Mat camMatrix, distCoeffs;
    cv::Vec3d rvecRef, tvecRef;

    std::vector<cv::Vec3d> rvecRefCache,tvecRefCache;

    bool trackPoint_;

};// end of class


void ArucoCalibrator::getImageCallback (const cv_camera::ImageSync &msg)
{
    j = msg.sequence;

    ROS_INFO("\tImage Callback Function at frame: %d", j);

    /** prepare for image*/
    try
    {
        capture_ = cv_bridge::toCvCopy (msg.image, sensor_msgs::image_encodings::BGR8);

        // CvImage.image is of type cv::Mat
        image = capture_->image;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Grey image serves for KLT
    cv::cvtColor(image, grey, CV_BGR2GRAY);
    if(prevGrey.empty())
        grey.copyTo(prevGrey);

    if (visualizeImage_)
    {
        image.copyTo(image2show);
    }


    /** Prepare detectors for markers (each detector object for detecting one marker) */
    /** In calibration case, only one detector is used as there is only one reference marker to detect*/

    ids.clear();
    corners.clear();
    rejected.clear();

    // built-in function for marker detection
    cv::aruco::detectMarkers(image,dictionary, corners, ids, detectorParams, rejected);

    // main function to detect marker that uses built-in detector together with KLT on corners
    singleDetector.detectMarkerOnce(image, grey,prevGrey,corners,ids);

    if (singleDetector.hasFinalDetection)
    {
        // get the pose of marker with respect to camera coordinate
        singleDetector.getPose(camMatrix, distCoeffs,markerLength);

        // get the pose of camera with respect to marker coordinate
        cv::Vec3d tvecRefTemp, rvecRefTemp;
        aruco_detection::inverseTransform(singleDetector.tvec, tvecRefTemp, singleDetector.rvec, rvecRefTemp);

        // saved to cache for later averaging
        rvecRefCache.push_back(rvecRefTemp);
        tvecRefCache.push_back(tvecRefTemp);

        count++;

        if (visualizeImage_)
        {
            std::ostringstream str;
            str <<"Successful detection:"<<count;

            cv::putText(image2show,str.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));

            cv::drawContours(image2show,singleDetector.markerContours,0, cv::Scalar(255, 0, 0),2,8);
            cv::aruco::drawAxis(image2show, camMatrix, distCoeffs,singleDetector.rvec,singleDetector.tvec,markerLength * 0.5f);

        }
    }

    if (visualizeImage_){
        cv::imshow("Deteciton", image2show); // choose which image to show
        int waitTime = 100; // video file
        cv::waitKey(waitTime);
    }
    // swap gray and prevGray after each frame
    cv::swap(prevGrey, grey);
}

ArucoCalibrator::ArucoCalibrator(ros::NodeHandle & n):n_ (n)
{
    package_path = ros::package::getPath (ROS_PACKAGE_NAME);

    ros::NodeHandle n_param ("~"); // for reading parameters from launch file
    XmlRpc::XmlRpcValue xml_marker_center;

    std::cout<<"Test::package_path"<<package_path<<std::endl;


    ROS_INFO("Starting ArucoCalibrator");


    if (!n_param.getParam("marker_length", markerLength))
        markerLength = 100.0;
    ROS_INFO ("\tMarker Length: %.1f", markerLength);


    if (!n_param.getParam("track_point", trackPoint_))
        trackPoint_ = true;
    ROS_INFO("\tTrack point: %d", trackPoint_);

    if (!n_param.getParam("visualize_image", visualizeImage_))
        visualizeImage_ = true;
    ROS_INFO("\tVisualize image: %d", visualizeImage_);

    n_param.getParam("camera_cal_path", abs_path_cal);
    ROS_INFO("\tAbsolute calibration file path:");
    std::cout<<abs_path_cal<<std::endl;

     n_param.getParam("detector_param_path", abs_path_det);
     ROS_INFO("\tAbsolute detector param file path:");
     std::cout<<abs_path_det<<std::endl;

     if (!n_param.getParam("image_width", image_width))
        image_width = 640;
     ROS_INFO("\tImage_width: %d", image_width);

     if (!n_param.getParam("image_height", image_height))
        image_height = 480;
     ROS_INFO("\tImage_height: %d", image_height);

    if (!n_param.getParam("targetID", targetID))
        targetID = 1;
    ROS_INFO("\ttargetID: %d", targetID);

    if (!n_param.getParam("maxCount", maxCount))
        maxCount = 10;
    ROS_INFO("\tmaxCount: %d", maxCount);

    if (!n_param.getParam("selfID", selfID))
        selfID = 1;
    ROS_INFO("\tselfID: %d", selfID);

    j=0;
    count = 0;
}

ArucoCalibrator::~ArucoCalibrator (void)
{
   //cv::VideoCapture::release();
}

int ArucoCalibrator::arucoInit ()
{

    std::cout<<"Opencv version"<<CV_VERSION<<std::endl;
    ROS_INFO ("\tInside the arucoInit...");

    /** marker deteciton related **/
    // initialise the marker dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    bool readOk = readCameraParameters(abs_path_cal);
    if(!readOk)
    {
        cerr << "Invalid camera file" << endl;
        return -1;
    }

    ROS_INFO ("\tSuccessfully read camera parameters");

    // parepare the detector parameters //
    readOk = aruco_detection::readDetectorParametersVersionWrapper(detectorParams,abs_path_det);
    if(!readOk)
    {
        cerr << "Invalid detector parameters file" << endl;
        return -1;
    }
    ROS_INFO ("\tSuccessfully read detector parameters");


    singleDetector.setTargetID(targetID);

    rvecRefCache.clear();
    tvecRefCache.clear();

    /** initialise subscribers/publishers **/

    ROS_INFO ("Subscribing to image topic");

    std::ostringstream str;
    str <<"camera"<<selfID<<"/image_raw";
    ROS_INFO("\tCamera %d subscribing to image topic...", selfID);
    cam_sub_ = n_.subscribe (str.str(), 1, &ArucoCalibrator::getImageCallback, this);
}

bool ArucoCalibrator::readCameraParameters(string filename)
{
    std::cout<<filename<<std::endl;
    cv::FileStorage fs;

    fs.open(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    return true;
}

bool ArucoCalibrator::writeCameraParameters()
{
    cv::FileStorage fs;

    fs.open(abs_path_cal, cv::FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    computeAverage();

    cv::Mat temprvecMat = aruco_detection::DoubleMatFromVec3d(rvecRef);
    cv::Mat temptvecMat = aruco_detection::DoubleMatFromVec3d(tvecRef);
    fs << "rvecRef" << temprvecMat;
    fs << "tvecRef" << temptvecMat;

    // rewrite as the WRITE mode will delete previous info
    fs << "camera_matrix" <<camMatrix;
    fs << "distortion_coefficients" <<distCoeffs;

    return true;
}

void ArucoCalibrator::computeAverage()
{
    cv::Vec3d rvecRefSum,tvecRefSum;
    for(int i = 0;i<count;++i){
        rvecRefSum = rvecRefSum + rvecRefCache[i];
        tvecRefSum = tvecRefSum + tvecRefCache[i];
    }

    rvecRef = rvecRefSum/(double)count;
    tvecRef = tvecRefSum/(double)count;
}


}// end of namespace

#endif
