/**
 * @brief  ArucoPublisherTopView class for detecting and estimating the pose of multiple markers w.r.t a reference coordinate and
 *         publishing the estimated marker poses to other nodes for further multi-robot control.
 *
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

#ifndef DETECT_ARUCO_ROS_TOPVIEW_HPP_INCLUDED
#define DETECT_ARUCO_ROS_TOPVIEW_HPP_INCLUDED

#include <stdarg.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
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


/**
 * @brief compute the 2D pose, i.e. x,y,theta (yaw angle) from a 3D Pose msg
 * @param input 3D Pose msg
 * @param output 2D pose in vector
 */
void getPose2D(const geometry_msgs::Pose & input_p3d, std::vector<float> & output_p2d)
{

    output_p2d.clear();
    double x,y,z,roll, pitch, yaw;
    x = input_p3d.position.x;
    y = input_p3d.position.y;
    z = input_p3d.position.y;

    tf::Quaternion rotation (input_p3d.orientation.x, input_p3d.orientation.y, input_p3d.orientation.z, input_p3d.orientation.w);
    tf::Vector3 origin (x, y, z);
    tf::Transform t (rotation, origin);

    t.getBasis().getEulerYPR(yaw,pitch,roll);

    output_p2d.push_back((float)x);
    output_p2d.push_back((float)y);
    output_p2d.push_back((float)yaw); // here yaw range is within (-180~180]
}

namespace aruco_detection
{

/**
 * @brief ArucoPublisherTopView class
 */
class ArucoPublisherTopView
{
public:

    /**
     * @brief ArucoPublisherTopView class constructor
     */
    ArucoPublisherTopView (ros::NodeHandle & n);

    /**
     * @brief ArucoPublisherTopView class destructor
     */
    ~ArucoPublisherTopView (void);

    /**
     * @brief initialise the variables
     */
    int arucoInit ();


private:

    /**
     * @brief call back function when receiving an ImageSync msg
     * @param the received ImageSync msg
     */
    void getImageCallback (const cv_camera::ImageSync &msg);

    /**
     * @brief prepare the pose of one marker into the message that encapsulates poses of multiple markers
     * @param the aruco_detector object for the marker
     */
    void preparePoseMessage (aruco_detection::aruco_detector &a);

    /**
     * @brief prepare and publish the pose of one marker to tf
     * @param the aruco_detector object for the marker
     */
    void publish2tf (aruco_detection::aruco_detector &a);

    /**
     * @brief read the camera parameters
     * @param the file name in absolute path
     */
    bool readCameraParameters(string filename);

    /**
     * @brief save data or video for one marker
     * @param the index of the marker detector object
     */
    void saveDataVideoSingle(int k);

    /**
     * @brief save all data/videos and visualise the images if visualisation is enabled
     */
    void saveDataVideo();

    int selfID;

    std::string package_path;
    // ros node
    ros::NodeHandle n_;
    // Topic related variables
    tf::TransformBroadcaster broadcaster_; // the tf obj
    ros::Publisher arMarkerPub_; // the publisher obj

    aruco_tool::ARMarkers arPoseMarkers_;

    ros::Subscriber cam_sub_; // the image message obj

    cv_bridge::CvImagePtr capture_;

    bool publishTf_;

    /** parameters from old file**/
    double videoFrameRate;
    cv::Mat image, imageCopy1, imageCopy2, imageCopy3, imageCopy4;

    /**Performance related variables**/
    bool saveData_;
    std::string data_folder_name;
    std::string time_file_name, measure_file_name;
    std::vector<std::vector<string> > data_name;
    int num_data_files;

    /**Ouput Video related variables**/
    bool saveVideo_;
    int num_video_files;
    int codec;  // select desired codec (must be available at runtime)
    int image_width,image_height;

    std::vector<cv::VideoWriter> writers;
    std::string generated_folder_name;

    cv::Mat grey, prevGrey; // for feature points

    /** marker deteciton related **/
    std::vector<int> targetIDs;
    int num_targets; // shoule be consistent with ID
    float markerLength;

    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;

    std::vector<aruco_detection::aruco_detector> singleDetectors; // for each target marker

#if CV_MINOR_VERSION >= 2
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
#else
    cv::aruco::Dictionary dictionary;
    cv::aruco::DetectorParameters detectorParams;
#endif

    std::string videoname;
    std::string abs_path_cal;
    std::string abs_path_det;

    cv::Mat camMatrix, distCoeffs;
    cv::Vec3d rvecRef, tvecRef;

    bool trackPoint_;
    /**Visualization related**/
    bool visualizeImage_;

    int j;

    std::string frame_id;
    ros::Time captureSentTime;
    ros::Time capturedTime;
    ros::Time receivedTime;
};// end of class


void ArucoPublisherTopView::getImageCallback (const cv_camera::ImageSync &msg)
{
    j = msg.sequence;

    frame_id = msg.image.header.frame_id;
    captureSentTime = msg.image.header.stamp;
    capturedTime = msg.captureTime;
    receivedTime = ros::Time::now();

    ROS_INFO("\tCamera %d received image at frame: %d", selfID, j);

    /** prepare for cv image*/
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

    // Grey image serves for KLT tracking
    cv::cvtColor(image, grey, CV_BGR2GRAY);
    if(prevGrey.empty())
        grey.copyTo(prevGrey);

    if (saveVideo_||visualizeImage_)
    {
        image.copyTo(imageCopy1);
        image.copyTo(imageCopy2);
        image.copyTo(imageCopy3);
        image.copyTo(imageCopy4);
    }

    /** Prepare detectors for markers (each detector object for detecting one marker) */

    ids.clear();
    corners.clear();
    rejected.clear();
    
    // clear the poses in the Messags
    arPoseMarkers_.markers.clear();

    // built-in function for marker detection
    cv::aruco::detectMarkers(image,dictionary, corners, ids, detectorParams, rejected);

    // iterate for each marker
    for (int k = 0; k<num_targets; k++)
    {
        /** Target Marker detection **/
        singleDetectors[k].detectMarkerOnce(image, grey,prevGrey,corners,ids);

        if (singleDetectors[k].hasFinalDetection)
        {
            singleDetectors[k].getPose(camMatrix, distCoeffs,markerLength);
            singleDetectors[k].preparePose4TopView(rvecRef,tvecRef);
            singleDetectors[k].preparePose4publish();
            
            if(publishTf_)
            {
                publish2tf(singleDetectors[k]);
            }

        }

        // prepare marker messages no matter it has final detection or not
        preparePoseMessage(singleDetectors[k]);

        // save raw data/video for each target marker
        saveDataVideoSingle(k);

    }

    /** publish marker messages */
    ros::Time currentTime = ros::Time::now();

    arPoseMarkers_.header.frame_id = frame_id;
    arPoseMarkers_.header.stamp    = currentTime;
    arPoseMarkers_.sequence = j;
    arPoseMarkers_.captureTime = capturedTime;

    arMarkerPub_.publish(arPoseMarkers_);

    /** save data and videos*/

    ros::Duration durReceiving = receivedTime - captureSentTime;
    ros::Duration durDetection = currentTime - receivedTime;
    ros::Duration durOverall = currentTime - capturedTime;

    if (saveData_){
        std::ostringstream str2write;
        str2write<<j<<" "<<durReceiving.toSec()<<" "<<durDetection.toSec()<<" "<<durOverall.toSec()<<"\n";
        fileUtil::write2file(time_file_name.c_str(), str2write.str());
    }

    saveDataVideo();

    // swap gray and prevGray after each frame
    cv::swap(prevGrey, grey);
}

ArucoPublisherTopView::ArucoPublisherTopView(ros::NodeHandle & n):n_ (n)
{

    // package_path = ros::package::getPath (ROS_PACKAGE_NAME); // save to this package
    package_path = ros::package::getPath ("exp_setup"); // save to other ros package

    ros::NodeHandle n_param ("~"); // for reading parameters from launch file

    //XmlRpc::XmlRpcValue xml_marker_center;

    std::cout<<"Test::package_path" << package_path<<std::endl;

    data_folder_name = "/savedData/";
    generated_folder_name = "/savedVideo/";

    num_data_files = 4;
    num_video_files = 5;

    codec = CV_FOURCC('X', '2', '6', '4');

    ROS_INFO("Starting ArucoPublisherTopView");

    // **** get parameters
    if (!n_param.getParam("videoFrameRate", videoFrameRate))
        videoFrameRate = 10.0;
    ROS_INFO ("\tvideoFrameRate: %.1f", videoFrameRate);

    if (!n_param.getParam("publish_tf", publishTf_))
        publishTf_ = true;
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    if (!n_param.getParam("marker_length", markerLength))
        markerLength = 100.0;
    ROS_INFO ("\tMarker Length: %.1f", markerLength);

    if (!n_param.getParam("save_data", saveData_))
        saveData_ = true;
    ROS_INFO("\tSave data: %d", saveData_);

    if (!n_param.getParam("save_video", saveVideo_))
        saveVideo_ = true;
    ROS_INFO("\tSave video: %d", saveVideo_);

    if (!n_param.getParam("track_point", trackPoint_))
        trackPoint_ = true;
    ROS_INFO("\tTrack point: %d", trackPoint_);

    if (!n_param.getParam("visualize_image", visualizeImage_))
        visualizeImage_ = true;
    ROS_INFO("\tVisualize image: %d", visualizeImage_);

    n_param.getParam("saved_video_name", videoname);
    ROS_INFO("\tVideo name:");
    std::cout<<videoname<<std::endl;

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

    if (!n_param.getParam("selfID", selfID))
        selfID = 1;
    ROS_INFO("\tCamera ID: %d", selfID);


    /// How to read target IDs
    if (!n_param.getParam("markerIDs", targetIDs))
        targetIDs.push_back(1);

    num_targets = (int)targetIDs.size();

    ROS_INFO("\tTarget number: %d",  num_targets);

    for (int i=0; i < targetIDs.size(); i++)
    {
        ROS_INFO("\tmarkerIDs: %d", targetIDs[i]);
    }
    j=0;
}


ArucoPublisherTopView::~ArucoPublisherTopView (void)
{
    //cv::VideoCapture::release();
}

int ArucoPublisherTopView::arucoInit ()
{
    ROS_INFO ("\tInside the arucoInit...");

    /** initialise video writers */
    std::vector<string> added_name_map;
    if (saveVideo_)
    {
        ROS_INFO ("\tSaving to videos");
        cv::Size video_size(image_width,image_height);
        added_name_map.clear();
        added_name_map.push_back("_onlyDetec");
        added_name_map.push_back("_withfalse");
        added_name_map.push_back("_onlyKL");
        added_name_map.push_back("_final");
        added_name_map.push_back("_rawVideo");

        for(int n = 0; n<num_video_files; n++)
        {
            cv::VideoWriter temp;
            string ext = ".avi";

            // check the file name instead of overwriting existing files
            std::string filename_temp = fileUtil::checkFileName(package_path+generated_folder_name+videoname+added_name_map[n],ext);

            // // overwrite existing files
            //string filename_temp = package_path+generated_folder_name+videoname+added_name_map[n]+ext;

            temp.open(filename_temp, codec, videoFrameRate, video_size, 1);

            if (!temp.isOpened())
            {
                cerr << "Could not open the output video file for write\n";
                return -1;
            }
            else
            {
                writers.push_back(temp);
            }

        }

    }else{
        ROS_INFO ("Not saving to videos");
    }


    /** marker deteciton related */

    // initialise the marker dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    bool readOk = readCameraParameters(abs_path_cal);
    if(!readOk)
    {
        cerr << "Invalid camera file" << endl;
        return -1;
    }

    ROS_INFO ("\tSuccessfully read camera parameters");

    // parepare the detector parameters
    readOk = aruco_detection::readDetectorParametersVersionWrapper(detectorParams,abs_path_det);

    if(!readOk)
    {
        cerr << "Invalid detector parameters file" << endl;
        return -1;
    }
    ROS_INFO ("\tSuccessfully read detector parameters");


    /** prepare file names for saving raw data */
    // prepare the file to save temporary data
    if (saveData_)
    {

        ROS_INFO ("\tSaving to files");
        added_name_map.clear();
        added_name_map.push_back("_detection");
        added_name_map.push_back("_cornerLoc");
        added_name_map.push_back("_translation");
        added_name_map.push_back("_rotation");

        string ext = ".txt";
        for(int k = 0; k<num_targets; k++)
        {
            std::vector<string> temp_data_name;
            std::string subfolder_name = "detector/temp/";
            for (int n = 0; n<num_data_files; n++)
            {

                string temp_file_name = package_path+data_folder_name+ subfolder_name+videoname +added_name_map[n]+to_string(targetIDs[k])+ext;
                temp_data_name.push_back(temp_file_name);
                std::cout<<temp_file_name<<std::endl;
                fileUtil::cleanfile(temp_file_name.c_str());
            }

            data_name.push_back(temp_data_name);
        }

        //prepare the file to save time

        time_file_name = fileUtil::checkFileName(package_path+data_folder_name+"detector/time/detectionTime_"+to_string(selfID),ext);
        //time_file_name = package_path+data_folder_name+"detector/time/detectionTime_"+to_string(selfID)+ext;

        fileUtil::cleanfile(time_file_name.c_str());

        //prepare the file to save the measurements (pose in 2D) for each marker

        measure_file_name = fileUtil::checkFileName(package_path+data_folder_name+"detector/measurement/measurement_"+to_string(selfID),ext);
        //measure_file_name = package_path+data_folder_name+"detector/measurement/measurement_"+to_string(selfID)+ext;

        fileUtil::cleanfile(measure_file_name.c_str());

    }else{
        ROS_INFO ("\tSaving to raw data files");
    }

    /** initiliaze the single aruco detector object **/
    for(int k = 0; k<num_targets; k++)
    {
        aruco_detection::aruco_detector temp_ad;
        temp_ad.setTargetID(targetIDs[k]);
        //std::cout<<"ID"<<targetIDs[k]<<std::endl;
        singleDetectors.push_back(temp_ad);
    }

    // advertsie multiple marker message
    std::ostringstream str1;
    str1 <<"camera"<<selfID<<"/measurement";
    ROS_INFO("\tCamera %d publish to measurement topic...", selfID);
    arMarkerPub_ = n_.advertise < aruco_tool::ARMarkers > (str1.str(), 1);

    std::ostringstream str;
    str <<"camera"<<selfID<<"/image_raw";
    ROS_INFO("\tCamera %d subscribing to image topic...", selfID);
    cam_sub_ = n_.subscribe (str.str(), 1, &ArucoPublisherTopView::getImageCallback, this);

}


void ArucoPublisherTopView::preparePoseMessage (aruco_detection::aruco_detector &a)
{

    aruco_tool::ARMarker ar_pose_marker_;

    ar_pose_marker_.id              = a.targetID;
    ar_pose_marker_.hasDetection = a.hasFinalDetection;

    geometry_msgs::Pose poseTemp;

    if (a.hasFinalDetection){

        poseTemp.position.x = a.trans_robot[0];
        poseTemp.position.y = a.trans_robot[1];
        poseTemp.position.z = a.trans_robot[2];

        poseTemp.orientation.x = a.quat_robot[0];
        poseTemp.orientation.y = a.quat_robot[1];
        poseTemp.orientation.z = a.quat_robot[2];
        poseTemp.orientation.w = a.quat_robot[3];

    }else{
        poseTemp.position.x = 0.0;
        poseTemp.position.y = 0.0;
        poseTemp.position.z = 0.0;

        poseTemp.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

    }

    ar_pose_marker_.pose.pose = poseTemp;
    arPoseMarkers_.markers.push_back (ar_pose_marker_);

    // save to file
    std::vector<float> pose2d;
    getPose2D(poseTemp, pose2d);

    ROS_INFO("\t Camera %d detects marker %d at (x %f, y %f, theta %f) at sequence %d", (int)selfID, (int)a.targetID,pose2d[0], pose2d[1], pose2d[2]*180/M_PI,j);
    if (saveData_){
        std::ostringstream str2write;
        // for detection count
        str2write<<j<<" "<<a.targetID<<" "<<a.hasFinalDetection<<" "<<pose2d[0]<<" "<<pose2d[1]<<" "<<pose2d[2]<<"\n";
        fileUtil::write2file(measure_file_name.c_str(), str2write.str());
    }
}

void ArucoPublisherTopView::publish2tf (aruco_detection::aruco_detector &a)
{
    tf::Quaternion rotation (a.quat_robot[0], a.quat_robot[1], a.quat_robot[2], a.quat_robot[3]);
    tf::Vector3 origin (a.trans_robot[0], a.trans_robot[1], a.trans_robot[2]);
    tf::Transform t (rotation, origin);
    
    string name = "marker_"+to_string(a.targetID);

    // transform from camera to odom
    tf::StampedTransform markerToCam (t, capturedTime, frame_id,name.c_str());
    broadcaster_.sendTransform(markerToCam);
}

bool ArucoPublisherTopView::readCameraParameters(string filename)
{
    std::cout<<filename<<std::endl;
    cv::FileStorage fs;

    fs.open(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    /*Read rvecRef and tvecRef*/
    cv::Mat tempRvecMat, tempTvecMat;
    fs["rvecRef"] >> tempRvecMat;
    fs["tvecRef"] >> tempTvecMat;

    rvecRef = aruco_detection::Vec3dFromDoubleMat(tempRvecMat);
    tvecRef = aruco_detection::Vec3dFromDoubleMat(tempTvecMat);

    return true;
}

void ArucoPublisherTopView::saveDataVideo()
{
    if (saveVideo_||visualizeImage_)
    {
        if(rejected.size() > 0)
            cv::aruco::drawDetectedMarkers(imageCopy2, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

        if (ids.size()>0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy1, corners, ids);
        }

        std::ostringstream str;
        str <<"Frame:"<<j;

        cv::putText(imageCopy1,str.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        cv::putText(imageCopy2,str.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        cv::putText(imageCopy3,str.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        cv::putText(imageCopy4,str.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        
    }

    if (saveVideo_)
    {
        writers[0].write(imageCopy1);
        writers[1].write(imageCopy2);
        writers[2].write(imageCopy3);
        writers[3].write(imageCopy4);
        writers[4].write(image);

        ROS_INFO("Camera %d saving video at sequence %d", (int)selfID,j);
    }

    /** show image during run time */
    if (visualizeImage_)
    {
        {
            cv::line(imageCopy4,cv::Point(0,int(image_height/2)), cv::Point(image_width,int(image_height/2)),cv::Scalar(0,0,0));
            cv::line(imageCopy4,cv::Point(int(image_width/2),0), cv::Point(int(image_width/2),image_height),cv::Scalar(0,0,0));
            cv::imshow("final_Deteciton", imageCopy4); // choose which image to show
            int waitTime = 100; // video file

            cv::waitKey(waitTime);
        }
    }
}

void ArucoPublisherTopView::saveDataVideoSingle(int k)
{
    /** data */
    if (saveData_)
    {
        // write to the file
        std::ostringstream str2write[4];
        // for detection count
        str2write[0]<<"\n"<<j<<" "<<(int) singleDetectors[k].hasDetection<<" "<<(int)singleDetectors[k].hasFinalDetection<<" "<<(int)singleDetectors[k].kl.hasOcclusion;

        // for corner positions;
        for (int m = 1; m<4; m++)
        {
            str2write[m]<<"\n"<<j<<" "<<(int)singleDetectors[k].hasFinalDetection<<" ";
        }

        if(singleDetectors[k].hasFinalDetection)
        {

            for(int m = 0; m<4; m++)
            {

                str2write[1]<<singleDetectors[k].single_corners[0][m].x<<" ";
                str2write[1]<<singleDetectors[k].single_corners[0][m].y<<" ";
            }

            for(int m = 0; m<3; m++)
            {
                str2write[2]<<singleDetectors[k].trans_robot[m]<<" ";
            }

        }
        else
        {
            for(int m = 0; m<4; m++)
            {
                str2write[1]<<0<<" ";
                str2write[1]<<0<<" ";
            }

            for(int m = 0; m<3; m++)
            {
                str2write[2]<<0<<" ";

            }

        }

        if(singleDetectors[k].hasTargetDection)
        {
            str2write[3]<<singleDetectors[k].approx_tilt_angle<<" ";

        }
        else
        {
            str2write[3]<<(-1)<<" ";
        }


        for (int m = 0; m<4; m++)
        {
            fileUtil::write2file(data_name[k][m].c_str(), str2write[m].str());
        }
    }

    /** video */
    if (saveVideo_||visualizeImage_)
    {
        if (singleDetectors[k].hasFinalDetection)
        {

            if (singleDetectors[k].use_KL)
            {
                cv::drawContours(imageCopy4,singleDetectors[k].markerContours,0, cv::Scalar(255,0,255),2,8);
            }
            else
            {
                cv::drawContours(imageCopy4,singleDetectors[k].markerContours,0, cv::Scalar(255, 0, 0),2,8);
            }

            cv::aruco::drawAxis(imageCopy4, camMatrix, distCoeffs,singleDetectors[k].rvec,singleDetectors[k].tvec,markerLength * 0.5f);
        }

        if (singleDetectors[k].has_KL)
        {
            /**All visualization happens here**/
            singleDetectors[k].kl.drawFeaturePoints(imageCopy3);

            cv::rectangle(imageCopy3,singleDetectors[k].KL_boundingbox, cv::Scalar(255,0,255),2,8);
            cv::drawContours(imageCopy3,singleDetectors[k].KL_contour,0, cv::Scalar(0,0,255),2,8);
        }



        if (singleDetectors[k].kl.hasOcclusion)
        {
            //******Something will happend here*****/
            std::ostringstream str;
            str <<"Occlusion on "<<singleDetectors[k].targetID<<" !";
            // draw at marker center
            cv::putText(imageCopy3,str.str(), cv::Point(100,20*(k+1)), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
            cv::putText(imageCopy4,str.str(), cv::Point(100,20*(k+1)), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        }

    } //end of if save video

}

}// end of namespace

#endif
