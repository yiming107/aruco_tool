/**
* @brief   defines LKtracker class for key points tracking (in our case, the marker corner tracking)
*           built on top of OpenCV sample code for tracking
*
* @author  Yiming Wang (wangyimingkaren@gmail.com)
* @date    2/11/2016
*/

#ifndef LK_UTIL_HPP_INCLUDED
#define LK_UTIL_HPP_INCLUDED

#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ctype.h>

#include <vector>
#include <math.h>
#include <iterator>
#include <algorithm>
#include <set>
#include <string>


namespace LucasKanade
{

/**
*  @brief   declare the LKtracker class
*/
class LKtracker
{
public:
    int MAX_COUNT;
    cv::Size subPixWinSize;
    cv::Size winSize;
    cv::TermCriteria termcrit;

    std::vector<cv::Point2f> points[2];
    std::vector<cv::Point2f> KL_corner_points;
    std::vector<float> motionVec[2];
    float vector_variance;

    bool noTracking; // if ini then no need for tracking
    bool hasOcclusion;
    bool initialized;

    /**
    * @brief   LKtracker constructor
    */
    LKtracker();

    /**
    * @brief   initialise the good points by detecting within ROI
    */
    void LK_tracking_ini_goodpoints( const cv::Mat &gray, const cv::Mat &prevGray, cv::Rect &roi);

    /**
    * @brief   initialise the good points by providing the courner points
    */
    void LK_tracking_ini_detection( const cv::Mat &gray, const cv::Mat &prevGray, std::vector<cv::Point> &detection_points);

    /**
    * @brief   check if the tracking is needed. If a detection is available then no need to track the points
    */
    bool LK_checkTrackingNeed();

    /**
    * @brief   track the points between two consecutive points
    */
    void LK_tracking( const cv::Mat &gray, const cv::Mat &prevGray);

    /**
    * @brief   check if there is occlusion on the marker
    */
    void LK_checkOcclusion();

    /**
    * @brief   obtain the bounding Rect (rotated) from the points
    */
    void LK_bounding(cv::Rect &LK_Rect);

    /**
    * @brief   extract the four corner points from tracked points
    */
    void LK_getCornerPoints();

    /**
    * @brief   refine the points using points from detection, i.e. the corners
    */
    void LK_refinePoints(std::vector<cv::Point> &detection_points);

    /**
    * @brief   computes how consistent the motion of all points (currently using variance for translation only, need to be improved for rotation)
    * @return  the variance of one dimension of the motion vector, either delta x or delta y.
    */
    float LK_motion_correlation_single(std::vector<float> &mv); // for one dimension of the motion vector
    void LK_motion_correlation(); // for all dimensions fo the motion vector

    /**
    *@brief   visualise the tracked points on a provided image
    */
    void drawFeaturePoints(cv::Mat &image);
}; // end of class declaration


LucasKanade::LKtracker::LKtracker()
{
    MAX_COUNT = 500;
    subPixWinSize = cv::Size(10,10);
    winSize = cv::Size(31,31);
    termcrit =cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    vector_variance = 0;
    hasOcclusion = false;
    initialized = false;
    noTracking = false;
}

void LucasKanade::LKtracker::LK_tracking_ini_detection(const cv::Mat &gray, const cv::Mat &prevGray,std::vector<cv::Point> &detection_points)
{

    points[0].clear();
    points[1].clear();

    LK_refinePoints(detection_points);


    motionVec[0].clear();
    motionVec[1].clear();
    // go back to the point position on the original image
    for( int i__ = 0; i__ < points[1].size(); i__++ )
    {
        motionVec[0].push_back(0);
        motionVec[1].push_back(0);
    }

    initialized = true;
    hasOcclusion = false;
    noTracking = true;

    //std::cout<<"I am in good INI::"<<" hasOcclusion::"<<hasOcclusion<<std::endl;
    std::swap(points[1], points[0]);
    //cv::swap(prevGray, gray);
}

void LucasKanade::LKtracker::LK_tracking_ini_goodpoints( const cv::Mat &gray, const cv::Mat &prevGray, cv::Rect &roi)
{

    cv::Mat imageROI = gray(roi);
    // take the first frame, detect some Shi-Tomasi corner points in it, and then iteratively track those points using Lucas-Kanade optical flow
    points[0].clear();
    points[1].clear();

    cv::goodFeaturesToTrack(imageROI, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);

    motionVec[0].clear();
    motionVec[1].clear();
    // go back to the point position on the original image
    for( int i__ = 0; i__ < points[1].size(); i__++ )
    {
        points[1][i__].x = points[1][i__].x+roi.x;
        points[1][i__].y = points[1][i__].y +roi.y;
        motionVec[0].push_back(0);
        motionVec[1].push_back(0);
    }

    cv::cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1,-1), termcrit);

    std::swap(points[1], points[0]);

    initialized = true;
    hasOcclusion = false;
    noTracking = true;
    //cv::swap(prevGray, gray);
}

void LucasKanade::LKtracker::LK_tracking( const cv::Mat &gray, const cv::Mat &prevGray)
{

    if( !points[0].empty() ) // when previous frame has points to track
    {

        std::vector<uchar> status;
        std::vector<float> err;
        motionVec[0].clear();
        motionVec[1].clear();

        //pass the previous frame, previous points and next frame.
        //returns next points along with some status numbers which has a value of 1 if next point is found, else zero
        cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,3, termcrit, 0, 0.001);

        size_t i__, k__;

        for( i__ = k__ = 0; i__ < (int)points[1].size(); i__++ )
        {
            // eliminates any points in the array where status[i] is false by copying over them, and then setting the length of the array (count) to k, the number
            if( !status[i__] )
                continue;

            points[1][k__++] = points[1][i__];

            float displacement_x = points[1][i__].x-points[0][i__].x;
            float displacement_y = points[1][i__].y-points[0][i__].y;

            motionVec[0].push_back(displacement_x);
            motionVec[1].push_back(displacement_y);
        }
        points[1].resize(k__);
    }
    else
    {

        motionVec[0].clear();
        motionVec[1].clear();
    }

    std::swap(points[1], points[0]);
}

void  LucasKanade::LKtracker::LK_checkOcclusion()
{
    // Hint, is it possible to get the points not following majority? outsider
    LK_motion_correlation();
    // update the hasOcclusion here
    bool bad_motion = vector_variance>30;
    bool bad_corner_num = points[0].size()<4;

    // update if occlusion happened
    hasOcclusion = bad_motion||bad_corner_num;

    if (hasOcclusion)
    {
        initialized = false;
    }

}

void LucasKanade::LKtracker::LK_bounding(cv::Rect &LK_Rect)
{
    int min_x, max_x, min_y, max_y;

    for( int i__ = 0; i__ < (int)points[0].size(); i__++ )
    {

        if (i__==0)
        {
            max_x = points[0][i__].x;
            max_y = points[0][i__].y;

            min_x = points[0][i__].x;
            min_y = points[0][i__].y;
        }


        if (points[0][i__].x>max_x)
        {
            max_x = points[0][i__].x;
        }

        if (points[0][i__].y>max_y)
        {
            max_y = points[0][i__].y;
        }

        if (points[0][i__].x<min_x)
        {
            min_x = points[0][i__].x;
        }

        if (points[0][i__].y<min_y)
        {
            min_y = points[0][i__].y;
        }

        LK_Rect.x = int (min_x);

        LK_Rect.y = int(min_y);


        LK_Rect.width = int(max_x-min_x+1);

        LK_Rect.height = int(max_y-min_y+1);
    }
}

void LucasKanade::LKtracker::LK_getCornerPoints()
{

// // when there are many points (not only 4 corner points)

//    int record_index[4] = {0,0,0,0}; // Have to initialize, otherwise segmentation fault;
//
//    float min_x, max_x, min_y, max_y;
//
//    for( int i = 0; i < (int)points.size(); i++ )
//    {
//
//        if (i==0)
//        {
//            max_x = points[i].x;
//            max_y = points[i].y;
//
//            min_x = points[i].x;
//            min_y = points[i].y;
//        }
//
//
//        if (points[i].x>max_x)
//        {
//            max_x = points[i].x;
//            record_index[2] = i; //Right_corner
//        }
//
//        if (points[i].y>max_y)
//        {
//            max_y = points[i].y;
//            record_index[3] = i; //Down_corner
//        }
//
//        if (points[i].x<min_x)
//        {
//            min_x = points[i].x;
//            record_index[0] = i;//Left_corner
//        }
//
//        if (points[i].y<min_y)
//        {
//            min_y = points[i].y;
//            record_index[1] = i;//Up_corner
//        }
//
//    }//end of for
//
//    std::vector<int> c_ids;
//
//    bool existing;
//
//    for(int j=0; j<4; j++)
//    {
//        int temp = record_index[j];
//        //std::cout<<"About to insert:"<<temp<<std::endl;
//
//        existing = std::find(c_ids.begin(), c_ids.end(), temp) != c_ids.end();
//
//        if (!existing) // if not exists
//        {
//            c_ids.push_back(temp);
//            //std::cout<<"Insert:"<<temp<<std::endl;
//            c_points.push_back(points[temp]);
//        }
//    }
//
//    std::cout<<"Hello::"<<c_points.size()<<std::endl;

    // when points only have 4 corners

    KL_corner_points.clear();
    int num = (int)points[0].size();
    for(int i__ = 0; i__<num; i__++)
    {
        KL_corner_points.push_back(points[0][i__]);
    }

}

float LucasKanade::LKtracker::LK_motion_correlation_single(std::vector<float> &mv)
{

    float sum = 0, mean =0, temp_variance=0,variance=0;

    int motion_num = mv.size();

    if (motion_num>0)
    {

        for( int i__ = 0; i__ < motion_num; i__++ )
        {

            sum += mv[i__];
        }

        mean= sum / motion_num;

        for(int i__= 0; i__ < motion_num; i__++)

        {
            temp_variance += (mv[i__] - mean) * (mv[i__] - mean) ;

        }

        variance=temp_variance/motion_num;

    }
    else
    {
        variance = -1;
    }

    return variance;
}

void LucasKanade::LKtracker::LK_motion_correlation()
{

    float variance[2]= {0,0};
    variance[0] = LK_motion_correlation_single(motionVec[0]);
    variance[1] = LK_motion_correlation_single(motionVec[1]);

    if (variance[1]>variance[0])
    {
        vector_variance= variance[1];
    }
    else
    {
        vector_variance= variance[0];
    }

}

void LucasKanade::LKtracker::LK_refinePoints(std::vector<cv::Point> &detection_points)
{

    points[1].clear();

    for (int i__ = 0; i__<(int)detection_points.size(); i__++)
    {
        points[1].push_back(detection_points[i__]);
    }

}

void LucasKanade::LKtracker::drawFeaturePoints(cv::Mat &image)
{
    if (!points[0].empty())
    {

        // draw each point
        int  points_num= points[0].size();
        int  color_gap = int(255/points_num);

        for (int i__ = 0; i__ < points_num; i__++ )
        {
            cv::circle(image, points[0][i__],3, cv::Scalar(0,color_gap*i__,255),2);

            cv::Point2f startPoint(points[0][i__].x-motionVec[0][i__],points[0][i__].y-motionVec[1][i__] );

            cv::line(image, startPoint, points[0][i__], cv::Scalar(0,color_gap*i__,255),2);

            std::ostringstream str;
            str <<i__;

            cv::putText(image,str.str(), points[0][i__], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,color_gap*i__,255));
        }

    }
}

}// end of namespace

#endif
