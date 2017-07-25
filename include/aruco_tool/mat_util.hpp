/**
* @brief  Functions for OpenCV data type conversion (cvMat<->Vec3d) and processing of detected Rect
*
* @author Yiming Wang (wangyimingkaren@gmail.com)
* @date 25/01/2017
*/

#ifndef MAT_UTIL_HPP_INCLUDED
#define MAT_UTIL_HPP_INCLUDED


#include <opencv2/core/core.hpp>
#include "opencv2/calib3d.hpp"
#include <ctype.h>

#include <vector>
#include <math.h>
#include <stdio.h>

namespace aruco_detection {

/**
 *@brief convert an input Vec3d type to cvMat type
 */
cv::Mat DoubleMatFromVec3d(cv::Vec3d &in);

/**
* @brief convert an input cvMat type to Vec3d type
*/
cv::Vec3d Vec3dFromDoubleMat(cv::Mat &in);

/**
* @brief compute the inverse of translation and rotation vector, inouts are in Vec3d type
*/
void inverseTransform(cv::Vec3d &inputTvec, cv::Vec3d &outputTvec, cv::Vec3d &inputRvec, cv::Vec3d &outputRvec);

/**
* @brief convert ratation vector to quaternion in array type
*/
void rvec2quaternion(cv::Vec3d &inputRvec, double (&outputQuat)[4]);

/**
 * @brief The RectTool class that contains all tools relate to rect.
 */
class RectTool
{
public:

    /**
     * @brief computes the approximted tilt angle of a deteced rect, in order to analyse the angle limit when the detector fails
     * @return approximated angle (in degrees)
     */
    double approximateAngle(std::vector<cv::Point2f> &points){
        double angle = 0.0;
        double width = abs(points[0].x-points[3].x)+abs(points[1].x-points[2].x) + abs(points[0].x-points[1].x)+abs(points[2].x-points[3].x);
        double height = abs(points[0].y-points[3].y)+abs(points[1].y-points[2].y) + abs(points[0].y-points[1].y)+abs(points[2].y-points[3].y);


        double cos_value = width/height;
        if (cos_value>1)
            cos_value = 1;
        angle = acos(cos_value)*180.0/M_PI;

        return angle;
    }

    /**
     * @brief print out the corner points of a rect in terminal for debug use
     */
    void printCorner(std::vector<cv::Point2f> points)
    {
        int num_corners = points.size();

        if (num_corners>0){
            for (int i = 0;i<num_corners;i++){
                std::cout<<"Corner "<<i<<std::endl;
                std::cout<<"    "<<points[i].x<<", "<<points[i].y<<std::endl;
            }
        }
    }

    /**
     * @brief overloaded function for print out the corner points of a rect for debug use, it takes point with cvPoint type
     */
    void printCorner(std::vector<cv::Point> points)
    {
        int num_corners = points.size();

        if (num_corners>0){
            for (int i = 0;i<num_corners;i++){
                std::cout<<"Corner "<<i<<std::endl;
                std::cout<<"    "<<points[i].x<<", "<<points[i].y<<std::endl;
            }
        }
    }

    /**
     * @brief get contour with corner points in cvPoint2f to points in cvPoint
     */
    void getContour(std::vector<cv::Point2f> &input_points, std::vector<std::vector<cv::Point> > &output_tempVec)
    {
        output_tempVec.clear();

        std::vector<cv::Point> tempContour;

        for (int j = 0; j < (int) input_points.size(); j++)
        {
            tempContour.push_back(cv::Point((int)input_points[j].x,(int)input_points[j].y));
        }

        output_tempVec.push_back(tempContour);
    }

    /**
     * @brief check if two consective detected Rect satisfy the constraints in size and shape (applied when the detector is not reliable)
     * @return bool true when all constrains are satisfied
     */
    bool checkConsecutiveDetection(cv::Rect previousRect, cv::Rect currentRect, double threshValue[])
    {

        bool x_, y_, width_, height_;

        // x satisfied
        double threshold_x = threshValue[0], threshold_y = threshValue[1], threshold_width = threshValue[2], threshold_height = threshValue[3];

        int p_center_x = previousRect.x+0.5*previousRect.width;
        int center_x = currentRect.x+0.5*currentRect.width;

        int p_center_y = previousRect.y+0.5*previousRect.height;
        int center_y = currentRect.y+0.5*currentRect.height;

        x_ = double(std::abs(p_center_x - center_x)) < threshold_x;

        // y satisfied
        y_ = double(std::abs(p_center_y - center_y))< threshold_y;

        // rows satisfied
        width_ = double(std::abs(previousRect.width - currentRect.width))< threshold_width;

        // height satisfied
        height_ = double(std::abs(previousRect.height - currentRect.height)) < threshold_height;

        std::cout<<"Constraints condition::"<< x_ <<","<< y_ <<","<<width_<<","<<height_<<std::endl;
        return x_ && y_ && width_ && height_;

    }


    /**
     * @brief set the constraints based on the based on the displacements and the number of continous failures
     */
    void setConstriants(cv::Rect pprev_MarkerRect, cv::Rect prev_MarkerRect, int cont_failure, double (&threshold)[4])
    {
        int pp_center_x = pprev_MarkerRect.x + int (0.5*pprev_MarkerRect.width);
        int pp_center_y = pprev_MarkerRect.y + int (0.5*pprev_MarkerRect.height);

        int p_center_x = prev_MarkerRect.x + int (0.5*prev_MarkerRect.width);
        int p_center_y = prev_MarkerRect.y + int (0.5*prev_MarkerRect.height);

        int center_displace_x = std::abs(p_center_x - pp_center_x);
        int center_displace_y = std::abs(p_center_x - pp_center_x);

        int width_displace = std::abs(prev_MarkerRect.width - pprev_MarkerRect.width);
        int height_displace = std::abs(prev_MarkerRect.height - pprev_MarkerRect.height);

        if (center_displace_x<10)
            center_displace_x= 10;

        if (center_displace_y<10)
            center_displace_y = 10;

        if (width_displace<10)
            width_displace= 10;

        if (height_displace<10)
            height_displace = 10;

        // enlarge the threshold as the number of continous failures increases
        threshold[0] = double((cont_failure+1)*center_displace_x+50);
        threshold[1] = double((cont_failure+1)*center_displace_y+50);
        threshold[2] = double((cont_failure+1)*width_displace+50);
        threshold[3] = double((cont_failure+1)*height_displace+50);

    }


    /**
     * @brief shrink or expand the rect by a ratio (r) within the image plane
     * @return new rect
     */
    cv::Rect changeRectShape(cv::Rect oldRect, float r[], const int &image_height, const int &image_width)
    {
        // prepare the ROI area

        int center_x = oldRect.x + int (0.5*oldRect.width);
        int center_y = oldRect.y + int (0.5*oldRect.height);

        int ideal_window_width = int (oldRect.width*r[0]);
        int ideal_window_height = int (oldRect.height*r[1]);

        int window_x = std::max((center_x - int (0.5*ideal_window_width)),0);
        int window_y = std::max((center_y - int (0.5*ideal_window_height)),0);

        // within the image plane
        int window_width = std::min(image_width-1- window_x,ideal_window_width);
        int window_height = std::min(image_height-1- window_y,ideal_window_height);

        cv::Rect newRect(window_x, window_y, window_width, window_height);

        return newRect;
    }

    /**
     * @brief computes the ROI ratio to enlarge or shrink based on previous rect for current detection, using previous two detections
     *        (applied when the detector is not reliable)
     */
    void setROIRatio(cv::Rect pprev_MarkerRect, cv::Rect prev_MarkerRect, float (&ratio4ROI)[2])
    {
        int pp_center_x = pprev_MarkerRect.x + int (0.5*pprev_MarkerRect.width);
        int pp_center_y = pprev_MarkerRect.y + int (0.5*pprev_MarkerRect.height);

        int p_center_x = prev_MarkerRect.x + int (0.5*prev_MarkerRect.width);
        int p_center_y = prev_MarkerRect.y + int (0.5*prev_MarkerRect.height);


        if (prev_MarkerRect.width!=0)
        {
            float displacement_ratio_x = float (std::abs(p_center_x-pp_center_x))/float (prev_MarkerRect.width);
            float size_ratio_x = float (std::max((prev_MarkerRect.width - pprev_MarkerRect.width),0))/float (prev_MarkerRect.width);

            ratio4ROI[0]  = 1.5+ 5*displacement_ratio_x + 5*size_ratio_x ;

            std::cout<<"width ratio to enlarge>>>>>>>"<<displacement_ratio_x<<" and "<<size_ratio_x<<std::endl;
        }
        else // when there is no pprev_MarkerRect
        {
            ratio4ROI[0] = 2;
        }


        if (prev_MarkerRect.height!=0)
        {

            float displacement_ratio_y = float (std::abs(p_center_y-pp_center_y))/float (prev_MarkerRect.height);
            float size_ratio_y = float (std::max((prev_MarkerRect.height - pprev_MarkerRect.height),0))/prev_MarkerRect.height;

            ratio4ROI[1] = 1.5 + 5*displacement_ratio_y + 5*size_ratio_y;

            std::cout<<"height ratio to enlarge>>>>>>>"<<displacement_ratio_y<<" and "<<size_ratio_y<<std::endl;
        }
        else // when there is no pprev_MarkerRect
        {
            ratio4ROI[1] = 2;
        }
    }

}; // end of class definition

void inverseTransform(cv::Vec3d &inputTvec, cv::Vec3d &outputTvec, cv::Vec3d &inputRvec, cv::Vec3d &outputRvec){

    // outputTvec = -inputTvec;

    cv::Mat tempRMatrix(3,3,cv::DataType<double>::type); // rotation matrix
    cv::Rodrigues(inputRvec, tempRMatrix);


    cv::Mat tempRMatrixTranspose(3,3,cv::DataType<double>::type);
    cv::transpose(tempRMatrix, tempRMatrixTranspose);

    cv::Mat temp = DoubleMatFromVec3d(inputTvec);
    cv::Mat outputTvecMat = -tempRMatrixTranspose*temp;

    outputTvec = Vec3dFromDoubleMat(outputTvecMat);
    cv::Rodrigues(tempRMatrixTranspose,outputRvec);

}


cv::Mat DoubleMatFromVec3d(cv::Vec3d &in)
{
    cv::Mat mat(3,1, CV_64FC1);
    mat.at <double>(0,0) = in [0];
    mat.at <double>(1,0) = in [1];
    mat.at <double>(2,0) = in [2];

    return mat;
}

cv::Vec3d Vec3dFromDoubleMat(cv::Mat &in)
{
    cv::Vec3d vec(3,1, CV_64FC1);
    vec[0] = in.at<double>(0, 0);
    vec[1] = in.at<double>(1, 0);
    vec[2] = in.at<double>(2, 0);
    return vec;
}

void rvec2quaternion(cv::Vec3d &inputRvec, double (&outputQuat)[4] ){

    double theta = sqrt(pow(inputRvec[0], 2) + pow(inputRvec[1], 2)+pow(inputRvec[2], 2));

    if (theta!=0){
        outputQuat[0] =  inputRvec[0]/theta*sin(theta/2);
        outputQuat[1] =  inputRvec[1]/theta*sin(theta/2);
        outputQuat[2] =  inputRvec[2]/theta*sin(theta/2);
        outputQuat[3] =  cos(theta/2);
    }else{
        outputQuat[0] =  0;
        outputQuat[1] =  0;
        outputQuat[2] =  0;
        outputQuat[3] =  1;
    }
}

} // end of name space

#endif // RECT_UTIL_HPP_INCLUDED
