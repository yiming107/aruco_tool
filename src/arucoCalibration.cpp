/**
 * @brief  main file for calibrating the camera pose w.r.t a reference marker coordinate that is used for the world cooridnate.
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

#include "aruco_tool/aruco_calibration.hpp"

int main (int argc, char **argv)
{
    ros::init (argc, argv, "ar_calibrator");
    ros::NodeHandle n;

    aruco_detection::ArucoCalibrator arucoCalib(n);
    arucoCalib.arucoInit();

    // keep listening to incoming image message and conducts marker detection/pose estimation
    // until the maximum number of detections is reached

    while(arucoCalib.count < arucoCalib.maxCount){
        ros::spinOnce();
    }

    ROS_INFO("Out of while loop");

    // write to the averaged camera pose to the camera parameters file
    bool writeOk = arucoCalib.writeCameraParameters();
    if(!writeOk)
    {
        cerr << "Failed to write to file" << endl;
        return -1;
    }else{
        return 0;
    }

}
