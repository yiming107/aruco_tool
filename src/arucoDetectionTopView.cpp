/**
 * @brief  main file for detecting markers and estimating marker pose w.r.t a reference coordinate
 *         the poses are published to topic that other ROS nodes can subscribe to for motion planning
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

#include "aruco_tool/detect_aruco_ros_topview.hpp"

int main (int argc, char **argv)
{
    ros::init (argc, argv, "ar_single");
    ros::NodeHandle n;

    aruco_detection::ArucoPublisherTopView arucoPub(n);
    arucoPub.arucoInit();

    ros::spin(); // keep listening to and processing image message
    return 0;
}
