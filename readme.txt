aruco_tool is developed as a ROS package that plays a part in a larger project for a multi-robot system. 

aruco_tool takes the image captures from ceiling-mounted cameras and outputs the poses with respect to a referrence coordinate. The marker detection makes use of built-in functions provided in OpenCV aruco library and KL points tracker. Two executables are implemented in this packages for:

1) calibrating a camera pose with respect to a reference coordinate using a marker
   -- src/arucoCalibration.cpp

2) detecting and estimating the poses of specified markers with respect to a reference coordinate.
   -- src/arucoDetectionTopView.cpp

Core functions are implemented within .hpp files that are located in include/aruco_tool folder.

  
This package defines two messages in msg folder for encapsulating the marker poses estimated from the marker detection on the image plane.          
   -- ARMarker.msg for single marker (is not designed to be published on its own) 
   -- ARMarkers.msg for multiple markers that wraps the ARMarker.msg and has other fields including header, sequence of the image and time of image capture.  

For successful compilation with 'catkin_make', this package requires dependancies on OpenCV3 and other self-developed ROS packages for message types (e.g. ImageSync, which is used for synchronised image capturing among cameras on multiple machines). 
