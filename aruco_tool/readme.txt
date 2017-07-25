Main file for marker detection from a top-view camera: detect_aruco_ros_topview.hpp
    - Input: video image
    - Process: 
             detector -> KLT tracker -> marker position/yaw relative to the camera
             
    - Output: publish a message that contains a list of all detected marker to a topic, which can be named via the launch file.
    
    
Main file camera calibration

    - Input: video image
    - Process:
             
    

