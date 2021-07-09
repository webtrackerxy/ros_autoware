Indoor Robot Navigation
================

Indoor robot using Autoware. See [Autoware.ai website](https://www.autoware.ai/)  

This project is to test ROS robot by using Autoware framework. Autoware provides framwork for sensor fusion, SLAM mapping and navigation. Although Autoware is aimed for autonomous vehicle and self-driving car. Customization with own code is easy. This project modified a wheelchair as a differential drive robot to fit with indoor environment. The wheelchair is able to support weight up to 200KG. We can put many many hardware stuffs such as sensors, robot arm, battery, computers, etc on it to maximize the use of power of ROS.

For the details, please see [project notes.pdf](https://github.com/webtrackerxy/ros_autoware/blob/master/project%20notes.pdf)

## Robot Hardware: 
1. Modified Wheelchair Base with 200W motor- See [Website for the hacking](https://www.allaboutcircuits.com/projects/building-an-rc-robot-using-a-wheelchair-base/)
2. Motor Controller - See [Sabertooth 2x32 Motor Controller](https://www.dimensionengineering.com/products/sabertooth2x32)
3. Lidar - See [VLP-16 Velodyne Lidar](https://www.mapix.com/lidar-scanner-sensors/velodyne/velodyne-puck-vlp16/)


## Robot Software: 
1. ROS Kinetic running on Nvidia Jetson TX2 (Ubuntu 18.04) - For Senor funsion and motor control
 
2. ROS Kinetic running on ASUS ROG STRIX G512LI (Ubuntu 18.04) - For visualization and Machine Learning


## Videos: 

 [SLAM Mapping with Point Cloud](https://www.youtube.com/watch?v=OghPLvHJVbM)  

 [Autoware - reprojection & fusion](https://www.youtube.com/watch?v=9VLVO0OEkQY)  

 [Auto Navigation with Autoware](https://www.youtube.com/watch?v=G2NaF90nbe4)  
