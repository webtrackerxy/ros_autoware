#ifndef FAKE_GPS_NODE_H
#define FAKE_GPS_NODE_H


#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

// Declare publishers to broadcast messages on the server
ros::Publisher gnss_pub;

// Define timer for publishing messages
ros::Timer timer;
void timerCallback(const ros::TimerEvent&);

bool ndt_state = 0; 
geometry_msgs::PoseStamped temp_pose;
geometry_msgs::PoseStamped last_pose;

#endif //FAKE_GPS_NODE_H
