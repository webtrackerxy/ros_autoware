#include "ros/ros.h"
#include "fake_gps_node.h"

void poseCallback(const geometry_msgs::PoseStamped& curr_pose){
    if (ndt_state) temp_pose = curr_pose;
}

void statusCallback(const std_msgs::String& status_msg){
    std::string status = status_msg.data;
    if (status.compare("NDT_OK") == 0){
        ndt_state = 1;
        // ROS_INFO("Localization ok");
    }else {
        ndt_state = 0;
        // ROS_INFO("Localization not ok");
    }
}

void publishTF(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform = tf::Transform(tf::Quaternion(last_pose.pose.orientation.x, last_pose.pose.orientation.y, last_pose.pose.orientation.z, last_pose.pose.orientation.w),
                            tf::Vector3(last_pose.pose.position.x, last_pose.pose.position.y, last_pose.pose.position.z));
  // publish to /tf
  br.sendTransform(tf::StampedTransform(transform, last_pose.header.stamp, "map", "fake_gps"));
}

void timerCallback(const ros::TimerEvent&){
    if (ndt_state){
        last_pose = temp_pose;
    }
    publishTF();
    gnss_pub.publish(last_pose);
}

int main(int argc, char **argv){
    ROS_INFO("Starting up fake gps node...");
    
    ros::init(argc, argv, "fake_gps_node");
    ros::NodeHandle n;
        
    // publisher
    gnss_pub = n.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 1000);

    // subscriber
    ros::Subscriber pose_sub = n.subscribe("/current_pose", 1, poseCallback);
    ros::Subscriber status_sub = n.subscribe("/ndt_monitor/ndt_status", 1, statusCallback);
    ROS_INFO("Publishers and subscriber defined");

    // Setup the timer for conversion and publish messages
    ROS_INFO("Setting up timer");
    timer = n.createTimer(ros::Duration(0.05), timerCallback);  //publish every 50ms

    ros::spin();

    ros::shutdown();
    ROS_INFO("fake gps node stopped");
    return 0;
}

