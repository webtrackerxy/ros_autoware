/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "teleop_twist_joy/teleop_twist_joy.h"

// should put these in header file actually, but I don't want to change the header file
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <map>
#include <string>
#include <std_msgs/Bool.h>


namespace teleop_twist_joy
{

enum{
  normal,
  turbo
};

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void autoCallback(const geometry_msgs::TwistStamped::ConstPtr& auto_msg);
  void statusCallback(const std_msgs::String::ConstPtr& status_msg);
  void saveJoyCmd(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void timerCallback(const ros::TimerEvent&);
  void setZero (geometry_msgs::Twist &msg);

  ros::Subscriber joy_sub;
  ros::Subscriber auto_sub;
  ros::Subscriber state_sub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher cmd_mode_pub;
  ros::Timer timer;

  int enable_button;
  int gear_button;
  int speed_mode;
  int auto_mode;   //1 for auto while 0 for manual
  int ndt_state;   //1 for localized while 0 for not localized
  bool toggled_speed_mode;
  bool toggled_auto_mode;

  geometry_msgs::Twist cmd_vel_msg;       //current speed, updated by joy or autoware - depends on the mode
  geometry_msgs::Twist prev_cmd_vel_msg;  //save down the speed of the last msg for speed control

  std::map<std::string, int> axis_linear_map;
  std::map< std::string, std::map<std::string, double> > scale_linear_map;

  std::map<std::string, int> axis_angular_map;
  std::map< std::string, std::map<std::string, double> > scale_angular_map;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->cmd_mode_pub = nh->advertise<std_msgs::Bool>("cmd_mode", 10);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
  pimpl_->auto_sub = nh->subscribe<geometry_msgs::TwistStamped>("twist_cmd", 1, &TeleopTwistJoy::Impl::autoCallback, pimpl_);
  pimpl_->state_sub = nh->subscribe<std_msgs::String>("/ndt_monitor/ndt_status", 1, &TeleopTwistJoy::Impl::statusCallback, pimpl_);
  pimpl_->timer = nh->createTimer(ros::Duration(0.05), &TeleopTwistJoy::Impl::timerCallback, pimpl_);  //publish every 50ms

  nh_param->param<int>("enable_button", pimpl_->enable_button, 2);
  nh_param->param<int>("gear_button", pimpl_->gear_button, 3);
  nh_param->param<int>("speed_mode", pimpl_->speed_mode, normal);
  nh_param->param<int>("auto_mode", pimpl_->auto_mode, 0);
  nh_param->param<bool>("toggled_speed_mode", pimpl_->toggled_speed_mode, false);
  nh_param->param<bool>("toggled_auto_mode", pimpl_->toggled_auto_mode, false);

  if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
    nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["normal"]["x"], 0.2);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]["x"], 0.4);
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
    nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["normal"]["yaw"], 0.1);
    nh_param->param<double>("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]["yaw"], 0.25);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->gear_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->gear_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->gear_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->gear_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->setZero(pimpl_->cmd_vel_msg);
  pimpl_->setZero(pimpl_->prev_cmd_vel_msg);
}

// set the twist msg to zero
void TeleopTwistJoy::Impl::setZero (geometry_msgs::Twist& msg){
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::saveJoyCmd(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map)
{
  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // toggle speed mode when button pressed
  if (joy_msg->buttons[gear_button] && !toggled_speed_mode){
    if (speed_mode == normal)
      speed_mode = turbo;
    else
      speed_mode = normal;
    toggled_speed_mode = true;
  }else if (!joy_msg->buttons[gear_button] && toggled_speed_mode)
    toggled_speed_mode = false;
  
  // if enabled button is pressed means it is in auto mode
  if (!joy_msg->buttons[enable_button] || !ndt_state){  // if car is not localized, not allowed to change to auto mode
    auto_mode = 0;
    if (speed_mode == normal)
      saveJoyCmd(joy_msg, "normal");
    else 
      saveJoyCmd(joy_msg, "turbo");
  }else { 
    auto_mode = 1;
  }
}

void TeleopTwistJoy::Impl::autoCallback(const geometry_msgs::TwistStamped::ConstPtr& auto_msg)
{
  // if it is in auto mode then set the twist according to input
  if (auto_mode){
    cmd_vel_msg = auto_msg->twist;
    cmd_vel_msg.linear.x /= 15;
    cmd_vel_msg.linear.y /= 15;
    cmd_vel_msg.linear.z /= 15;
    cmd_vel_msg.angular.x /= 20;
    cmd_vel_msg.angular.y /= 20;
    cmd_vel_msg.angular.z /= 20;
  }
}

// check if the car is localized
void TeleopTwistJoy::Impl::statusCallback(const std_msgs::String::ConstPtr& status_msg){
  std::string status = status_msg->data;
  if (status.compare("NDT_OK") == 0){
    ndt_state = 1;
  }else {
    ndt_state = 0;
  }
}

float checkSpeed(float num, float prev_num){
  float diff = num - prev_num;
  if (diff > 0.03 || diff < -0.03){
    num = (num + prev_num) / 2;
  }
  return num;
}

void TeleopTwistJoy::Impl::timerCallback(const ros::TimerEvent&){
  float diff = 0;
  
  // check the linear speed
  do {
    checkSpeed(cmd_vel_msg.linear.x, prev_cmd_vel_msg.linear.x);
    diff = cmd_vel_msg.linear.x - prev_cmd_vel_msg.linear.x;
  }while(diff > 0.03 || diff < -0.03);

  // check the angular speed
  do {
    checkSpeed(cmd_vel_msg.angular.z, prev_cmd_vel_msg.angular.z);
    diff = cmd_vel_msg.angular.z - prev_cmd_vel_msg.angular.z;
  }while(diff > 0.03 || diff < -0.03);
  
  cmd_vel_pub.publish(cmd_vel_msg);
  prev_cmd_vel_msg = cmd_vel_msg;
}

}  // namespace teleop_twist_joy
