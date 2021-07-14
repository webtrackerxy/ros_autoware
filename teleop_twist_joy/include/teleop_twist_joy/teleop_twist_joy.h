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

#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <map>
#include <string>
#include <std_msgs/Bool.h>

namespace ros { class NodeHandle; }

namespace teleop_twist_joy
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class TeleopTwistJoy
{
public:
  TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};
}  // namespace teleop_twist_joy

// ----- PARAMETERS HERE ----- //
const float TRANSLATION_SPEED_NORMAL    = 0.2;    // normal speed for translation
const float TRANSLATION_SPEED_TURBO     = 0.4;    // turbo speed for translation
const float ROTATION_SPEED_NORMAL       = 0.15;   // normal speed for rotation
const float ROTATION_SPEED_TURBO        = 0.25;   // turbo speed for rotation
const int TRANSLATION_PARAM           = 10;     // devide the translation speed from /twist_cmd by this value before sending to Arduino
const int ROTATION_PARAM              = 15;     // devide the rotation speed from /twist_cmd by this value before sending to Arduino
const float ACCELERATION_LIMIT        = 0.01;   // speed change larger then this value is not allowed (difference between current speed and upcoming speed)
const float ZERO_POSE                 = 0.03;    // the zero pose to reset the joystick after the car stop (-ZERO_POSE < input < ZERO_POSE means zero)

#endif  // TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
