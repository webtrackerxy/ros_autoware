teleop_twist_joy [![Build Status](https://travis-ci.org/ros-teleop/teleop_twist_joy.svg?branch=indigo-devel)](https://travis-ci.org/ros-teleop/teleop_twist_joy)
================

Simple joystick teleop for twist robots. See [ROS Wiki](http://wiki.ros.org/teleop_twist_joy)  


## Buttons: 
Blue: holding for auto mode while release for manual mode  
Yellow: press to toggle the speed (normal / turbo)


## Structure: 
  
Subscribe to joy and the twist from autoware  
Depends on the input from joy (the enable button) to change the mode (auto or manual mode)  
Convert the joy to twist message  
Depends on the mode, send the twist message to Arduino for motor control  

## Features: 
Auto mode is allowed only when the ndt_status is ok  
- auto mode will stop immediately when the car is  not localized
- it will automatically resume the auto mode once the car is localized
  
When the car is not localized, it will stop  
- it acts as a reminder for the driver to know the car is not localized
- it attempt to wait to see if the car can localize again itself
- driver can overwrite this by resetting the joystick to zero, then the driver can control the car again  
Example:  
When the car is localized and driving by the driver in manual mode, if the car suddenly not localized, it will stop to alert the driver that it is not localized. Driver can choose to wait for it to localized again (since usually stopping immediately can help the car to localize again), or choose to control it again. To control the car again, driver need to reset the joystick to zero position to acknowledge that he/she already know the car is not localized. Then moving the joystick again allows the driver to control the car again. 

## Remarks:   
IF it does not publish and ask if /clock is published, run the following command:  
``` rosparam get use_sim_time ```  
If return true, runs:  
``` rosparam set use_sim_time false ```  
See [this website](https://answers.ros.org/question/12083/messages-being-blocked-from-publishing/)

### Subscribe: 
1. /twist_cmd
2. /joy
3. /ndt_monitor/ndt_status

### Publish: 
1. /cmd_vel

  
### Parameters: 
These are the parameters in the header file  
  
| Parameter                | Usage                                                                                                       | Values   |
|--------------------	   |----------------------------------------------------------------------------------------------------------	 |--------	|
| TRANSLATION_PARAM  	   | devide the translation speed from /twist_cmd by this value before sending to Arduino                     	 | 15     	|
| ROTATION_PARAM     	   | devide the rotation speed from /twist_cmd by this value before sending to Arduino                        	 | 20     	|
| ACCELERATION_LIMIT 	   | speed change larger then this value is not allowed (difference between current speed and upcoming speed) 	 | 0.01   	|
| TRANSLATION_SPEED_NORMAL | normal speed for translation                                                                                | 0.2      | 
| TRANSLATION_SPEED_TURBO  | turbo speed for translation                                                                                 | 0.4      | 
| ROTATION_SPEED_NORMAL    | normal speed for rotation                                                                                   | 0.15     | 
| ROTATION_SPEED_TURBO     | turbo speed for rotation                                                                                    | 0.25     | 
| ZERO_POSE                | the zero pose to reset the joystick after the car stop (-ZERO_POSE < input < ZERO_POSE means zero)          | 0.03     | 
  
