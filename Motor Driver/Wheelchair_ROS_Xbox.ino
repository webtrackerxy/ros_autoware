/*
 * Rosserial Wheelchair Control with XBox 360 Controller
 *
 * This sketch controls the motors of a wheelchair via a Sabertooth 2x32 Motor Controller 
 * using ROS and the arduino
 * 
 * IMPORTANT: to avoid latency when using a XBox controller, increase the baud rate of Arduino to 250000 with the following command:
 * 
 * rosrun rosserial_python serial_node.py _port:[INSERT PORT NAME, e.g."=/dev/ttyACM0"] _baud:=250000
 * 
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>


ros::NodeHandle  nh;
std_msgs::Float32 joy_msg;

//Decide where you are going to plug in the Arduino
int Tx_pin = 2;             // Arduino Transmit Pin to Sabertooth S1
int Rx_pin = 3;             // Arduino Receive Pin to Sabertooth S2

// Create variables to read joystick values
float ForeAft_Input = 0.0;       // Variable to store data for Fore/Aft input from joystick
float LeftRight_Input = 0.0;     // Variable to store data for Left/Right input from joystick

// These variables allow for math conversions and later error checking as the program evolves.
int Fore_Limit = 1;       // High ADC Range of Joystick ForeAft
int Aft_Limit = -1;        // Low ADC Range of Joystick ForeAft
int Right_Limit = 1;      // High ADC Range of Joystick LeftRight
int Left_Limit = -1;       // Low ADC Range of Joystick LeftRight

// Create variables for Sabertooth
int ForwardReverse_power = 0; // -127 for full reverse, 0 for stop, +127 for full forward.
int LeftRight_power = 0;      // -127 for full CounterClockwise, 0 for stop, +127 for full Clockwise

SoftwareSerial SWSerial(Rx_pin, Tx_pin); // RX, TX on pin 2 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.


float mapf(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void joystick_cb( const sensor_msgs::Joy& joy){
  ForeAft_Input = joy.axes[1];
  LeftRight_Input = joy.axes[0];

  // Debugging
  char FA_stmt[] = "ForeAft_Input Values: ";
  char FA_val[10];
  char LR_stmt[] = "LeftRight_Input Values: ";
  char LR_val[10];

  //4 is mininum width, 2 is precision; float value is copied onto buffer
  dtostrf(ForeAft_Input, 4, 2, FA_val);
  dtostrf(LeftRight_Input, 4, 2, LR_val);
  
  nh.loginfo(FA_stmt);
  nh.loginfo(FA_val);
  nh.loginfo(LR_stmt);
  nh.loginfo(LR_val);
  
  // Convert the joystick values to Sabertooth values
  // For some unknown reasons, the mapping values had to be reversed for the robot to operate normally
  ForwardReverse_power=(int)round(mapf(ForeAft_Input, Aft_Limit, Fore_Limit, 30.0, -30.0));  // Translate and scale joystick values to Sabertooth values - speed capped
  LeftRight_power=(int)round(mapf(LeftRight_Input, Left_Limit, Right_Limit, 30.0, -30.0));  // Translate and scale joystick values to Sabertooth values - speed capped
  
  if (ForwardReverse_power < 5 && ForwardReverse_power > -5) {
    ForwardReverse_power = 0;
  }

  if (LeftRight_power < 5 && LeftRight_power > -5) {
    LeftRight_power = 0;
  }

  // Command the Sabertooth to drive the motors
  // For some unknown reasons, LeftRight was used by Sabertooth to control the translational motion and ForwardReverse was used by Sabertooth to control the rotational motion
  ST.drive(LeftRight_power);  //Command the Sabertooth translational motion
  ST.turn(ForwardReverse_power);        //Command the Sabertooth rotational motion

  // Create debug information

  // Debugging
  char FRP_stmt[] = "ForwardReverse_power Values: ";
  char FRP_val[10];
  char LRP_stmt[] = "LeftRight_power Values: ";
  char LRP_val[10];

  dtostrf(ForwardReverse_power, 4, 2, FRP_val);
  dtostrf(LeftRight_power, 4, 2, LRP_val);
  
  nh.loginfo(FRP_stmt);
  nh.loginfo(FRP_val);
  nh.loginfo(LRP_stmt);
  nh.loginfo(LRP_val);

}

ros::Subscriber<sensor_msgs::Joy> joy_control("joy", joystick_cb);

void setup(){
  ST.drive(0);              // Send command to stop transldational motion
  ST.turn(0);               // Send command to stop rotational motion
  SWSerial.begin(9600);     // Establish serial connection to Sabertooth
  //Serial.begin(57600);
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(joy_control);
  
}

void loop(){
  nh.spinOnce();
}
