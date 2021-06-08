/*
   Rosserial Wheelchair Control with XBox 360 Controller

   This sketch controls the motors of a wheelchair via a Sabertooth 2x32 Motor Controller
   using ROS and the arduino

   IMPORTANT: to avoid latency when using a XBox controller, increase the baud rate of Arduino to 250000 with the following command:

   rosrun rosserial_python serial_node.py _port:[INSERT PORT NAME, e.g."=/dev/ttyACM0"] _baud:=250000

*/

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

ros::NodeHandle  nh;

//Decide where you are going to plug in the Arduino
int Tx_pin = 2;             // Arduino Transmit Pin to Sabertooth S1
int Rx_pin = 3;             // Arduino Receive Pin to Sabertooth S2

SoftwareSerial SWSerial(Rx_pin, Tx_pin); // RX, TX on pin 2 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

float raw_translation_power = 0.0, raw_rotation_power = 0.0;  //power from -1.0 to 1.0

int watch_dog_counter = 0;

void joyTwistCallback(const geometry_msgs::Twist& joy_twist) {
    raw_translation_power = joy_twist.linear.x;
    raw_rotation_power = joy_twist.angular.z;
    watch_dog_counter = 0;
}

ros::Subscriber<geometry_msgs::Twist> joy_twist_sub("cmd_vel", joyTwistCallback);

int powerMinMax(float raw_power) {
  if (raw_power > 1) raw_power = 1;
  else if (raw_power < -1)  raw_power = -1;
  return round(raw_power * 127.0);
}

void setup() {
  ST.drive(0);              // Send command to stop transldational motion
  ST.turn(0);               // Send command to stop rotational motion
  SWSerial.begin(9600);     // Establish serial connection to Sabertooth
  //Serial.begin(57600);
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(joy_twist_sub);
}

void loop() {
  if (watch_dog_counter < 20) watch_dog_counter++;

  if (watch_dog_counter >= 10){
    raw_translation_power /= 2;
    raw_rotation_power /= 2;
  }else if (watch_dog_counter >= 15){
    raw_translation_power = 0;
    raw_rotation_power = 0;
  }

  int translation_power = powerMinMax(raw_translation_power);
  int rotation_power = powerMinMax(raw_rotation_power);

  char translation_log[8];
  char rotation_log[8];
  dtostrf(translation_power, 4, 2, translation_log);
  dtostrf(rotation_power, 4, 2, rotation_log);

  nh.loginfo("Translation: "); nh.loginfo(translation_log);
  nh.loginfo("Rotation: "); nh.loginfo(rotation_log);

  ST.drive(0-rotation_power);  //since  left right wheel pin is flipped
  ST.turn(0-translation_power); //since  left right wheel pin is flipped
  nh.spinOnce();
  delay(100);
}
