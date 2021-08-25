/* 
 * Adpated from rosserial Servo Control Example to work with the
 * Adafruit PWM Sheild
 *
 * For the full tutorial write up, visit
 * http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)

ros::NodeHandle  nh;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void servo_cb( const std_msgs::UInt16& cmd_msg){
  pwm.setPWM(2, 0, cmd_msg.data); 
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Wire.setClock(400000);
  delay(10);

}

void loop(){
  nh.spinOnce();
  delay(1);
}