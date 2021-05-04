#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>

//#include <std_msgs/UInt16.h>
#include <rosbrachiograph/Servo.h>

ros::NodeHandle  nh;

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SHOULDER_SERVO 0
#define ELBOW_SERVO 1
#define PEN_SERVO 2

int pen_goal = 300;
int pen_current_pos = 300;
int elbow_goal = 190;
int elbow_current_pos = 190;
int shoulder_goal = 190;
int shoulder_current_pos = 190;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

 void jog_cb( const rosbrachiograph::Servo&  cmd_msg){
  elbow_goal = cmd_msg.elbow_pos;  
  nh.logdebug(cmd_msg.elbow_pos);
 
}

/*
void command_cb( const std_msgs::UInt16&  cmd_msg){ 
  nh.logdebug(cmd_msg.data);
 
}


void elbow_servo_cb( const std_msgs::UInt16&  cmd_msg){
  elbow_goal = cmd_msg.data; 
  nh.logdebug(cmd_msg.data);

}

void pen_cb( const std_msgs::UInt16&  cmd_msg){
  pwm.setPWM(PEN_SERVO, 0, cmd_msg.data); //set servo pulse width, should be 160 or 300
  nh.logdebug(cmd_msg.data);

} */

ros::Subscriber<rosbrachiograph::Servo> subjog("jog_servo", jog_cb);
//ros::Subscriber<std_msgs::UInt16> subcmd("command", command_cb);

//ros::Subscriber<std_msgs::UInt16> sub2("elbow", elbow_servo_cb);
//ros::Subscriber<std_msgs::UInt16> sub3("pen", pen_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(subjog);
  //nh.subscribe(subcmd);
  // nh.subscribe(sub2);
  // nh.subscribe(sub3);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Wire.setClock(400000);
  delay(10);

}

void loop(){
  if (abs(shoulder_current_pos - shoulder_goal) > 10) {
    if (shoulder_goal > shoulder_current_pos) {shoulder_current_pos += 3;}
    if (shoulder_goal < shoulder_current_pos) {shoulder_current_pos -= 3;}
    pwm.setPWM(SHOULDER_SERVO, 0, shoulder_current_pos);
  }

    if (abs(elbow_current_pos - elbow_goal) > 10) {
    if (elbow_goal > elbow_current_pos) {elbow_current_pos += 3;}
    if (elbow_goal < elbow_current_pos) {elbow_current_pos -= 3;}
    pwm.setPWM(ELBOW_SERVO, 0, elbow_current_pos);
  }

  nh.spinOnce();
  delay(20);
}