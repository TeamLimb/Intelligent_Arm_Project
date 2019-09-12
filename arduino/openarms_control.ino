#include <ros.h>
#include <std_msgs/Float64.h>
#include <Wire.h>
//You have to download this Library
#include <Adafruit_PWMServoDriver.h>

ros::NodeHandle nh;

uint8_t state = 0;
int motion = -1;
int tog_val;
float con_data = 0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//Subscriber function
void messageDetection(const std_msgs::Float64& msg){
    motion = (int)msg.data;
    motion_pwm(motion);
    digitalWrite(13, HIGH-digitalRead(13));
}

std_msgs::Float64 exec_dect;
ros::Subscriber<std_msgs::Float64> s("result_detection", &messageDetection);
ros::Publisher p("exec_detection", &exec_dect);


#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

void setup() {

  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  motion_pwm(-1);
}

void loop() {
  tog_val = analogRead(A0);
  nh.spinOnce();
  if(state == 0 && tog_val >= 100){
    state = 1;
    exec_dect.data = 1.0;
  }else if((state == 0 && tog_val <100) || (state == 1 && tog_val >= 100)){
    exec_dect.data = 0.0;
  }else if(state == 1 && tog_val < 100){
    state = 0;
    exec_dect.data = 0.0;
    motion_pwm(-1);
  }
  p.publish( &exec_dect );
}

//This function is about action in openArms MK.2
//You can customize this if you want add or edit some actions.
void motion_pwm(int mot){
  switch(mot){
    case -1:
  pwm.setPWM(0 , 0, 320);
  delay(1);
  pwm.setPWM(1 , 0, 380);
  delay(1);
  pwm.setPWM(2 , 0, 350);
  delay(1);
  pwm.setPWM(3 , 0, 350);
  delay(1);
  pwm.setPWM(4 , 0, 250);
  delay(1);
  pwm.setPWM(5 , 0, 350);
  delay(1);
  pwm.setPWM(6 , 0, 350);
  delay(1);
  pwm.setPWM(7 , 0, 250);
  delay(1);
  pwm.setPWM(8 , 0, 280);
  delay(1);
  break;
    case 1:
  pwm.setPWM(0 , 0, 150);
  delay(1);
  pwm.setPWM(1 , 0, 440);
  delay(1);
  pwm.setPWM(2 , 0, 350);
  delay(1);
  pwm.setPWM(3 , 0, 540);
  delay(1);
  pwm.setPWM(4 , 0, 200);
  delay(1);
  pwm.setPWM(5 , 0, 400);
  delay(1);
  pwm.setPWM(6 , 0, 500);
  delay(1);
  pwm.setPWM(7 , 0, 190);
  delay(1);
  pwm.setPWM(8 , 0, 200);
  delay(1);
  break;
    case 2:
  pwm.setPWM(0 , 0, 150);
  delay(1);
  pwm.setPWM(1 , 0, 440);
  delay(1);
  pwm.setPWM(2 , 0, 350);
  delay(1);
  pwm.setPWM(3 , 0, 500);
  delay(1);
  pwm.setPWM(4 , 0, 200);
  delay(1);
  pwm.setPWM(5 , 0, 350);
  delay(1);
  pwm.setPWM(6 , 0, 500);
  delay(1);
  pwm.setPWM(7 , 0, 190);
  delay(1);
  pwm.setPWM(8 , 0, 210);
  delay(1);
  break;
    case 3:
  pwm.setPWM(0 , 0, 150);
  delay(1);
  pwm.setPWM(1 , 0, 440);
  delay(1);
  pwm.setPWM(2 , 0, 390);
  delay(1);
  pwm.setPWM(3 , 0, 500);
  delay(1);
  pwm.setPWM(4 , 0, 200);
  delay(1);
  pwm.setPWM(5 , 0, 320);
  delay(1);
  pwm.setPWM(6 , 0, 500);
  delay(1);
  pwm.setPWM(7 , 0, 190);
  delay(1);
  pwm.setPWM(8 , 0, 180);
  delay(1);
  break;
  }
}
