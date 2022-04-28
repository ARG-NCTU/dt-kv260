#include <PID_v1.h>
#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 12;//B pin -> the digital pin 3
int in1 =8; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int in2 =9; //The enabling of L298PDC motor driver board connection to the digital interface port 4
int ena =5; 
const byte encoder1pinA = 3;//A pin -> the interrupt pin 0
const byte encoder1pinB = 13;//B pin -> the digital pin 3
int in3 =10; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int in4 =11; //The enabling of L298PDC motor driver board connection to the digital interface port 4
int enb =6; 


int symbol_r=1,symbol_l=1;
int val_r=0, val_l=0;

ros::NodeHandle nh;





void cb_twist(const geometry_msgs::Twist& cmd_vel){
  
  float WHEEL_DIST = 5;
  float speed_wish_right = ((cmd_vel.angular.z * WHEEL_DIST)/2 + cmd_vel.linear.x);
  float speed_wish_left = (cmd_vel.linear.x * 2-speed_wish_right);

  if(speed_wish_right>=0){
    symbol_r = 1;
    val_r = speed_wish_right*255;
  }
  if(speed_wish_right<0){
    symbol_r = 0;
    val_r = -speed_wish_right*255;
  }
  if(speed_wish_left>=0){
     symbol_l = 1;
     val_l = speed_wish_left*255;
  }
  if(speed_wish_left<0){
    symbol_l = 0;
    val_l = -speed_wish_left*255;
  }

  Serial.print("the r value is ");
  Serial.print(speed_wish_right);
  
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&cb_twist);

void setup()
{
   Serial.begin(57600);//Initialize the serial port
   pinMode(in1, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
   pinMode(in2, OUTPUT);
   pinMode(ena, OUTPUT);
   pinMode(in3, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
   pinMode(in4, OUTPUT);
   pinMode(enb, OUTPUT);
   pinMode(encoder0pinA, INPUT);
   pinMode(encoder0pinB, INPUT);
   pinMode(encoder1pinA, INPUT);
   pinMode(encoder1pinB, INPUT);
   nh.initNode();
   nh.subscribe(sub);
   Serial.print("INIT DONE");
}



void loop()
{     
  if(symbol_r==1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    analogWrite(ena,val_r);
  }
  if(symbol_r==0){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(ena,val_r);
  }
  if(symbol_l==1){
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enb,val_l);
  }
  if(symbol_l==0){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    analogWrite(enb,val_l);
  }
  nh.spinOnce();
}
