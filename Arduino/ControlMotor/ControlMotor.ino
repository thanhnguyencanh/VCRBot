#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

#define ENB_L 10
#define IN3_L 22
#define IN4_L 23
#define ENA_R 11
#define IN1_R 24
#define IN2_R 25
double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is distance between 2 wheels.
double wheel_rad = 0.03, wheel_sep = 0.17;
double speed_ang=0, speed_lin=0;
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;

  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));  //rad/s
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));  //rad/s

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup(){
 Motors_init();
 nh.initNode();
 nh.subscribe(sub);
}
void loop(){  
 MotorL(w_l*10);
 MotorR(w_r*10);
 nh.spinOnce();
}
void Motors_init(){
 pinMode(ENB_L, OUTPUT);
 pinMode(ENA_R, OUTPUT);
 pinMode(IN3_L, OUTPUT);
 pinMode(IN4_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 digitalWrite(ENB_L, LOW);
 digitalWrite(ENA_R, LOW);
 digitalWrite(IN3_L, LOW);
 digitalWrite(IN4_L, LOW);
 digitalWrite(IN1_R, LOW);
 digitalWrite(IN2_R, LOW);
}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(ENB_L, Pulse_Width1);
     digitalWrite(IN3_L, HIGH);
     digitalWrite(IN4_L, LOW);
 }
 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(Pulse_Width1);
     analogWrite(ENB_L, Pulse_Width1);
     digitalWrite(IN3_L, LOW);
     digitalWrite(IN4_L, HIGH);
 }
 if (Pulse_Width1 == 0){
     analogWrite(ENB_L, Pulse_Width1);
     digitalWrite(IN3_L, LOW);
     digitalWrite(IN4_L, LOW);
 }
}
void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     analogWrite(ENA_R, Pulse_Width2);
     digitalWrite(IN1_R, HIGH);
     digitalWrite(IN2_R, LOW);
 }
 if (Pulse_Width2 < 0){
     Pulse_Width2=abs(Pulse_Width2);
     analogWrite(ENA_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, HIGH);
 }
 if (Pulse_Width2 == 0){
     analogWrite(ENA_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, LOW);
 }
}
