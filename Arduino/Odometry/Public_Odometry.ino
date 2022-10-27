#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <LiquidCrystal.h>

ros::NodeHandle nh;

#define ENB_L 10
#define IN3_L 22
#define IN4_L 23
#define ENA_R 11
#define IN1_R 24
#define IN2_R 25

#define encoderLeftA 18
#define encoderLeftB 16
#define encoderRightA 19
#define encoderRightB 17

volatile double counterL = 0, counterR = 0;

double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.03, wheel_sep = 0.17;
double speed_ang=0, speed_lin=0;

double x = 0.0;
double y = 0.0;
double theta = 0.0;
double dt = 0.1; //delta t = 100 ms

double vx = 0.0;
double vth = 0.0;

double dLeft = 0.0;
double dRight = 0.0;
double dCenter = 0.0;
double phi = 0.0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
void Motors_init();
void countpulseLeft();
void countpulseRight();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

double getDistanceLeft();
double getDistanceRight();

//Tranform broadcaster object
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::Quaternion odom_quat;
tf::TransformBroadcaster odom_broadcaster;
//Odometry object
nav_msgs::Odometry odom;
ros::Publisher odom_pub("/odom", &odom);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb );

void setup() {
//  Serial.begin(9600);
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(odom_pub);
  odom_broadcaster.init(nh);
}

void loop() {

  MotorL(w_l*10);
  MotorR(w_r*10);

  dLeft = getDistanceLeft();
  dRight = getDistanceRight();
  dCenter = (dLeft + dRight)/2.0;
  phi = (dRight - dLeft)/wheel_sep;
  theta += phi;
  //constrain theta to the range 0 to 2pi
  if (theta > 2.0*PI) theta -=2.0*PI;
  if (theta <0.0) theta +=2.0*PI;
  //update Robot's x, y coordinates
  x += dCenter*cos(theta);
  y += dCenter*sin(theta);
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
  //
  //publish the transform over tf
  //
  odom_trans.header.stamp = nh.now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "link_chassis";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
  //
  //publish the odometry msg over ROS
  //
  vth = speed_ang;
  vx = speed_lin;
  odom.header.stamp = nh.now();
  odom.header.frame_id = "odom";
  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  //set the velocity
  odom.child_frame_id = "link_chassis";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = vth;
  //publish the message
  odom_pub.publish(&odom);
  
  nh.spinOnce();
  
}
double getDistanceRight(){
  double sR = (counterR/374.0)*2.0*PI*wheel_rad;
  Serial.print(" counter Right: ");
  Serial.println(counterR);
  counterR = 0;
  return sR;
}
double getDistanceLeft(){
  double sL = (counterL/374.0)*2.0*PI*wheel_rad;
  Serial.print(" counter Left: ");
  Serial.println(counterL);
  counterL = 0;
  return sL;
}

void Motors_init(){
 pinMode(ENB_L, OUTPUT);
 pinMode(ENA_R, OUTPUT);
 pinMode(IN3_L, OUTPUT);
 pinMode(IN4_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 pinMode(encoderLeftA, INPUT);
 pinMode(encoderLeftB, INPUT);
 pinMode(encoderRightA, INPUT);
 pinMode(encoderRightB, INPUT);
 digitalWrite(ENB_L, LOW);
 digitalWrite(ENA_R, LOW);
 digitalWrite(IN3_L, LOW);
 digitalWrite(IN4_L, LOW);
 digitalWrite(IN1_R, LOW);
 digitalWrite(IN2_R, LOW);
 digitalWrite(encoderLeftA, HIGH);
 digitalWrite(encoderLeftB, HIGH);
 digitalWrite(encoderRightA, HIGH);
 digitalWrite(encoderRightB, HIGH);
 attachInterrupt(5,countpulseLeft,RISING);
 attachInterrupt(4,countpulseRight,RISING);
}
void countpulseLeft(){
        counterL++;
}
void countpulseRight(){
        counterR++;
}
void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(ENB_L, Pulse_Width1);
     digitalWrite(IN3_L, LOW);
     digitalWrite(IN4_L, HIGH);
 }
 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(Pulse_Width1);
     analogWrite(ENB_L, Pulse_Width1);
     digitalWrite(IN3_L, HIGH);
     digitalWrite(IN4_L, LOW);
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
