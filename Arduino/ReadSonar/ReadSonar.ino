#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

#define MAX_RANGE 2.0
#define MIN_RANGE 0.02 
#define MAX_DISTANCE 200     //Max distance to detect obstacles. (cm)
#define SONAR_NUM 3          //The number of sensors.
#define echoPinLeft 2
#define trigPinLeft 3

#define echoPinCenter 4
#define trigPinCenter 5

#define echoPinRight 6
#define trigPinRight 7

uint8_t leftSensor;             //Store raw sensor's value.
uint8_t centerSensor;
uint8_t rightSensor;

uint8_t leftSensorKalman;             //Store kalman sensor's value.
uint8_t centerSensorKalman;
uint8_t rightSensorKalman;
 
void Sensors_init();
void sensorCycle();
uint8_t PingSonar(int trigPin, int echoPin);

SimpleKalmanFilter KF_Left(2, 2, 0.01);
SimpleKalmanFilter KF_Center(2, 2, 0.01);
SimpleKalmanFilter KF_Right(2, 2, 0.01);

//looping the sensors
void sensorCycle() {
  leftSensor = PingSonar(3,2);
//  delay(30);
  centerSensor = PingSonar(5,4);
//  delay(30);
  rightSensor = PingSonar(7,6);
}
 
//Apply Kalman Filter to sensor reading.
void applyKF() {
  leftSensorKalman = KF_Left.updateEstimate(leftSensor);
  centerSensorKalman = KF_Center.updateEstimate(centerSensor);
  rightSensorKalman = KF_Right.updateEstimate(rightSensor);
}
  
void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = MIN_RANGE;
  range_name.max_range = MAX_RANGE;
}
 
//Create three instances for range messages.
sensor_msgs::Range range_left;
sensor_msgs::Range range_center;
sensor_msgs::Range range_right;
 
//Create publisher onjects for all sensors
ros::Publisher pub_range_left("/robot/sonar_sensor/link_sonar_left", &range_left);
ros::Publisher pub_range_center("/robot/sonar_sensor/link_sonar_center", &range_center);
ros::Publisher pub_range_right("/robot/sonar_sensor/link_sonar_right", &range_right);
    
void setup() {
  Sensors_init();
  nh.initNode();
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_center);
  nh.advertise(pub_range_right);
}

void loop() {
  sensorCycle();
  applyKF();
  range_left.range   = leftSensorKalman*0.01;
  range_center.range = centerSensorKalman*0.01; 
  range_right.range  = rightSensorKalman*0.01;
 
  range_left.header.stamp = nh.now();
  range_center.header.stamp = nh.now();
  range_right.header.stamp = nh.now();
 
  pub_range_left.publish(&range_left);
  pub_range_center.publish(&range_center);
  pub_range_right.publish(&range_right);
 
  nh.spinOnce();//Handle ROS events
}
//------------Init sensor-----------
void Sensors_init(){
  pinMode(trigPinLeft, OUTPUT); 
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinCenter, OUTPUT); 
  pinMode(echoPinCenter, INPUT);
  pinMode(trigPinRight, OUTPUT); 
  pinMode(echoPinRight, INPUT); 

  sensor_msg_init(range_left, "link_sonar_left");
  sensor_msg_init(range_center, "link_sonar_center");
  sensor_msg_init(range_right, "link_sonar_right");
}
uint8_t PingSonar(int trigPin, int echoPin){
  long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}
