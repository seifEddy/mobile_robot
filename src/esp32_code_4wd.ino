#include <ESP32Encoder.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// Pin definitions adapted for ESP32
#define ENC1A 32
#define ENC1B 33
#define ENC2A 25
#define ENC2B 26
#define ENC3A 27
#define ENC3B 14
#define ENC4A 12
#define ENC4B 13

#define PWM1R 23
#define PWM1L 22
#define PWM2R 19
#define PWM2L 18
#define PWM3R 5
#define PWM3L 17
#define PWM4R 16
#define PWM4L 4

// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// ROS node handle
ros::NodeHandle nh;

// Encoder position messages
std_msgs::Int32 encoder_pos1_msg;
std_msgs::Int32 encoder_pos2_msg;
std_msgs::Int32 encoder_pos3_msg;
std_msgs::Int32 encoder_pos4_msg;

// ROS publishers
ros::Publisher motor1_pub("motor1_encoder", &encoder_pos1_msg);
ros::Publisher motor2_pub("motor2_encoder", &encoder_pos2_msg);
ros::Publisher motor3_pub("motor3_encoder", &encoder_pos3_msg);
ros::Publisher motor4_pub("motor4_encoder", &encoder_pos4_msg);

// PWM control variables
byte PWM1R_value = 0;
byte PWM1L_value = 0;
byte PWM2R_value = 0;
byte PWM2L_value = 0;
byte PWM3R_value = 0;
byte PWM3L_value = 0;
byte PWM4R_value = 0;
byte PWM4L_value = 0;

const int max_velocity = 309;
const float L = 0.444; // distance between the left and right wheels
const int counts_per_rev = 140;
const float wheel_radius = 0.055;

void twist_callback(const geometry_msgs::Twist& msg)
{
   float vl;
   float vr;

   // Linear velocity is msg.linear.x
   // Angular velocity is msg.angular.z
   float v = msg.linear.x;
   float w = msg.angular.z;

   
   vl = counts_per_rev * (v - 0.5 * w * L) / (2 * PI * wheel_radius);
   vr = counts_per_rev * (v + 0.5 * w * L) / (2 * PI * wheel_radius);
   if(abs(vr) > 255)
   {
      vr = (vr / abs(vr)) * 255;
   }
   
   if (vr >= 0)
   {
      PWM1R_value = vr;
      PWM1L_value = 0;
      PWM2R_value = vr;
      PWM2L_value = 0;
   }
   else
   {
      PWM1R_value = 0;
      PWM1L_value = abs(vr);
      PWM2R_value = 0;
      PWM2L_value = abs(vr);
   }

   if(abs(vl) > 255)
   {
      vl = (vl / abs(vl)) * 255;
   }

   if (vl >= 0)
   {
      PWM4R_value = vl;
      PWM4L_value = 0.0;
      PWM3R_value = vl;
      PWM3L_value = 0.0;
   }
   else
   {
      PWM4R_value = 0.0;
      PWM4L_value = abs(vl);
      PWM3R_value = 0.0;
      PWM3L_value = abs(vl);
   }
 
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_callback);

void setup() {
  // Initialize encoder pins
  encoder1.attachFullQuad(ENC1A, ENC1B);
  encoder2.attachFullQuad(ENC2A, ENC2B);
  encoder3.attachFullQuad(ENC3A, ENC3B);
  encoder4.attachFullQuad(ENC4A, ENC4B);

  // Set all encoders' count to zero
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder3.setCount(0);
  encoder4.setCount(0);

  // Initialize PWM pins
  pinMode(PWM1R, OUTPUT);
  pinMode(PWM1L, OUTPUT);
  pinMode(PWM2R, OUTPUT);
  pinMode(PWM2L, OUTPUT);
  pinMode(PWM3R, OUTPUT);
  pinMode(PWM3L, OUTPUT);
  pinMode(PWM4R, OUTPUT);
  pinMode(PWM4L, OUTPUT);

  // Initialize ROS
  nh.initNode();
  nh.advertise(motor1_pub);
  nh.advertise(motor2_pub);
  nh.advertise(motor3_pub);
  nh.advertise(motor4_pub);
  nh.subscribe(twist_sub);

//   Serial.begin(115200); // Start serial communication at 115200 baud
}

void loop() {
  // Read encoder positions
  encoder_pos1_msg.data = encoder1.getCount();
  encoder_pos2_msg.data = encoder2.getCount();
  encoder_pos3_msg.data = encoder3.getCount();
  encoder_pos4_msg.data = encoder4.getCount();

  // Publish encoder readings
  motor1_pub.publish(&encoder_pos1_msg);
  motor2_pub.publish(&encoder_pos2_msg);
  motor3_pub.publish(&encoder_pos3_msg);
  motor4_pub.publish(&encoder_pos4_msg);

  // Set motor PWM values
  // Example: analogWrite(PWM1R, PWM1R_value);
  // Note: You might need to use ledcWrite on ESP32 for PWM control

  nh.spinOnce(); // Handle ROS communication
  delay(10); // Short delay to ensure stability
}