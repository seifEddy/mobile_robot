#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// Definición de pines
#define ENC1A 20
#define ENC1B 41
#define ENC2A 19
#define ENC2B 39
#define ENC3A 21
#define ENC3B 37
#define ENC4A 18
#define ENC4B 44

#define PWM1R 6
#define PWM1L 7
#define PWM2R 4
#define PWM2L 5
#define PWM3R 2
#define PWM3L 3
#define PWM4R 8
#define PWM4L 9

#define R_en1 29
#define L_en1 31
#define R_en2 27
#define L_en2 28
#define R_en3 24
#define L_en3 25
#define R_en4 35
#define L_en4 33

// Encoders
volatile long encoderPos[4] = {0, 0, 0, 0}; // Encoder positions for 4 motors
volatile byte lastEncoded[4] = {0, 0, 0, 0}; // Last encoded values for 4 motors

// ROS setup
ros::NodeHandle nh;

std_msgs::Int32 encoder_pos_msgs[4]; // Encoder position messages for 4 motors
ros::Publisher motor_pubs[4] = { // Publishers for 4 motors
  ros::Publisher("motor1_encoder", &encoder_pos_msgs[0]),
  ros::Publisher("motor2_encoder", &encoder_pos_msgs[1]),
  ros::Publisher("motor3_encoder", &encoder_pos_msgs[2]),
  ros::Publisher("motor4_encoder", &encoder_pos_msgs[3])
};

// PWM values for motor control
byte PWM_values[8] = {0};

const int max_velocity = 309;
const float L = 0.444; // distance between the left and right wheels
const int counts_per_rev = 140;
const float wheel_radius = 0.055;

// Update encoder count, generalized for all encoders
void updateEncoder(int index, int pinA, int pinB) {
  int MSB = digitalRead(pinA); // MSB = most significant bit
  int LSB = digitalRead(pinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded[index] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos[index]++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos[index]--;

  lastEncoded[index] = encoded;
}

// Interrupt service routines for encoders
void ISR_updateEncoder0() { updateEncoder(0, ENC1A, ENC1B); }
void ISR_updateEncoder1() { updateEncoder(1, ENC2A, ENC2B); }
void ISR_updateEncoder2() { updateEncoder(2, ENC3A, ENC3B); }
void ISR_updateEncoder3() { updateEncoder(3, ENC4A, ENC4B); }

void twist_callback(const geometry_msgs::Twist& msg) {
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
      PWM_values[0] = vr;
      PWM_values[1] = 0;
      PWM_values[2] = vr;
      PWM_values[3] = 0;
   }
   else
   {
      PWM_values[0] = 0;
      PWM_values[1] = abs(vr);
      PWM_values[2] = 0;
      PWM_values[3] = abs(vr);
   }

   if(abs(vl) > 255)
   {
      vl = (vl / abs(vl)) * 255;
   }

   if (vl >= 0)
   {
      PWM_values[6] = vl;
      PWM_values[7] = 0.0;
      PWM_values[4] = vl;
      PWM_values[5] = 0.0;
   }
   else
   {
      PWM_values[6] = 0.0;
      PWM_values[7] = abs(vl);
      PWM_values[4] = 0.0;
      PWM_values[5] = abs(vl);
   }
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_callback);

void setup() {
  // Pin setup code
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);
  pinMode(ENC2A, INPUT_PULLUP);
  pinMode(ENC2B, INPUT_PULLUP);
  pinMode(ENC3A, INPUT_PULLUP);
  pinMode(ENC3B, INPUT_PULLUP);
  pinMode(ENC4A, INPUT_PULLUP);
  pinMode(ENC4B, INPUT_PULLUP);
  
  pinMode(R_en1, OUTPUT);
  pinMode(L_en1, OUTPUT);
  pinMode(R_en2, OUTPUT);
  pinMode(L_en2, OUTPUT);
  pinMode(R_en3, OUTPUT);
  pinMode(L_en3, OUTPUT);
  pinMode(R_en4, OUTPUT);
  pinMode(L_en4, OUTPUT);
  
  pinMode(PWM1R, OUTPUT);
  pinMode(PWM1L, OUTPUT);
  pinMode(PWM2R, OUTPUT);
  pinMode(PWM2L, OUTPUT);
  pinMode(PWM3R, OUTPUT);
  pinMode(PWM3L, OUTPUT);
  pinMode(PWM4R, OUTPUT);
  pinMode(PWM4L, OUTPUT);
  
  // Attach interrupt service routines to encoder signals
  attachInterrupt(digitalPinToInterrupt(ENC1A), ISR_updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), ISR_updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), ISR_updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), ISR_updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3A), ISR_updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3B), ISR_updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4A), ISR_updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4B), ISR_updateEncoder3, CHANGE);

  // ROS setup code
  for (int i = 0; i < 4; i++) {
    nh.advertise(motor_pubs[i]);
  }

  nh.subscribe(twist_sub);
  digitalWrite(R_en1, HIGH);
  digitalWrite(L_en1, HIGH);
  digitalWrite(R_en2, HIGH);
  digitalWrite(L_en2, HIGH);
  digitalWrite(R_en3, HIGH);
  digitalWrite(L_en3, HIGH);
  digitalWrite(R_en4, HIGH);
  digitalWrite(L_en4, HIGH);
}

void loop() {
  // Motor control code
  analogWrite(PWM1R, PWM_values[0]); 
  analogWrite(PWM1L, PWM_values[1]); 
  analogWrite(PWM2R, PWM_values[2]); 
  analogWrite(PWM2L, PWM_values[3]); 
  analogWrite(PWM3R, PWM_values[4]); 
  analogWrite(PWM3L, PWM_values[5]); 
  analogWrite(PWM4R, PWM_values[6]); 
  analogWrite(PWM4L, PWM_values[7]);

  // Publishing encoder counts
  for (int i = 0; i < 4; i++) {
    encoder_pos_msgs[i].data = encoderPos[i];
    motor_pubs[i].publish(&encoder_pos_msgs[i]);
  }

  nh.spinOnce();
  delay(50);
}
