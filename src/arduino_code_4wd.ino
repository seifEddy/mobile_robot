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

//Encoders

volatile int encoderPos,encoderPos1,encoderPos2,encoderPos3 = 0;
volatile boolean aSetLast,aSetLast1,aSetLast2,aSetLast3 = false;
volatile boolean bSetLast,bSetLast1,bSetLast2,bSetLast3 = false;
int a;

//Setup ROS stuff

ros::NodeHandle nh;


std_msgs::Int32 encoder_pos1;
std_msgs::Int32 encoder_pos2;
std_msgs::Int32 encoder_pos3;
std_msgs::Int32 encoder_pos4;

ros::Publisher motor1_pub("motor1_encoder", &encoder_pos1);
ros::Publisher motor2_pub("motor2_encoder", &encoder_pos2);
ros::Publisher motor3_pub("motor3_encoder", &encoder_pos3);
ros::Publisher motor4_pub("motor4_encoder", &encoder_pos4);

byte PWM1R_value;
byte PWM1L_value;
byte PWM2R_value;
byte PWM2L_value;
byte PWM3R_value;
byte PWM3L_value;
byte PWM4R_value;
byte PWM4L_value;

const int max_velocity = 309;
const float L = 0.435; // distance between the left and right wheels
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

  /*Serial.print("[ ");
  Serial.print(vl);
  Serial.print(",");
  Serial.print(" ");
  Serial.print(vr);
  Serial.print(",****,");
  Serial.print(" ");
  Serial.print(v);
  Serial.print(",");
  Serial.print(" ");
  Serial.print(w);
  Serial.println(" ]");*/
 
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_callback);

void setup() {
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);
  // pinMode(ENC2A, INPUT_PULLUP);
  // pinMode(ENC2B, INPUT_PULLUP);
  // pinMode(ENC3A, INPUT_PULLUP);
  // pinMode(ENC3B, INPUT_PULLUP);
  pinMode(ENC4A, INPUT_PULLUP);
  pinMode(ENC4B, INPUT_PULLUP);
  
  pinMode(R_en1, OUTPUT);
  pinMode(L_en1, OUTPUT);
  // pinMode(R_en2, OUTPUT);
  // pinMode(L_en2, OUTPUT);
  // pinMode(R_en3, OUTPUT);
  // pinMode(L_en3, OUTPUT);
  pinMode(R_en4, OUTPUT);
  pinMode(L_en4, OUTPUT);
  
  pinMode(PWM1R, OUTPUT);
  pinMode(PWM1L, OUTPUT);
  // pinMode(PWM2R, OUTPUT);
  // pinMode(PWM2L, OUTPUT);
  // pinMode(PWM3R, OUTPUT);
  // pinMode(PWM3L, OUTPUT);
  pinMode(PWM4R, OUTPUT);
  pinMode(PWM4L, OUTPUT);

 attachInterrupt(digitalPinToInterrupt(ENC1A), encoder, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ENC2A), encoder1, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ENC3A), encoder2, CHANGE);
 attachInterrupt(digitalPinToInterrupt(ENC4A), encoder3, CHANGE);

 // attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder, CHANGE);

  Serial.begin(9600);
  nh.initNode();
  nh.advertise(motor1_pub);
  // nh.advertise(motor2_pub);
  // nh.advertise(motor3_pub);
  nh.advertise(motor4_pub);

  nh.subscribe(twist_sub);
}

void loop() {
  
  digitalWrite(R_en1, HIGH);
  digitalWrite(L_en1, HIGH);
  // digitalWrite(R_en2, HIGH);
  // digitalWrite(L_en2, HIGH);
  // digitalWrite(R_en3, HIGH);
  // digitalWrite(L_en3, HIGH);
  digitalWrite(R_en4, HIGH);
  digitalWrite(L_en4, HIGH);

  analogWrite(PWM1R, PWM1R_value); 
  analogWrite(PWM1L, PWM1L_value); 
  // analogWrite(PWM2R, PWM2R_value); 
  // analogWrite(PWM2L, PWM2L_value); 
  // analogWrite(PWM3R, PWM3R_value); 
  // analogWrite(PWM3L, PWM3L_value); 
  analogWrite(PWM4R, PWM4R_value); 
  analogWrite(PWM4L, PWM4L_value); 

  /*
  // Muestra la posición actual del encoder
  Serial.print("[ ");
  Serial.print(encoderPos);
  Serial.print(",");
  Serial.print(" ");
  Serial.print(encoderPos1);
  Serial.print(",");
  Serial.print(" ");
  Serial.print(encoderPos2);
  Serial.print(",");
  Serial.print(" ");
  Serial.print(encoderPos3);
  Serial.println(" ]");
  */
  
  encoder_pos1.data = -encoderPos;
  // encoder_pos2.data = -encoderPos1;
  // encoder_pos3.data = -encoderPos2; 
  encoder_pos4.data = -encoderPos3;

  motor1_pub.publish(&encoder_pos1);
  // motor2_pub.publish(&encoder_pos2);
  // motor3_pub.publish(&encoder_pos3);
  motor4_pub.publish(&encoder_pos4);
  
  nh.spinOnce();
  delay(25);
}

void encoder() {
  boolean aSet = digitalRead(ENC1A);
  boolean bSet = digitalRead(ENC1B);

  if (aSet != aSetLast || bSet != bSetLast) {
    if (aSet == bSet) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    aSetLast = aSet;
    bSetLast = bSet;
  }
}
void encoder1() {
  boolean aSet = digitalRead(ENC2A);
  boolean bSet = digitalRead(ENC2B);

  if (aSet != aSetLast1 || bSet != bSetLast1) {
    if (aSet == bSet) {
      encoderPos1++;
    } else {
      encoderPos1--;
    }
    aSetLast1 = aSet;
    bSetLast1 = bSet;
  }
}
void encoder2() {
  boolean aSet = digitalRead(ENC3A);
  boolean bSet = digitalRead(ENC3B);
  a++;
  if (aSet != aSetLast2 || bSet != bSetLast2) {
    if (aSet == bSet) {
      encoderPos2++;
    } else {
      encoderPos2--;
    }
    aSetLast2 = aSet;
    bSetLast2 = bSet;
  }
}
void encoder3() {
  boolean aSet = digitalRead(ENC4A);
  boolean bSet = digitalRead(ENC4B);

  if (aSet != aSetLast3 || bSet != bSetLast3) {
    if (aSet == bSet) {
      encoderPos3++;
    } else {
      encoderPos3--;
    }
    aSetLast3 = aSet;
    bSetLast3 = bSet;
  }
}