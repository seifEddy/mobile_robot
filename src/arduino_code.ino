#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include <ros.h>
#include <std_msgs/Int32.h>

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 4
#define PWM1 5
#define IN2 6
#define IN1 7
#define IN3 8
#define IN4 9

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

ros::NodeHandle nh;

std_msgs::Int32 encoder_pos;
ros::Publisher motor_pub("motor_encoder", &encoder_pos)

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT); 
  Serial.println("target pos");
  nh.initNode();
  nh.advertise(motor_pub);
}

void loop() {

  // set target position
  int target = 700;
  //int target = 5000*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  encoder_pos.data = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoder_pos.data = posi;
  }

    motor_pub.publish(&encoder_pos);
    nh.spinOnce();
    delay(50);

  
  // error
  int e = encoder_pos.data - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2,IN3,IN4,PWM1);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(encoder_pos.data);
  Serial.print(" ");
  Serial.print(pwr*dir);
  Serial.println();
  
}

void setMotor(int dir, int ??, int pwm, int in1, int in2,int in3, int in4,int pwm1){
  
  if(dir == 1){
    analogWrite(pwm,pwmVal);
  //  analogWrite(pwm1,0);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    }

  
  else
  {
  //  analogWrite(pwm,0);
    analogWrite(pwm1,pwmVal);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
  }  
  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}