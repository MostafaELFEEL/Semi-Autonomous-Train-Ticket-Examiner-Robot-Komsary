#include<Arduino.h>
#include "PinChangeInterrupt.h"
#include <Wire.h>



//Right motor
#define encoder_1_A 2
#define encoder_1_B 3
#define PWM1 5
#define IN1 7
#define IN2 8




//Left motor
#define encoder_2_A 12
#define encoder_2_B 11
#define PWM2 6
#define IN3 4
#define IN4 9





volatile int pos_i_L = 0;
void readEncoder_L() {
  int b = digitalRead(encoder_1_B);
  if (b > 0) {
    // If B is high, increment forward
    pos_i_L ++;
  }
  else {
    // Otherwise, increment backward
    pos_i_L --;
  }
}



volatile int pos_i_R = 0;
void readEncoder_R() {
  int b = digitalRead(encoder_2_B);
  if (b > 0) {
    // If B is high, increment forward
    pos_i_R ++;
  }
  else {
    // Otherwise, increment backward
    pos_i_R --;
  }
}



int pos_old_R = 0;
long time_old_R = 0;
float omega_old_R = 0;
float error_old_R = 0;
float omega_target_R = 0;
float omega_filtered_R = 0;
float error_integral_R = 0;



int pos_old_L = 0;
long time_old_L = 0;
float omega_old_L = 0;
float error_old_L = 0;
float omega_target_L = 0;
float error_integral_L = 0;
float omega_filtered_L = 0;

float velocity_final_L=0;
float velocity_final_R=0;

int PID_R() {
  int pos = pos_i_R;
  long time_new = micros();
  float Time_delta = ((float) (time_new - time_old_R)) / 1000000;
  float velocity = (pos - pos_old_R) / Time_delta;
  float omega_new = velocity / 320 * 60.0; // Convert ticks/sec to RPM

  omega_filtered_R = 0.854 * omega_filtered_R + 0.0728 * omega_new + 0.0728 * omega_old_R; // Low-pass filter (25 Hz cutoff)

  float kp = 2.5;//2.5;
  float ki = 0.5;//0.5;
  float kd = 0.08;
  float error = omega_target_R - omega_filtered_R;
  error_integral_R = error_integral_R + (error * Time_delta);     //(1/s)error_new=(1/s)error_old + error*time  (d_new=d_old+v*t)
  float error_dot = (error - error_old_R) / (Time_delta);         // de/dt = delta_e/delta_time
  velocity_final_R = kp * error + ki * error_integral_R + error_dot * kd;
  pos_old_R = pos;
  time_old_R = time_new;
  omega_old_R = omega_new;
  error_old_R = error;
  return velocity_final_R;
}



int PID_L() {
  int pos = pos_i_L;
  long time_new = micros();
  float Time_delta = ((float) (time_new - time_old_L)) / 1000000;
  float velocity = (pos - pos_old_L) / Time_delta;
  float omega_new = velocity / 320 * 60.0;  // Convert count/s to RPM

  omega_filtered_L = 0.854 * omega_filtered_L + 0.0728 * omega_new + 0.0728 * omega_old_L;  // Low-pass filter (25 Hz cutoff)

  float kp = 2.5;//2.5;
  float ki = 0.5;//0.8;
  float kd = 0.08;
  float error = omega_target_L - omega_filtered_L;
  error_integral_L = error_integral_L + (error * Time_delta);   //(1/s)error_new=(1/s)error_old + error*time  (d_new=d_old+v*t)
  float error_dot = (error - error_old_L) / (Time_delta);      // de/dt = delta_e/delta_time
  velocity_final_L = kp * error + ki * error_integral_L + error_dot * kd;
  pos_old_L = pos;
  time_old_L = time_new;
  error_old_L = error;
  omega_old_L = omega_new;
  return velocity_final_L;
}



void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}



void setup() {
  Wire.begin(0x8);
  
  // Call receiveEvent when data received                
  Wire.onReceive(receiveEvent);
  
  Serial.begin(9600);

  pinMode(encoder_1_A, INPUT);
  pinMode(encoder_1_B, INPUT);

  pinMode(encoder_2_A, INPUT);
  pinMode(encoder_2_B, INPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(PWM2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

 
  attachInterrupt(digitalPinToInterrupt(encoder_1_A), readEncoder_L, RISING);
  attachPCINT(digitalPinToPCINT(encoder_2_A), readEncoder_R, RISING);
}

void receiveEvent(int howMany) {
  while (Wire.available()) { 
    int c = Wire.read(); 
switch(c){
  case 1:
    omega_target_L =0 ; //Read user input and hold it in a variable
    omega_target_R =0;
    break;
    case 2:
    omega_target_L =80 ; //Read user input and hold it in a variable
    omega_target_R =80;
    break;
    case 3:
    omega_target_L =80  ;//Read user input and hold it in a variable
    omega_target_R =50;
    break;
    case 4:
    omega_target_L =50  ;//Read user input and hold it in a variable
    omega_target_R =80;
    break;
    
 
}
  }
}

void loop() {

    
///  if (Serial.available() > 0) {
  //  omega_target_L = Serial.parseFloat();  //Read user input and hold it in a variable
  //  omega_target_R = Serial.parseFloat();  //Read user input and hold it in a variable
  //}
  // Set the motor speed and direction
  int pwr_R = PID_R();
  int pwr_l = PID_L();
  int dir_R = 1;
  int dir_l = 1;

  if (pwr_R < 0) {
    dir_R = -1;
  }
  if (pwr_R > 255) {
    pwr_R = 255;
  }
  if (pwr_l < 0) {
    dir_l = -1;
  }
  if (pwr_l > 255) {
    pwr_l = 255;
  }

  setMotor(dir_l, pwr_l, PWM1, IN1, IN2);
  setMotor(dir_R, pwr_R, PWM2, IN3, IN4);

  Serial.println(omega_target_R);
  Serial.print(" ");
  Serial.print(omega_target_L);
  Serial.print(" ");
  Serial.print(omega_filtered_R);
  Serial.print(" ");
  Serial.print(omega_filtered_L);
  Serial.print(" ");
}
