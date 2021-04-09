//Libraries
#include "SR04.h" //Sonar
#include <Adafruit_Sensor.h> //Generic Adafruit
#include <Adafruit_BNO055.h> //IMU
#include <Servo.h> //Motor Controll

//Constants
#define TRIGGER_PIN 12
#define ECHO_PIN 2

#define FRONT_RIGHT 3
#define FRONT_LEFT 4
#define BACK_RIGHT 5
#define BACK_LEFT 6

#define KILLSWITCH 19

//#define RAD_TO_DEG 180.0/3.14159
//#define DEG_TO_RAD 3.14159/180.0
#define HZ250 1.0/250.0

#define THROTTLE_MAX 1800
#define OUTPUT_MAX 2000

//Global Variables

unsigned long timer;
boolean startup = true;
boolean isFlying = false;

//calibration
boolean ESCsCalibrated = false;

//Sonar
long distance;
long prevDist;

//PID

boolean auto_level = true; // Turn PID on/off

float error_temp;

//roll
float roll_p_gain = 2.5;
float roll_i_gain = 0.1;
float roll_d_gain = 6.0;
int roll_max = 400;
float roll_input, roll_setpoint, roll_i, roll_d, roll_output;

//pitch
float pitch_p_gain = roll_p_gain;
float pitch_i_gain = roll_i_gain;
float pitch_d_gain = roll_d_gain;
int pitch_max = roll_max;
float pitch_input, pitch_setpoint, pitch_i, pitch_d, pitch_output;

//yaw
float yaw_p_gain = 4.0;
float yaw_i_gain = 0.02;
float yaw_d_gain = 0.0;
int yaw_max = 400;
float yaw_input, yaw_setpoint, yaw_i, yaw_d, yaw_output;\


//Code for ESC's and Motors
int motorPins[] = {FRONT_RIGHT, BACK_RIGHT, BACK_LEFT, FRONT_LEFT};
int throttle = 0;
int adjusted_throttle[4]; // {FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT}

float gyro_avg[3];
float gyro_axis[3];
float gyro_yaw,gyro_pitch,gyro_roll;

float accel_axis[3];
float accel_avg[3];
float ax,ay,az,acc_pitch,acc_roll,acc_yaw,acc_total;

float pitch,yaw,roll;

//Create servo instance for each motor
Servo motor[4];

//Create sonar instance
//SR04 sonar = SR04(ECHO_PIN,TRIGGER_PIN);

//Create IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//create IMU event
sensors_event_t event;


void setup(){

  Serial.begin(115200);

  //Kill switch interupt
  //attachInterrupt(digitalPinToInterrupt(KILLSWITCH), kill, LOW); //kill when signal is tied to gnd

  //attaching the Servo instances to the pins of the motors
  for(int i = 0; i < 4; i++){
    motor[i].attach(motorPins[i]);
  }

  if(!ESCsCalibrated) ESC_cal();
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);

  //look into calibration here: https://www.youtube.com/watch?v=Bw0WuAyGsnY

  //Average the output of the gyroscope over 2000 readings
//  int count = 0;
//  for(count = 0; count < 2000; count++){
//    bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
//    gyro_avg[0] += event.gyro.x;
//    gyro_avg[1] += event.gyro.y;
//    gyro_avg[2] += event.gyro.z;
//    delay(1);
//  }
//  for(int i = 0; i < 3; i++) gyro_avg[i] /= count;
//
//  //Average the output of the accelerometer over 2000 readings
//  count = 0;
//  for(count = 0; count < 2000; count++){
//    bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//    accel_avg[0] += event.acceleration.x;
//    accel_avg[1] += event.acceleration.y;
//    accel_avg[2] += event.acceleration.z;
//    delay(1);
//  }
//  for(int i = 0; i < 3; i++) accel_avg[i] /= count;


  //Start timer for the loop
  timer = micros();
  
}


void loop(){

  //read in gyro events in dps
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro_yaw = event.gyro.x-gyro_avg[0];
  gyro_pitch = event.gyro.y-gyro_avg[1];
  gyro_roll = event.gyro.z-gyro_avg[2];

  //read in accel events in g
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ax = event.acceleration.x;
  ay = event.acceleration.y;
  az = event.acceleration.z;

  //Serial.println(ay);
  //Serial.println(az);

  //find total g vector to find pitch and roll using trig
  acc_total = sqrt((ax*ax)+(ay*ay)+(az*az));
  if(abs(ay) < acc_total) acc_pitch = asin((float)ay/acc_total) * RAD_TO_DEG;
  if(abs(ax) < acc_total) acc_roll = asin((float)ax/acc_total) * RAD_TO_DEG;

  //remove accel calibration from pitch and roll
  //acc_pitch -= accel_avg[1];
  //acc_roll -= accel_avg[2];
  //acc_roll -= 9;

  //PID inputs
  yaw_input = (yaw_input * 0.7) + (gyro_yaw * 0.3);
  pitch_input = (pitch_input * 0.7) + (gyro_pitch * 0.3);
  roll_input = (roll_input * 0.7) + (gyro_roll * 0.3);

  //calculating cumulative pitch and roll
  pitch += gyro_pitch * HZ250;
  roll += gyro_roll * HZ250;

  
  //adjusting pitch and roll for yaw during pitch and roll
  //pitch -= roll * sin(gyro_yaw * HZ250 * DEG_TO_RAD);
  //roll += pitch * sin(gyro_yaw * HZ250 * DEG_TO_RAD);

  
  bno.getEvent(&event);
  pitch = event.orientation.y;
  roll = event.orientation.z;

//  Serial.print("gyro_pitch: ");
//  Serial.print(pitch);
//  Serial.print("    gyro_roll: ");
//  Serial.print(roll);
//  Serial.print("  acc_pitch: ");
//  Serial.print(acc_pitch);
//  Serial.print("    acc_roll: ");
//  Serial.print(acc_roll);

  //combine gyro and accel to minimize drift
//  pitch = pitch*0.9996 + acc_pitch*0.0004;
//  roll = roll*0.9996 + acc_roll*0.0004;

  Serial.print("  pitch: ");
  Serial.print(pitch);
  Serial.print("    roll: ");
  Serial.println(roll);

  /////Sets maximum commanded tilt of the drone if using some kind of transmitter. 15 ~= 33 deg///////
  /*
  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction
  */

  if(startup){

    for(int i = 0; i < 4; i++) motor[i].writeMicroseconds(1000);
    
    pitch = acc_pitch;
    roll = acc_roll;

    //Reset PID controllers
    roll_i = 0;
    roll_d = 0;
    pitch_i = 0;
    pitch_d = 0;
    yaw_i = 0;
    yaw_d = 0;

    isFlying = true;
    startup = false;
  }

  
  isFlying = digitalRead(KILLSWITCH);


  //Set axis to 0 to maintain a hover
  roll_setpoint = 0;
  pitch_setpoint = 0;
  yaw_setpoint = 0;

  calculate_pid();

  throttle = analogRead(A0);
  throttle = map(throttle,0,1023,1000,THROTTLE_MAX);

  //Serial.print("throttle");
  //Serial.print(throttle);

  adjusted_throttle[0] = throttle + pitch_output + roll_output + yaw_output;
  adjusted_throttle[1] = throttle - pitch_output + roll_output - yaw_output;
  adjusted_throttle[2] = throttle - pitch_output - roll_output + yaw_output;
  adjusted_throttle[3] = throttle + pitch_output - roll_output - yaw_output;

  for(int i = 0; i < 4; i++){
    adjusted_throttle[i] = constrain(adjusted_throttle[i], 1100, OUTPUT_MAX);
//    Serial.print("  Throttle: ");
//    Serial.print(i);
//    Serial.print(": ");
//    Serial.print(adjusted_throttle[i]);
  }
//  Serial.println("|");

  if(!isFlying) for(int i = 0; i < 4; i++) adjusted_throttle[i] = 1000;

  while(micros() - timer < 4000);
  timer = micros();

  for(int i = 0; i < 4; i++) motor[i].writeMicroseconds(adjusted_throttle[i]);

    
  
}

void calculate_pid(){

  error_temp = roll_input - roll_setpoint;

  roll_i += roll_i_gain * error_temp;
  if(roll_i > roll_max){
    roll_i = roll_max;
  }else if(roll_i < roll_max * -1) roll_i = roll_max * -1;

  roll_output = (roll_p_gain * error_temp) + roll_i + (roll_d_gain * (error_temp - roll_d));
  if(roll_output > roll_max){
    roll_output = roll_max;
  }else if(roll_output < roll_max * -1) roll_output = roll_max * -1;

  roll_d = error_temp;


  error_temp = pitch_input - pitch_setpoint;

  pitch_i += pitch_i_gain * error_temp;
  if(pitch_i > pitch_max){
    pitch_i = pitch_max;
  }else if(pitch_i < pitch_max * -1) pitch_i = pitch_max * -1;

  pitch_output = (pitch_p_gain * error_temp) + pitch_i + (pitch_d_gain * (error_temp - pitch_d));
  if(pitch_output > pitch_max){
    pitch_output = pitch_max;
  }else if(pitch_output < pitch_max * -1) pitch_output = pitch_max * -1;

  pitch_d = error_temp;


  error_temp = yaw_input - yaw_setpoint;

  yaw_i += yaw_i_gain * error_temp;
  if(yaw_i > yaw_max){
    yaw_i = yaw_max;
  }else if(yaw_i < yaw_max * -1) yaw_i = yaw_max * -1;

  yaw_output = (yaw_p_gain * error_temp) + yaw_i + (yaw_d_gain * (error_temp - yaw_d));
  if(yaw_output > yaw_max){
    yaw_output = yaw_max;
  }else if(yaw_output < yaw_max * -1) yaw_output = yaw_max * -1;

  yaw_d = error_temp;
  
}

void ESC_cal(){

  Serial.print("Calibrating ESCs...");
  
  int start_time = millis(); // record starting time

  while(millis() - start_time < 30000){ // calibration takes around 30 seconds
    if(millis()-start_time<15000){ // set throttle high for 15 seconds
    
      for(int i = 0; i < 4; i++){
        motor[i].writeMicroseconds(2000); //2000 is max throttle
      }
    
      //Serial.println("HIGH");
    
    }
    else { // set throttle low for 15 seconds
         
      for(int i = 0; i < 4; i++){
        motor[i].writeMicroseconds(1000); //1000 is zero throttle
      }
      
      //Serial.println("LOW");
      
    }

    if((millis()-start_time)%100==0) Serial.print(".");
    
    delay(5);
  }

  Serial.println("ESCs Calibrated");
  ESCsCalibrated = true;
  
}

void kill(){
  isFlying = false;
}
