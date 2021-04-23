#include <Wire.h>
#include "SR04.h" //Sonar
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>
#include <EEPROM.h>

class PID{
  private:
    
    
  public:

    double kp, ki, kd;
    double target, error;
    double errorOld, errorChange, errorSlope, errorArea;
    int maxCorrection;
    long oldTime, newTime, changeTime;
    double tempi;

    PID(double target, double kp, double ki, double kd, int maxCorrection){
      this->target = target;
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->maxCorrection = maxCorrection;
    }

    void start(){
      this->error = 0;
      this->errorSlope = 0;
      this->errorArea = 0;
      this->errorOld = 0;
      this->tempi = 0;
      newTime = millis();
    }
  
    double getOutput(double measured){
      oldTime = newTime;
      newTime = millis();
      changeTime = newTime - oldTime;
      
      errorOld = error;
      error = target - measured;
      errorChange = error-errorOld;
      errorSlope = errorChange/changeTime;
      errorArea += error*changeTime;      

      double correction = (kp*error + ki*errorArea + kd*errorSlope);

      if((abs(correction) > maxCorrection)){
        correction = (correction/abs(correction))*maxCorrection;
      }
//      if(((abs(correction) > maxCorrection) && (correction/abs(correction) == error/abs(error)))){
//        correction = (kp*error + tempi + kd*errorSlope);
//      }else tempi = ki*errorArea;
      
      return correction;
    }

    void setTarget(double target){
      this->target = target;
    }

    void setGain(int PID, double gain){
      switch(PID){
        case 0: kp = gain;
        break;
        case 1: ki = gain;
        break;
        case 2: kd = gain;
        break;
        default: return;
      }
    }
};


#define BNO055_SAMPLERATE_DELAY_MS 100

//Constants
#define TRIGGER_PIN 12
#define ECHO_PIN 2
#define PULSE_TIMEOUT 37500L  // 100ms

#define FRONT_RIGHT 3
#define FRONT_LEFT 4
#define BACK_RIGHT 5
#define BACK_LEFT 6

#define KILLSWITCH 15

#define THROTTLE_MAX 1800
#define OUTPUT_MAX 2000

Adafruit_BNO055 bno = Adafruit_BNO055();

SR04 sonar = SR04(ECHO_PIN,TRIGGER_PIN);

double rpkp = 0.5,
       rpki = 0.0,
       rpkd = 250,
       rpmax = 250;

double ykp = 5,
       yki = 0,
       ykd = 0,
       ymax = 75;

double akp = 3,
       aki = 0.03,
       akd = 400,
       amax = 400;
                                              //setpoint unit
PID rollPID = PID(0, rpkp, rpki, rpkd,rpmax); //degrees
PID pitchPID = PID(0, rpkp, rpki, rpkd,rpmax);//degrees
PID yawPID = PID(0, ykp, yki, ykd, ymax);     //degrees
PID altPID = PID(10,akp, aki, akd, amax);     //centimeters

//Sonar
double distance;
double prevDist;

boolean startup = true;
boolean isFlying = false;

//calibration
boolean ESCsCalibrated = true;

//Code for ESC's and Motors
int motorPins[] = {FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT};
int throttle = 1300;
int adjusted_throttle[4]; // {FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT}

double pitchCorrection, rollCorrection, yawCorrection, altCorrection;
double pitchTarget, rollTarget, yawTarget;

uint8_t calibData;

imu::Quaternion quat;
imu::Vector<3> euler;

Servo motor[4];

unsigned long timer;

void setup() {

  Serial.begin(115200);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  
  //attaching the Servo instances to the pins of the motors
  for(int i = 0; i < 4; i++){
    motor[i].attach(motorPins[i]);
  }

  delay(1200);//BNO055_SAMPLERATE_DELAY_MS);
  calibData = EEPROM.read(0);    
  bno.setSensorOffsets(calibData);
  delay(2000);

  if(!ESCsCalibrated) ESC_cal();

  quat = bno.getQuat();
  euler = quat.toEuler();

  pitchTarget = euler[1]*RAD_TO_DEG;
  rollTarget = euler[2]*RAD_TO_DEG;
  yawTarget = euler[0]*RAD_TO_DEG;
  
  pitchPID.setTarget(pitchTarget);
  rollPID.setTarget(rollTarget);
  yawPID.setTarget(yawTarget);

  pitchPID.start();
  rollPID.start();
  yawPID.start();
  altPID.start();

  timer = millis();
}

void loop() {

  double gain = map(analogRead(A1),0,1023,0,100);
  gain = 1 * (gain/100);
  //pitchPID.setGain(2,gain+150);

  uint8_t system, gyro, accel, mg = 0;
  bno.getCalibration(&system, &gyro, &accel, &mg);

  quat = bno.getQuat();
  euler = quat.toEuler();

  
  pitchCorrection = pitchPID.getOutput((euler[1]*RAD_TO_DEG));
  rollCorrection = rollPID.getOutput((euler[2]*RAD_TO_DEG));
  yawCorrection = yawPID.getOutput((euler[0]*RAD_TO_DEG));

  //distance = correctSonar();
  long duration = 0;
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
    distance = MicrosecondsToCentimeter(duration);
  
  altCorrection = altPID.getOutput((double)distance);

  //altCorrection = 0;
//  Correction = 0;

  throttle = 1400;
  
  throttle = map(analogRead(A0),0,1023,1000,THROTTLE_MAX);

  int altTarg = map(analogRead(A1),0,1023,5,15);
  altPID.setTarget(altTarg);

  adjusted_throttle[0] = throttle + altCorrection - pitchCorrection - rollCorrection - yawCorrection;  //front right  
  adjusted_throttle[1] = throttle + altCorrection - pitchCorrection + rollCorrection + yawCorrection;  //front left
  adjusted_throttle[2] = throttle + altCorrection + pitchCorrection - rollCorrection + yawCorrection;  //back right
  adjusted_throttle[3] = throttle + altCorrection + pitchCorrection + rollCorrection - yawCorrection;  //back left


  isFlying = digitalRead(KILLSWITCH);

  if(millis()-timer < 3000){
    isFlying = false;
  }
  

  

  for(int i = 0; i < 4; i++){
    adjusted_throttle[i] = constrain(adjusted_throttle[i], 1100, OUTPUT_MAX);
    
    if(!isFlying||throttle<1050) adjusted_throttle[i] = 1000;
  }


  for(int i = 0; i < 4; i++) motor[i].writeMicroseconds(adjusted_throttle[i]);

//  Serial.print(system);
//  Serial.print(",");
//  Serial.print(gyro);
//  Serial.print(",");
//  Serial.print(accel);
//  Serial.print(",");
//  Serial.print(mg);
//  Serial.print(",");
  Serial.print((euler[1]*RAD_TO_DEG));
  Serial.print(",");
  Serial.print((euler[2]*RAD_TO_DEG));
  Serial.print(",");
  Serial.print((euler[0]*RAD_TO_DEG));
  Serial.print(",");
//  Serial.print(adjusted_throttle[0]/10);
//  Serial.print(",");
//  Serial.print(adjusted_throttle[1]/10);
//  Serial.print(",");
//  Serial.print(adjusted_throttle[2]/10);
//  Serial.print(",");
//  Serial.print(adjusted_throttle[3]/10);
//  Serial.print(",");
//  Serial.print(altCorrection);
//  Serial.print(",");
//  Serial.print(distance);
//  Serial.print(","),
//  Serial.print(pitchCorrection);
//  Serial.print(","),
//  Serial.print(rollCorrection);
//  Serial.print(","),
  Serial.print(gain);
  Serial.print(",");
  Serial.print(altTarg);
  Serial.print(",");
  Serial.print(distance);
  Serial.print(",");
  Serial.print(pitchTarget);
  Serial.print(",");
  Serial.print(rollTarget);
  Serial.print(",");
  Serial.print(yawTarget);
  Serial.print(",");
  
  //Serial.println((micros()-timer));
  
  Serial.println("");

  //delay(BNO055_SAMPLERATE_DELAY_MS);
  

}

int correctSonar() {
  
  int Maximum = 400;
  int Minimum = 6; 

  int dist = sonar.Distance();
  
  if(dist < Maximum && dist > Minimum) {
   prevDist = dist;  
   return dist;   
  }

  return prevDist;
}

double MicrosecondsToCentimeter(long duration){
  double d = (duration*100.0)/5882.0;
  return d;
}

void ESC_cal(){

  Serial.print("Calibrating ESCs...");
  
  int start_time = millis(); // record starting time

  while(millis() - start_time < 20000){ // calibration takes around 30 seconds
    if(millis()-start_time<8000){ // set throttle high for 15 seconds
    
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
