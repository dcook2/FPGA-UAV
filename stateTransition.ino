//library for the sonar
#include "SR04.h"
///libraries for the imu
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
//library for the brushless motors
#include <Servo.h>

//All distances will be in cm
//Starting distance: should be the distance from the sonar attached to the drone to the floor
#define DISTANCE_A 1
//Distance where the floor no longer affects the thrust of the motors. Should increase the throtttle accordingly to account for this
#define DISTANCE_B 1
//Peak distance where the drone should be hovering without an increase or decrease in altitude 
#define DISTANCE_C 1
//Distance where the drone should start decelerating to prepare for landing
#define DISTANCE_D 1
//Distance just a little bit higher than DISTANCE_A that is used for landing
#define DISTANCE_E 1

//Time for which the drone will hover for
#define HOVER_TIME 1

//Throttle to account for thrust from the floor
#define ASCEND_1 1
//Throttle to ascend
#define ASCEND_2 1
//Throttle to hover
#define HOVER 1
//Throttle to begin descending 
#define DESCEND_1 1
//Throttle to finish descending taking the thrust from the floor into account
#define DESCEND_2 1
//Throttle to turn motors off
#define LANDED 0
//Maximum throttle, used to initialize the esc's 
#define MAX 1023

//Pins for the trigger and echo of the sonar
#define TRIGGER_PIN 1
#define ECHO_PIN 2

//global variable for the distance that the sonar reads. Volitile so it can be used in the interrupt
volatile long distance;

/*Global variable to determine and change the state of the drone
 * 0: Starting state, will configure the esc's and test motors
 * 1: Begin ascent state, will begin to ascend with a throttle that takes the thrust from the floor into account
 * -Distance should be greater or equal to A and less than B
 * 2: Ascent state, will continue acsending, but will now change the throttle so that it not longer account for floor
 * -Distance should be greater or equal to B and less than C
 * 3: Hover state, will reach the final distance ad hover the drone for a set amount of time
 * -Distance should be greater or equal to C
 * 4: Begin descent state, will now begin descinding the drone with a throttle slightly lower than the hover throttle
 * -Distance should be less than C and greater or equal to D
 * 5: Slow descent state, will slow the descent in order to begine preparing to land. Should take thrust from floor into account
 * -Distance should be less than or equal D and greater than A
 * 6: Landing state, should be landed at this point. Turn motors off
 */
volatile int currentState;

//Global variables for the IMU and PID
volatile float xAxis;
volatile float yAxis;
volatile float zAxis;
volatile float xAccel;
volatile float yAccel;
volatile float zAccel;

//Code for ESC's and Motors
int motorPins[] = {};
int throttle = 0;
int startup = 5000;

//initializing the 4 Servo instances
Servo motor[4];

//Sonar instance for the SR04 class
SR04 sonar = SR04(ECHO_PIN,TRIGGER_PIN);

//IMU instance for the BNO055 class
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);

  //defining the interrupt routine
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), changeState, CHANGE);
  currentState = 0;

  //attaching the Servo instances to the pins of the motors
  for(int i = 0; i < 4; i++){
    motor[i].attach(motorPins[i]);
  }
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  //firt it will set up the ESC's, then the functionality of the state diagram will occur
  //Currently set to 7 so that it will not reach this code
  if(currentState == 7){
  //if less than 15 seconds have passed
   if(millis()-startup<15000){
    throttle = MAX;
    throttle = map(throttle,0,MAX,1000,2000);
    for(int i = 0; i < 4; i++){
      motor[i].writeMicroseconds(throttle);
    }
    Serial.println("HIGH");
    }else{
      throttle = 0;
      throttle = map(throttle,0,MAX,1000,2000);
      for(int i = 0; i < 4; i++){
        motor[i].writeMicroseconds(throttle);
      }
      Serial.println("LOW");
    }
    delay(5);

    if(millis()-startup>30000){
      currentState == 1;
    }
  }else{

    //checks before the sonar reads in order to prevent the state transition interrupt
    if(currentState == 3){
      //Code to have the drone hover for set amount of time
      delay(HOVER_TIME);
    
      currentState = 4;
    }

    //Getting the axis orientation from the IMU
    sensors_event_t event; 
    bno.getEvent(&event);

    xAxis = event.orientation.x;
    yAxis = event.orientation.y;
    zAxis = event.orientation.z;
  
  
    //Getting the distance from the sonar
    distance = sonar.Distance();
    delay(50);
  }
  
}

void changeState() {
  
  switch(currentState)
  {
//    case 0:
//      //Code for initializing esc's
//
//      currentState = 1;
//      break;

    case 1:
      if(distance >= DISTANCE_A && distance < DISTANCE_B)
      {
        //Code for setting throttle to account for increased thrust from the floor

        throttle = ASCEND_1;
        
        for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);
    
      }
      else if(distance >= DISTANCE_B)
      {
        currentState = 2;
      }
      
      break;
    case 2:
      if(distance >= DISTANCE_B && distance < DISTANCE_C)
      {
        //Code for setting throttle to account for increased thrust from the floor

        throttle = ASCEND_2;
        
        for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);
          
      }
      else if(distance >= DISTANCE_C)
      {
        currentState = 3;
      }
      
      break;
    case 3:
      if(distance >= DISTANCE_C)
      {
        //Code for setting the throttle to hover the drone

        throttle = HOVER;
        
        for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);

      }
      
      break;
    case 4:
      if(distance >= DISTANCE_D && distance < DISTANCE_C)
      {
        //Code for setting throttle to begin descinding 

        throttle = DESCEND_1;
        
        for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);
          
      }
      else if(distance <= DISTANCE_D)
      {
        currentState = 5;
      }
      
      break;
    case 5:
      if(distance > DISTANCE_A && distance <= DISTANCE_D)
      {
        //Code for setting throttle to begin landing. Will have to take the thrust from the floor into account here as well

        throttle = DESCEND_2;
        
        for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);
          
      }
      else if(distance < DISTANCE_E)
      {
        currentState = 6;
      }
      break;
      
    case 6:
        //Code for when the drone has landed

        throttle = LANDED;
        
        for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);
          
      break;
  }
}

void PID() {
  
  float pitch = 0;
  float roll = 0;
  float yaw = 0;

  pitch = 180 * atan(xAccel/sqrt(yAccel*yAccel + zAccel * zAccel))/3.14159265358979323846;
  yaw =  180 * atan(zAccel/sqrt(xAccel*xAccel + yAccel * yAccel))/3.14159265358979323846;
  roll = 180 * atan(yAccel/sqrt(xAccel*xAccel + zAccel * zAccel))/3.14159265358979323846;
  
}

float correctSonar(int dist) {
  int Maximum = 0;
  int Minimum = 0;

  int returnVal = 0;

  if(distance < Maximum && dist > minimum) {
   return distance; 
    
  }

  return returnVal;
  
}
