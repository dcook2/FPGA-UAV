//Library for the sonar
#include "SR04.h"
//Libraries for the imu
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
//Library for the brushless motors
#include <Servo.h>

//Starting distance: should be the distance from the sonar attached to the drone to the floor
#define DISTANCE_A 7
//Distance where the floor no longer affects the thrust of the motors. Should increase the throttle accordingly to account for this
#define DISTANCE_B 30
//Peak distance where the drone should be hovering without an increase or decrease in altitude 
#define DISTANCE_C 100
//Distance where the drone should start decelerating to prepare for landing
#define DISTANCE_D 30
//Distance just a little bit higher than DISTANCE_A that is used for landing
#define DISTANCE_E 9

//Time for which the drone will hover for
#define HOVER_TIME 10000

//Throttle to account for thrust from the floor
#define ASCEND_1 1200
//Throttle to ascend
#define ASCEND_2 1400
//Throttle to hover
#define HOVER 1150
//Throttle to begin descending 
#define DESCEND_1 1100
//Throttle to finish descending taking the thrust from the floor into account
#define DESCEND_2 1050
//Throttle to turn motors off
#define LANDED 1000
//Maximum throttle, used to initialize the esc's 
#define MAX 1023

//Pins for the trigger and echo of the sonar
#define TRIGGER_PIN 12
#define ECHO_PIN 2

//Pins for the four different motors
#define FRONT_RIGHT 3
#define FRONT_LEFT 4
#define BACK_RIGHT 5
#define BACK_LEFT 6

//Killswitch
#define BUTTON 19



//Global variable for the distance that the sonar reads. Volatile so it can be used in the interrupt
long distance;
long previousDistance;

/*Global variable to determine and change the state of the drone
 * 
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
float roll_p_gain = 1.3;
float roll_i_gain = 0.04;
float roll_d_gain = 18.0;
int roll_max = 400; 

float pitch_p_gain = roll_p_gain;
float pitch_i_gain = roll_i_gain;
float pitch_d_gain = roll_d_gain;
int pitch_max = roll_max;

float yaw_p_gain = 4.0;
float yaw_i_gain = 0.02;
float yaw_d_gain = 0.0;
int yaw_max = 400;

//Enables and disables the PID loop
boolean auto_level = true;

float temp_error;
float roll_input, roll_setpoint, roll_i, roll_d, roll_output;
float pitch_input, pitch_setpoint, pitch_i, pitch_d, pitch_output;
float yaw_input, yaw_setpoint, yaw_i, yaw_d, yaw_output;


//Code for ESC's and Motors
int motorPins[] = {FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT};
int throttle = 0;
int startup = 5000;
int FR_THROTTLE, BR_THROTTLE, FL_THROTTLE, BL_THROTTLE;


//Initializing the 4 Servo instances
Servo motor[4];

//Sonar instance for the SR04 class
SR04 sonar = SR04(ECHO_PIN,TRIGGER_PIN);

//IMU instance for the BNO055 class
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  
  Serial.begin(9600);

  //defining the interrupt routine
  //attachInterrupt(digitalPinToInterrupt(ECHO_PIN), changeState, CHANGE);

  attachInterrupt(digitalPinToInterrupt(BUTTON), kill, CHANGE);
  
  currentState = 1;

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
  
  //Firt it will set up the ESC's, then the functionality of the state diagram will occur
  if(currentState == 0){
    
    if(millis()-startup<15000){
      
      throttle = MAX;
      throttle = map(throttle,0,MAX,1000,2000);
    
      for(int i = 0; i < 4; i++){
        motor[i].writeMicroseconds(throttle);
      }
    
      Serial.println("HIGH");
    
    }
    else {
      
      throttle = 0;
      throttle = map(throttle,0,MAX,1000,2000);
      
      for(int i = 0; i < 4; i++){
        motor[i].writeMicroseconds(throttle);
      }
      
      Serial.println("LOW");
      
    }
    
    delay(5);

    if(millis()-startup>30000)
      currentState = 1;
    
  }
  else {

    //Checks before the sonar reads in order to prevent the state transition interrupt
    if(currentState == 3){
      
      //Code to have the drone hover for set amount of time
      delay(HOVER_TIME);
    
      currentState = 4;
      
    }


    //Getting the distance from the sonar
    distance = correctSonar();
//    Serial.print("distance ");
//    Serial.print(distance);
//    Serial.print("    currentState ");
//    Serial.print(currentState);
    Serial.print("    throttle ");
    Serial.print(throttle);
    Serial.print("    FR throttle ");
    Serial.print(FR_THROTTLE);
    Serial.print("    FL throttle ");
    Serial.print(FL_THROTTLE);
    Serial.print("    BR throttle ");
    Serial.print(BR_THROTTLE);
    Serial.print("    BL throttle ");
    Serial.println(BL_THROTTLE);
    delay(50);

    //S
    switch(currentState)
    {
      case 1:
        if(distance >= DISTANCE_A && distance < DISTANCE_B)
        {
                     
          //Code for setting throttle to account for increased thrust from the floor
          throttle = ASCEND_1;
          
          if(millis() - startup < 15000)
            throttle = 1000; 
           
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
    
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
        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
          
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
        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);

        }
      
      break;
      
      case 4:
        if(distance >= DISTANCE_D && distance < DISTANCE_C)
        {
        
          //Code for setting throttle to begin descinding 
          throttle = DESCEND_1;
        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
          
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
        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
          
        }
        else if(distance < DISTANCE_E)
        {
          currentState = 6;
        }
      break;
      
      case 6:
          //Code for when the drone has landed
          throttle = LANDED;
        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
          
      break;
      
    }

    if(auto_level){
    
      //reseting pid oontrollers
      roll_i = 0;
      roll_d = 0;
      pitch_i = 0;
      pitch_d = 0;
      yaw_i = 0;
      yaw_d = 0;
    
      //Calling the PID loop for auto-correction
      PID();

      if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
      FR_THROTTLE = throttle - pitch_output + roll_output - yaw_output; //Calculate the pulse for esc 1 (front-right - CCW)
      BR_THROTTLE = throttle + pitch_output + roll_output + yaw_output; //Calculate the pulse for esc 2 (rear-right - CW)
      BL_THROTTLE = throttle + pitch_output - roll_output - yaw_output; //Calculate the pulse for esc 3 (rear-left - CCW)
      FL_THROTTLE = throttle - pitch_output - roll_output + yaw_output; //Calculate the pulse for esc 4 (front-left - CW)

      if(millis() - startup < 15000){
            throttle = 1000; 
            for(int i = 0; i < 4; i++)
          motor[i].writeMicroseconds(throttle);   
      }
      else{
      motor[0].writeMicroseconds(FR_THROTTLE);
      motor[1].writeMicroseconds(FL_THROTTLE);
      motor[2].writeMicroseconds(BR_THROTTLE);
      motor[3].writeMicroseconds(BL_THROTTLE);
      }
    }
    else{
      for(int i = 0; i < 4; i++)
        motor[i].writeMicroseconds(throttle);   
    }
  }
  
}

//Currently not in use
//void changeState() {
//  
//  switch(currentState)
//  {
//    case 1:
//      if(distance >= DISTANCE_A && distance < DISTANCE_B)
//      {
//        
//        //Code for setting throttle to account for increased thrust from the floor
//        throttle = ASCEND_1;
//        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
//    
//      }
//      else if(distance >= DISTANCE_B)
//      {
//        currentState = 2;
//      }
//      
//      break;
//      
//    case 2:
//      if(distance >= DISTANCE_B && distance < DISTANCE_C)
//      {
//        
//        //Code for setting throttle to account for increased thrust from the floor
//        throttle = ASCEND_2;
//        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
//          
//      }
//      else if(distance >= DISTANCE_C)
//      {
//        currentState = 3;
//      }
//      
//      break;
//      
//    case 3:
//      if(distance >= DISTANCE_C)
//      {
//        
//        //Code for setting the throttle to hover the drone
//        throttle = HOVER;
//        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
//
//      }
//      
//      break;
//      
//    case 4:
//      if(distance >= DISTANCE_D && distance < DISTANCE_C)
//      {
//        
//        //Code for setting throttle to begin descinding 
//        throttle = DESCEND_1;
//        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
//          
//      }
//      else if(distance <= DISTANCE_D)
//      {
//        currentState = 5;
//      }
//      
//      break;
//      
//    case 5:
//      if(distance > DISTANCE_A && distance <= DISTANCE_D)
//      {
//        
//        //Code for setting throttle to begin landing. Will have to take the thrust from the floor into account here as well
//        throttle = DESCEND_2;
//        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
//          
//      }
//      else if(distance < DISTANCE_E)
//      {
//        currentState = 6;
//      }
//      break;
//      
//    case 6:
//        //Code for when the drone has landed
//        throttle = LANDED;
//        
//        for(int i = 0; i < 4; i++)
//          motor[i].writeMicroseconds(throttle);
//          
//      break;
//      
//  }
//  
//}


//Function for the PID loop that corrects the orientation of the drone
void PID() {
  
  //Getting the axis orientation from the IMU
  sensors_event_t event; 
  bno.getEvent(&event);

  roll_input = event.orientation.x;
  pitch_input = event.orientation.y;
  yaw_input = event.orientation.z;

  //Roll PID calculation
  temp_error = roll_input - roll_setpoint;
  roll_i += roll_i_gain* temp_error;
  if (roll_i > roll_max)
    roll_i = roll_max;
  else if(roll_i < roll_max * -1)
    roll_output = roll_max * -1;

  roll_output = roll_p_gain * temp_error + roll_i + roll_d_gain * (temp_error - roll_d);
  if(roll_output > roll_max)
    roll_output = roll_max;
  else if (roll_output < roll_max * -1)
    roll_output = roll_max * -1;

  roll_d = temp_error;

  //Pitch PID calculation
  temp_error = pitch_input - pitch_setpoint;
  pitch_i += pitch_i_gain* temp_error;
  if (pitch_i> pitch_max)
    pitch_i = pitch_max;
  else if(pitch_i < pitch_max * -1)
    pitch_output = pitch_max * -1;

  pitch_output = pitch_p_gain * temp_error + pitch_i + pitch_d_gain * (temp_error - pitch_d);
  if(pitch_output > pitch_max)
    pitch_output = pitch_max;
  else if (pitch_output < pitch_max * -1)
    pitch_output = pitch_max * -1;

  pitch_d = temp_error;
   
  //Yaw PID calculation
  temp_error = yaw_input - yaw_setpoint;
  yaw_i += yaw_i_gain* temp_error;
  if (yaw_i> yaw_max)
    yaw_i = yaw_max;
  else if(yaw_i < yaw_max * -1)
    yaw_output = yaw_max * -1;

  yaw_output = yaw_p_gain * temp_error + yaw_i + yaw_d_gain * (temp_error - yaw_d);
  if(yaw_output > yaw_max)
    yaw_output = yaw_max;
  else if (yaw_output < yaw_max * -1)
    yaw_output = yaw_max * -1;

  yaw_d = temp_error;
  
  
}

//Function to ensure that no 'trash' values are passed to the interrupt
float correctSonar() {
  
  int Maximum = 400;
  int Minimum = 6; 

  int dist = sonar.Distance();
  
  if(dist < Maximum && dist > Minimum) {
   previousDistance = dist;  
   return dist;   
  }

  return previousDistance;
}

void kill() {
  currentState = 6;
}
