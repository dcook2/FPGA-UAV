#include <Wire.h>

const int INITIALIZING = 0,
          IDLEING = 1,
          TAKEOFF = 2,
          FLYING = 3,
          LANDING = 4;

int STATE = 0;

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  while(1){
    switch(STATE){
      case INITIALIZING: initialize();
        break;
      case IDLEING: idle();
        break;
      case TAKEOFF: takeoff();
        break;
      case FLYING: fly();
        break;
      case LANDING: land();
        break;
    }
  }

}

float limit(float num, float low, float high){
  if (num > high) {
    num = high;
  }else if (num < low){
    num = low;
  }
  return num;
}

void initialize(){
  
}

void idle(){
  
}

void takeoff(){
  
}

void fly(){
  
}

void land(){
  
}

class PID{

  float goal, kp, ki, kd;
  float integral = 0;
  float previous_error = 0;
  
  
  public:
    void pidinit(float goal, float kp, float ki, float kd);
    float output(float measurement);

};

  void PID::pidinit(float goal,float kp,float ki,float kd){
    this->goal = goal;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }

  float PID::output(float measurement){
    float error = goal - measurement;
    integral = integral + error;
    float derivative = error - previous_error;
    float output = kp*error + ki*integral + kd*derivative;
    previous_error = error;
    return output;
  }
