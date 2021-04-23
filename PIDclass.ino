void setup(){}
void loop(){}

class PID{
  private:
    float kp, ki, kd;
    float target, measured, error;
    float errorOld, errorChange, errorSlope, errorArea;
    long oldTime, newTime, changeTime;

  PID(float target, float kp, float ki, float kd){
    this->target = target;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->error = 0;
    this->errorSlope = 0;
    this->errorArea = 0;
    newTime = micros();
  }

  float getOutput(float measured){
    oldTime = newTime;
    newTime = micros();
    changeTime = newTime - oldTime;

    this->measured = measured;
    errorOld = error;
    error = target - measured;
    errorChange = error-errorOld;
    errorSlope = errorChange/changeTime;
    errorArea += error*changeTime;

    return kp*error + ki*errorArea + kd*errorSlope;
  }
};
