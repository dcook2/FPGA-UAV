#include <Servo.h>

Servo motor[4];
int throttle=0;
int startup = 5000;
bool first = false;

int motorPin[] = {3,4,5,6};
int potPin = A0;

void setup() {
  // put your setup code here, to run once:

  for(int i = 0; i < 4; i++){
    motor[i].attach(motorPin[i]);
  }

  startup = millis();

  Serial.begin(115200);
 

}

void loop() {
  // put your main code here, to run repeatedly:
if(first){
  
  if(millis()-startup<15000){
    throttle = 1023;
    throttle = map(throttle,0,1023,1000,2000);
    for(int i = 0; i < 4; i++){
      motor[i].writeMicroseconds(throttle);
    }
    Serial.println("HIGH");
    
  }else{
    throttle = 0;
    throttle = map(throttle,0,1023,1000,2000);
    for(int i = 0; i < 4; i++){
      motor[i].writeMicroseconds(throttle);
    }
    Serial.println("LOW");
  }
  delay(5);

  if(millis()-startup>30000){
    first=false;
  }
}else{

    throttle = analogRead(potPin);
    throttle = map(throttle,0,1023,0,20);
    throttle = 1000 + throttle*50;
    for(int i = 0; i < 4; i++){
      motor[i].writeMicroseconds(throttle);
    }
    delay(3);
   /* throttle++;
    if(throttle>30){
      throttle=0;
    }
    */
    //Serial.println("TESTING");
}

Serial.println(throttle);
//Serial.println(millis()-startup);


}
