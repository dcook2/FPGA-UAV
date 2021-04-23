#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

Adafruit_BNO055 bno = Adafruit_BNO055();

uint8_t calibData;
uint8_t ssystem, gyro, accel, mg = 0;

int eeadr = 0;
boolean first = true;

int MODE = 1; //0 - write cal data 1 - read cal data



void setup() {
  Serial.begin(115200);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);

  delay(1200);



}

void loop() {
  switch(MODE){
    case 0: if(bno.getSensorOffsets(&calibData)){
              EEPROM.write(eeadr,calibData);
              Serial.print("Storing ");
              Serial.print(calibData);
              Serial.print(" to EEPROM.");
              Serial.println(calibData);
            }else{
              bno.getCalibration(&ssystem, &gyro, &accel, &mg);
              Serial.print(ssystem);
              Serial.print(",");
              Serial.print(gyro);
              Serial.print(",");
              Serial.print(accel);
              Serial.print(",");
              Serial.println(mg);
            }
            break;
    case 1: calibData = EEPROM.read(0); 
            if(first){
              bno.setSensorOffsets(&calibData);
              first = false;
            }
            if(bno.getSensorOffsets(&calibData)){
              //EEPROM.write(eeadr,calibData);
              Serial.print("calibData: ");
              //Serial.print(calibData);
              //Serial.print(" to EEPROM.");
              Serial.println(calibData);
            }else{
              bno.getCalibration(&ssystem, &gyro, &accel, &mg);
              Serial.print(ssystem);
              Serial.print(",");
              Serial.print(gyro);
              Serial.print(",");
              Serial.print(accel);
              Serial.print(",");
              Serial.println(mg);
            }
            break;
    default: Serial.println("dumbass");
  }
  
  delay(100);
}
