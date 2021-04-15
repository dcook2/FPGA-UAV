#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>


float gyro_avg[3];


Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event, e;

//EulerAngle Struct
struct EulerAngles{
  double roll, pitch, yaw;
};


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  //attachInterrupt(digitalPinToInterrupt(19), kill, CHANGE);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);

  int count = 0;
//  for(count = 0; count < 2000; count++){
//    bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
//    gyro_avg[0] += event.gyro.x;
//    gyro_avg[1] += event.gyro.y;
//    gyro_avg[2] += event.gyro.z;
//    delay(3);
//    Serial.print("x:");
//    Serial.print(event.gyro.x);
//    
//    Serial.print("\ty:");
//    Serial.print(event.gyro.y);
//    
//    Serial.print("\tz:");
//    Serial.println(event.gyro.z);
//  }
//  for(int i = 0; i < 3; i++) gyro_avg[i] /= count;
//    Serial.print("avgx:");
//    Serial.print(gyro_avg[0]);
//    Serial.print("\tavgy:");
//    Serial.print(gyro_avg[1]);
//    Serial.print("\tavgz:");
//    Serial.println(gyro_avg[2]);
//
//    delay(5000);


}

void loop() {
  // put your main code here, to run repeatedly:

  bno.getEvent(&event);
  imu::Quaternion quat = bno.getQuat();
  
  Serial.print(F("Quaternion: "));
  Serial.print((float)quat.w());
  Serial.print(F(", "));
  Serial.print((float)quat.x());
  Serial.print(F(", "));
  Serial.print((float)quat.y());
  Serial.print(F(", "));
  Serial.print((float)quat.z());
  Serial.print(F("\t"));

  imu::Vector<3> eulerangles = quat.toEuler();

  //Euler
  //EulerAngles angles = ToEulerAngles(quat);
  
  Serial.print(F("Euler: "));
  Serial.print((float)eulerangles[2]*RAD_TO_DEG);
  Serial.print(F(", "));
  Serial.print((float)eulerangles[1]*RAD_TO_DEG);
  Serial.print(F(", "));
  Serial.print((float)eulerangles[0]*RAD_TO_DEG);
  Serial.println(F(""));
}

void kill(){
  Serial.println("kill");
}

// Derived from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
EulerAngles ToEulerAngles(imu::Quaternion q){
  EulerAngles angles;

  //roll
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.y() - q.z() * q.y());
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  //pitch
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if(abs(sinp) >= 1){
    angles.pitch = copysign(PI / 2, sinp);
  }else angles.pitch = asin(sinp);

  //yaw
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}
