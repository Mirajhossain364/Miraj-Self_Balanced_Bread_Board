#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

#define xservopin 3
#define yservopin 5

Servo xservo;
Servo yservo;

MPU6050 mpu(Wire);

float kp = 0.5;
float kd = 1000;
float ki = 0.0000001;

unsigned long lastTime;
int dt;

float xTarget = 0, xError = 0, xErrorOld = 0, xErrorChange = 0, xErrorSlope = 0, xErrorArea = 0, xServoPos = 80;
float yTarget = 0, yError = 0, yErrorOld = 0, yErrorChange = 0, yErrorSlope = 0, yErrorArea = 0, yServoPos = 80;

float xCorrection, yCorrection;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  xservo.attach(xservopin);
  yservo.attach(yservopin);

  while (mpu.begin() != 0) { }
  
  mpu.calcOffsets();

  xservo.write(80);
  yservo.write(80);
}

void loop() {
  mpu.update();

  float xActual = mpu.getAngleX();
  float yActual = mpu.getAngleY();

  dt = millis() - lastTime;
  lastTime = millis();

  xErrorOld = xError;
  xError = xActual - xTarget;
  xErrorChange = xError - xErrorOld;
  xErrorSlope = xErrorChange / dt;
  xErrorArea += xError * dt;

  yErrorOld = yError;
  yError = yActual - yTarget;
  yErrorChange = yError - yErrorOld;
  yErrorSlope = yErrorChange / dt;
  yErrorArea += yError * dt;

  xCorrection = kp * xError + kd * xErrorSlope + ki * xErrorArea;
  yCorrection = kp * yError + kd * yErrorSlope + ki * yErrorArea;

  xServoPos += xCorrection;
  yServoPos += yCorrection;

  xservo.write(xServoPos);
  yservo.write(yServoPos);

  Serial.print(xActual);
  Serial.print("||");
  Serial.println(xServoPos);

  Serial.print(yActual);
  Serial.print("||");
  Serial.println(yServoPos);
  
  delay(200);
}
