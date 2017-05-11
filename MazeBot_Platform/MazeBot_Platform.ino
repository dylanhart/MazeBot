#include <Wire.h>
#include <motordriver_4wd.h>
#include <seeed_pwm.h>

#define DIR_FWD 0
#define DIR_REV 1

int bumpPin = 11;
bool isdead = false;
int leftSpeed = 0, rightSpeed = 0;
byte leftDir = 0, rightDir = 0;

void setup() {
  Serial.begin(9600);
  MOTOR.init();

  pinMode(bumpPin, INPUT);

  Wire.begin(11);
  Wire.onReceive(controlEvent);
  Wire.onRequest(statusEvent);
}

void controlEvent(int n) {
  while (Wire.available() < 6);
  leftSpeed = (Wire.read() << 8) | Wire.read();
  leftDir = Wire.read();
  rightSpeed = (Wire.read() << 8) | Wire.read();
  rightDir = Wire.read();
}

void statusEvent() {
  Wire.write(isdead);
}

void loop() {
  if (digitalRead(bumpPin)) {
    kill();
  }
  
  if (isdead || leftSpeed == 0)
    MOTOR.setStop1();
  else
    MOTOR.setSpeedDir1(leftSpeed, leftDir);

  if (isdead || rightSpeed == 0)
    MOTOR.setStop2();
  else
    MOTOR.setSpeedDir2(rightSpeed, !rightDir);

  delay(20);
}

void kill() {
  if (leftDir == DIR_FWD)
    MOTOR.setStop1();
  if (rightDir == DIR_FWD)
    MOTOR.setStop2();
//  isdead = true;
  Serial.write("i am ded");
}

