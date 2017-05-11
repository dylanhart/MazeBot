#pragma once

class Ultrasonic {
  int pin;
public:
  Ultrasonic(int pin) : pin(pin) {}
  
  unsigned long ping() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin,LOW);
    pinMode(pin,INPUT);
    return pulseIn(pin,HIGH);
  }
};

