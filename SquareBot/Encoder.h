#pragma once

volatile int left_ticks = 0, right_ticks = 0;

void leftEncIsr() {
  left_ticks++;
}

void rightEncIsr() {
  right_ticks++;
}

void initEncoders(int left_encoder, int right_encoder) {
  attachInterrupt(digitalPinToInterrupt(left_encoder), leftEncIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder), rightEncIsr, CHANGE);
}

void logEncoders() {
  Serial.print(left_ticks, DEC);
  Serial.write(", ");
  Serial.println(right_ticks, DEC);
}


