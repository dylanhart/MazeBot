#pragma once

#include <math.h>
#include <Wire.h>
#include "Encoder.h"

#define DIR_FWD 0
#define DIR_REV 1
#define DIR_LEFT 0
#define DIR_RIGHT 1

#define TICKS_PER_REV 72
#define UNITS_PER_TICK (3.14*7.5 / TICKS_PER_REV)
#define PLATFORM_WIDTH 17.0

double normalize_angle(double theta) {
    while (theta > 3.1415) theta -= 3.1415*2;
    while (theta <= -3.1415) theta += 3.1415*2;
    return theta;
}

class Platform {
  byte addr;
  bool ldir = DIR_FWD;
  bool rdir = DIR_FWD;
  double err_theta = 0;
  double sum_err_theta = 0;
  double err_dist = 0;
  double sum_err_dist = 0;
  int turn_time = 0;
public:
  double x = 0, y = 0, theta = 3.1415/2;
  double goal_x = 0, goal_y = 0;
  
  Platform(byte addr, int left_enc, int right_enc)
    : addr(addr) {
    initEncoders(left_enc, right_enc);
  }

  void updateBotPos() {
    int left = ldir == DIR_FWD ? left_ticks : -left_ticks;
    int right = rdir == DIR_FWD ? right_ticks : -right_ticks;
    left_ticks = right_ticks = 0;

    double v_avg = (left + right) * UNITS_PER_TICK / 2.0;
    x += v_avg * cos(theta);
    y += v_avg * sin(theta);
    theta += (right - left) * UNITS_PER_TICK / PLATFORM_WIDTH;
    theta = normalize_angle(theta);
  }

  double distToGoal() {
    return hypot(goal_y - y, goal_x - x);
  }
  
  void setSpeed(int left_spd, byte left_dir, int right_spd, byte right_dir) {
    ldir = left_dir;
    rdir = right_dir;
    Wire.beginTransmission(this->addr);
    Wire.write(left_spd >> 8);
    Wire.write(left_spd & 0xff);
    Wire.write(left_dir);
    Wire.write(right_spd >> 8);
    Wire.write(right_spd & 0xff);
    Wire.write(right_dir);
    Wire.endTransmission();
  }

  void stop() {
    setSpeed(0,0,0,0);
  }

  void setGoalPos(double x, double y) {
    goal_x = x;
    goal_y = y;
    err_theta = 0;
    sum_err_theta = 0;
    err_dist = 0;
    sum_err_dist = 0;
  }

//  void update(int max_speed) {
//    updateBotPos();
//
//
//    // p
//    double dist = distToGoal();
//    double angle_offset = normalize_angle(atan2(goal_y - y, goal_x - x) - theta);
//
//    // i
//    sum_err_dist += dist;
//    sum_err_theta += angle_offset;
//
//    // d
//    double delta_err_dist = dist - err_dist;
//    double delta_err_theta = angle_offset - err_theta;
//    
//    if (abs(angle_offset) > .05 && dist > 15) {
//      // rotate
//      const double k_p = 3.0;
//      const double k_i = 0.4;
//      const double k_d = 0.2;
//      
//      double wheel_speed = k_p*angle_offset + k_i*sum_err_theta + k_d*delta_err_theta;
//      wheel_speed = min(max_speed, wheel_speed);
//      setSpeed(wheel_speed, angle_offset > 0, wheel_speed,  angle_offset <= 0);
//    } else {
//      // move fwd
//      const double k_p = 0.5;
//      const double k_i = 0.01;
//      const double k_d = 0.0;
//
//      const double k_angle = 150;
//      double turn_speed = k_angle*angle_offset;
//
//      double wheel_speed = k_p*dist + k_i*sum_err_dist + k_d*delta_err_dist;
//      wheel_speed = min(max_speed, wheel_speed);
//      if (abs(turn_speed) > wheel_speed) {
//        if (turn_speed > 0)
//          setSpeed(0, DIR_FWD, wheel_speed+turn_speed, DIR_FWD);
//        else
//          setSpeed(wheel_speed-turn_speed, DIR_FWD, 0, DIR_FWD);
//      } else {
//        setSpeed(wheel_speed-turn_speed, DIR_FWD, wheel_speed+turn_speed, DIR_FWD);
//      }
//    }
//
//    err_dist = dist;
//    err_theta = angle_offset;
//  }

//  void update(int max_speed) {
//    updateBotPos();
//
//    double dist = distToGoal();
//    double angle_offset = normalize_angle(atan2(goal_y - y, goal_x - x) - theta);
//
//    const double k_dist = 1;
//    const double k_angle = .1;
//
//    double speed = min(k_dist * dist, 1);
//    double gamma = k_angle * angle_offset;
//
//    double v_left = speed - gamma * PLATFORM_WIDTH / 2;
//    double v_right = speed + gamma * PLATFORM_WIDTH / 2;
//
//    setSpeed(
//      abs(v_left * max_speed), v_left < 0,
//      abs(v_right * max_speed), v_right < 0
//    );
//  }

  void driveTicks(int ticks, int speed, byte dir) {
    updateBotPos();
    setSpeed(speed, dir, speed, dir);
    left_ticks = right_ticks = 0;
    while ((left_ticks + right_ticks)/2 < ticks);
    stop();
  }

  void turnTicks(int ticks, int speed, byte dir) {
    updateBotPos();
    setSpeed(speed, !dir, speed, dir);
    left_ticks = right_ticks = 0;
    while ((left_ticks + right_ticks)/2 < ticks);
    stop();
  }

  byte getStatus() {
    Wire.requestFrom(11, 1, false);
    while (!Wire.available());
    return Wire.read();
  }
};


