#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

MyRobot::MyRobot(float w) {
  _startTime = 0;
  _endTime = 0;
  _w = w;
}

/**
* @brief Robot moves forward specified distance at specified speed
* 
* @param distance in meters
* @param speed in meters per second, default 0.2 m/s
* @param wait_time time in seconds to wait after moving, default 0 s
**/
void MyRobot::forward(float distance, float speed = base_speed, float wait_time = 0) {
  _motors.setSpeeds(single_to_milli * speed * left_wheel_adjust, single_to_milli * speed * right_wheel_adjust);
  wait(distance / speed);
  halt(wait_time);
}

/**
* @brief Robot moves backward specified distance at specified speed
* 
* @param distance in meters
* @param speed in meters per second, default 0.2 m/s
* @param wait_time time in seconds to wait after moving, default 0 s
**/
void MyRobot::backward(float distance, float speed = base_speed, float wait_time = 0) {
  _motors.setSpeeds(-speed * single_to_milli, -speed * single_to_milli);
  wait(distance / speed);
  halt(wait_time);
}

/**
* @brief Robot turns left for specified distance at specified speed
* 
* @param duration in seconds
* @param speed in meters per second, default 0.2 m/s
* @param wait_time time in seconds to wait after moving, default 0 s
**/
void MyRobot::turn_left(float duration, float speed = base_speed, float wait_time = 0) {
  _motors.setSpeeds(-speed * single_to_milli, speed * single_to_milli);
  wait(duration);
  halt(wait_time);
}

/**
* @brief Robot turns right for specified distance at specified speed
* 
* @param dduration in seconds
* @param speed in meters per second, default 0.2 m/s
* @param wait_time time in seconds to wait after moving, default 0 s
**/
void MyRobot::turn_right(float duration, float speed = base_speed, float wait_time = 0) {
  _motors.setSpeeds(speed * single_to_milli, -speed * single_to_milli);
  wait(duration);
  halt(wait_time);
}

/**
* @brief Robot stops moving
*
* @param duration in seconds
**/
void MyRobot::halt(float duration) {
  _motors.setSpeeds(0, 0);
  wait(duration);
}

/**
* @brief Robot moves forward and right simultaneously for specified duration at specified speed
* 
* @param duration in seconds
* @param speed in meters per second, default 0.2 m/s
* @param wait_time time in seconds to wait after moving, default 0 s
**/
void MyRobot::forward_right(float duration, float speed = base_speed, float wait_time = 0) {
  _motors.setSpeeds(speed * single_to_milli, 0.5 * speed * single_to_milli);
  wait(duration);
  halt(wait_time);
}

/**
* @brief Robot moves forward and left simultaneously for specified duration at specified speed
* 
* @param duration in seconds
* @param speed in meters per second, default 0.2 m/s
* @param wait_time time in seconds to wait after moving, default 0 s
**/
void MyRobot::forward_left(float duration, float speed = base_speed, float wait_time = 0) {
  _motors.setSpeeds(0.5 * speed * single_to_milli, speed * single_to_milli);
  wait(duration);
  halt(wait_time);
}

/**
* @brief Do nothing for specified time
* 
* @param duration time to wait in seconds
**/
void MyRobot::wait(float duration) {
  _startTime = millis();
  _endTime = (unsigned long) single_to_milli * duration + _startTime;
  while (millis() < _endTime) {
    // do nothing
  }
}

/*
* Calculate time to turn given radians at given speed in m/s.
*/
float MyRobot::calculate_turn_time(float radians, float speed) {
  return abs(radians * _w / (2 * speed * 100));
}

/*
* Turn robot to the left specified radians at specified speed (m/s) with wait_time in s.
*/
void MyRobot::turn_left_rads(float radians, float speed, float wait_time) {
  turn_left(calculate_turn_time(radians, speed), speed, wait_time);
}

/*
* Turn robot to the right specified radians at specified speed (m/s) with wait_time in s.
*/
void MyRobot::turn_right_rads(float radians, float speed, float wait_time) {
  turn_right(calculate_turn_time(radians, speed), speed, wait_time);
}