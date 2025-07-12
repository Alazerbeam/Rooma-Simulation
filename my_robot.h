#ifndef my_robot_h
#define my_robot_h
#define single_to_milli 1000
#define base_speed 0.200          // in m/s
#define left_wheel_adjust 1       // values to manually adjust wheel strengths
#define right_wheel_adjust 1.015  // our robot's right wheel is slightly weaker
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class MyRobot{
  public:
    MyRobot(float w);
    void forward(float distance, float speed = base_speed, float wait_time = 0);
    void backward(float distance, float speed = base_speed, float wait_time = 0);
    void turn_left(float duration, float speed = base_speed, float wait_time = 0);
    void turn_right(float duration, float speed = base_speed, float wait_time = 0);
    void halt(float duration);
    void forward_right(float duration, float speed = base_speed, float wait_time = 0);
    void forward_left(float duration, float speed = base_speed, float wait_time = 0);
    float calculate_turn_time(float radians, float speed);
    void turn_left_rads(float radians, float speed, float wait_time = 0);
    void turn_right_rads(float radians, float speed, float wait_time = 0);
  private:
    unsigned long _startTime;
    unsigned long _endTime;
    float _w;
    Motors _motors;
    void wait(float duration);
};

#endif
