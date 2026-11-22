#pragma once
// Stand-in for pigpio's client header so servo.cpp builds in the unit test
// harness without pigpio installed. servo_test.cpp defines the function.
extern "C" int set_servo_pulsewidth(int pi, unsigned user_gpio, unsigned pulsewidth);
