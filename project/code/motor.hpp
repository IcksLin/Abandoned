/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/


#ifndef __MOTOR_H_HPP__
#define __MOTOR_H_HPP__

#include "zf_common_typedef.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_encoder.hpp"

extern zf_driver_pwm right_motor;
extern zf_driver_pwm left_motor;
extern zf_driver_gpio right_dir;
extern zf_driver_gpio left_dir;

bool motor_init();
void motor_set_speed(int16_t left_speed, int16_t right_speed);
void motor_stop();
bool get_and_remap_speed(float* right_speed,float*left_speed,int16 sampling_period);
#endif