/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/



#ifndef __MY_TASK_FUNCTION_HPP__
#define __MY_TASK_FUNCTION_HPP__

#include "my_global.hpp"
#include <algorithm>


void thread_callback_test();
void camera_test();

void show_all_of_the_component_without_ips();
void send_picture_to_Serve();
void tracking();
void get_image_datasets();
void send_img_infor();
#endif // __MY_TASK_FUNCTION_HPP__
