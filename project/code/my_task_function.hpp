/*********************************************************************************************************************
* 文件名称          my_task_function
* 功能说明          主循环任务函数声明
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-05-01        Assistant                  补充 Doxygen 注释
********************************************************************************************************************/



#ifndef __MY_TASK_FUNCTION_HPP__
#define __MY_TASK_FUNCTION_HPP__

#include "my_global.hpp"
#include <algorithm>


/**
 * @brief 定时线程回调测试函数
 */
void thread_callback_test();

/**
 * @brief 摄像头灰度图显示测试函数
 */
void camera_test();

/**
 * @brief 无屏幕综合状态测试函数
 * @note IPS 屏幕刷新会明显影响线程周期，所以需要测控制周期时优先用该函数。
 */
void show_all_of_the_component_without_ips();

/**
 * @brief 将当前 RGB 图像发送到上位机
 */
void send_picture_to_Serve();

/**
 * @brief 主循迹任务函数
 * @details 获取摄像头图像并调用 image_proc 完成视觉处理，最终方向会写入全局 onto。
 */
void tracking();

/**
 * @brief 数据集采集辅助函数
 */
void get_image_datasets();
#endif // __MY_TASK_FUNCTION_HPP__
