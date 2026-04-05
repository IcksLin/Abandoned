/*********************************************************************************************************************
* 文件名称          my_image_transmitter
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-24        HeavenCornerstone         
********************************************************************************************************************/
// #pragma once
#ifndef __MY_IMAGE_TRANSMITTER_HPP__  
#define __MY_IMAGE_TRANSMITTER_HPP__  

#include "zf_common_typedef.hpp"
#include "zf_driver_udp.hpp"
#include "zf_device_uvc.hpp"
#include "seekfree_assistant_interface.hpp"
#include "seekfree_assistant.hpp"
#include "zf_driver_tcp_client.hpp"

#include <cstdint>
#include <arpa/inet.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>
// TCP图像传输服务器配置
#define SERVER_IP                "192.168.43.9"                 // 上位机IP地址
#define SERVER_PORT              8086                           // 上位机端口号

uint32 tcp_send_wrap(const uint8 *buf, uint32 len);
uint32 tcp_read_wrap(uint8 *buf, uint32 len);
bool img_transmitter_init();
bool rgb_img_transmitter(const uint16_t* rgb_image_ptr, uint32_t width, uint32_t height, bool flip_vertical = false);

// 灰度图像 + 三条边线传输函数（左边线、右边线、中线）
bool gray_img_with_centerline_transmitter(const uint8_t* gray_image_ptr, 
                                          uint32_t width, uint32_t height,
                                          uint8 (*Lline)[2], int Lline_num,
                                          uint8 (*Rline)[2], int Rline_num,
                                          uint8 (*Mline)[2], int Mline_num,
                                          bool flip_vertical,
                                          bool flip_horizontal);

bool is_transmitter_ready();
void img_transmitter_deinit();






#endif // _ZF_DRIVER_IMAGE_CLIENT_HPP_