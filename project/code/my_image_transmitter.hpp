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
#include "zf_driver_tcp_client.hpp"

#include <cstdint>
#include <arpa/inet.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>
// 简单的图像传输协议头（网络字节序传输）
// 总头长度为 24 字节
#pragma pack(push, 1)
struct zf_image_packet_header_t {
    uint32_t magic;        // 魔数，识别包，例如 0x5A465644 ('ZFVD')
    uint16_t format;       // 1=JPEG, 2=GRAY_RAW, 3=RGB565_RAW, 4=BGR_RAW
    uint16_t reserved;     // 保留
    uint32_t width;        // 图像宽
    uint32_t height;       // 图像高
    uint32_t payload_len;  // 负载长度（字节）
    uint32_t seq;          // 序号，可选
};
#pragma pack(pop)

class zf_driver_image_client
{
private:
    zf_driver_tcp_client m_tcp;
    uint32_t m_seq;

public:
    zf_driver_image_client();
    ~zf_driver_image_client();

    // 连接服务器（使用 SERVER_IP / SERVER_PORT 常量时可直接传入）
    int8 init(const char *ip_addr, uint32 port);

    // 将 OpenCV Mat 编码为 JPEG 后发送（返回发送的字节数，失败返回 -1）
    // quality: 0-100
    int32 send_mat_as_jpeg(const cv::Mat &mat, int quality = 90, int timeout_ms = 1000);

    // 直接发送灰度原始数据（data 指向连续的 width*height 字节）
    int32 send_gray_raw(const uint8_t *data, uint32 width, uint32 height, int timeout_ms = 1000);
};

#endif // _ZF_DRIVER_IMAGE_CLIENT_HPP_