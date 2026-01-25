/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#pragma once
#ifndef _ZF_DRIVER_TCP_CLIENT_HPP_
#define _ZF_DRIVER_TCP_CLIENT_HPP_

#include "zf_common_typedef.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class zf_driver_tcp_client
{
private:
    int                 m_socket;        // TCP套接字句柄
    struct sockaddr_in  m_server_addr;   // 服务器地址结构体

    int set_nonblocking(int fd);         // 设置非阻塞模式私有方法
    int wait_writable(int timeout_ms);   // 等待socket可写（用于非阻塞发送）

    zf_driver_tcp_client(const zf_driver_tcp_client&) = delete;
    zf_driver_tcp_client& operator=(const zf_driver_tcp_client&) = delete;

public:
    zf_driver_tcp_client(void);
    ~zf_driver_tcp_client(void);

    // 初始化并连接
    int8 init(const char *ip_addr, uint32 port);

    // 发送数据（尽量发送完所有数据，返回实际发送字节数；出错返回 -1）
    int32 send_data_all(const uint8 *buff, uint32 length, int timeout_ms = 500);

    // 非阻塞读取数据：返回 >=0 表示读到的字节数；0 表示无数据（非阻塞）；-1 表示错误
    int32 read_data(uint8 *buff, uint32 length);
};

#endif // _ZF_DRIVER_TCP_CLIENT_HPP_