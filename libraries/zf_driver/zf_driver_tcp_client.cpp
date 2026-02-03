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

#include "zf_driver_tcp_client.hpp"

// #include <sys/select.h>
// #include <sys/time.h>
// #include <unistd.h>
// #include <cstdio>
// #include <cerrno>

// 设置文件句柄为非阻塞模式
int zf_driver_tcp_client::set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) return -1;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) return -1;
    return 0;
}

int zf_driver_tcp_client::wait_writable(int timeout_ms)
{
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(m_socket, &wfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int ret = select(m_socket + 1, NULL, &wfds, NULL, &tv);
    return ret; // >0 writable, 0 timeout, -1 error
}

zf_driver_tcp_client::zf_driver_tcp_client()
{
    m_socket = -1;
    memset(&m_server_addr, 0, sizeof(m_server_addr));
}

zf_driver_tcp_client::~zf_driver_tcp_client()
{
    if (m_socket >= 0)
    {
        close(m_socket);
        m_socket = -1;
    }
}

int8 zf_driver_tcp_client::init(const char *ip_addr, uint32 port)
{
    m_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket < 0)
    {
        perror("socket");
        return -1;
    }

    memset(&m_server_addr, 0, sizeof(m_server_addr));
    m_server_addr.sin_family = AF_INET;
    m_server_addr.sin_addr.s_addr = inet_addr(ip_addr);
    m_server_addr.sin_port = htons(port);

    printf("Wait connect tcp server %s:%u\r\n", ip_addr, (unsigned)port);

    int ret = connect(m_socket, (struct sockaddr*)&m_server_addr, sizeof(m_server_addr));
    if (ret < 0)
    {
        // 对于非阻塞connect you'd use EWOULDBLOCK; here we do blocking connect by default
        perror("connect() error");
        close(m_socket);
        m_socket = -1;
        return -1;
    }

    if (set_nonblocking(m_socket) < 0) {
        perror("set_nonblocking");
        close(m_socket);
        m_socket = -1;
        return -1;
    }

    return 0;
}

// 发送所有数据（处理短发送 / 非阻塞 EAGAIN）
// 返回实际发送的字节数，失败返回 -1
int32 zf_driver_tcp_client::send_data_all(const uint8 *buff, uint32 length, int timeout_ms)
{
    if (m_socket < 0) return -1;
    uint32_t total_sent = 0;
    const uint8 *p = buff;
    uint32_t remain = length;

    while (remain > 0)
    {
        ssize_t sent = send(m_socket, p, remain, 0);
        if (sent > 0)
        {
            total_sent += (uint32_t)sent;
            p += sent;
            remain -= (uint32_t)sent;
            continue;
        }
        else if (sent == 0)
        {
            // 远端关闭?
            return (int32)total_sent;
        }
        else // sent < 0
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 等待socket可写
                int sel = wait_writable(timeout_ms);
                if (sel <= 0)
                {
                    // 超时或出错
                    if (sel == 0) {
                        // 超时，返回已发送字节数（可能为0）
                        return (int32)total_sent;
                    } else {
                        // select 错误
                        return -1;
                    }
                }
                // 否则继续尝试发送
                continue;
            }
            else if (errno == EINTR)
            {
                continue; // 重试
            }
            else
            {
                perror("send");
                return -1;
            }
        }
    }

    return (int32)total_sent;
}

// 非阻塞读取数据：返回实际读取字节数；0 为无数据（EAGAIN/EWOULDBLOCK）；-1 为错误
int32 zf_driver_tcp_client::read_data(uint8 *buff, uint32 length)
{
    if (m_socket < 0) return -1;
    ssize_t r = recv(m_socket, buff, length, 0);
    if (r > 0) return (int32)r;
    if (r == 0) {
        // 远端已关闭连接
        return -1;
    }
    // r < 0
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
        return 0; // 非阻塞：当前无数据
    }
    if (errno == EINTR)
    {
        return 0; // 可以让调用者重试
    }
    perror("recv");
    return -1;
}