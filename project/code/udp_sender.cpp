/*********************************************************************************************************************
* UDP发送模块 - 实现文件（纯图传版本）
* 功能：通过UDP协议发送JPEG图像或原始数据
* 适配：LS2K0300开源库
********************************************************************************************************************/

#include "udp_sender.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>

//-------------------------------------------------------------------------------------------------------------------
// 【用户数据区域 - 请在此定义你的二值化和边界数组】
//-------------------------------------------------------------------------------------------------------------------
// MODE_BINARY 模式需要：
//   - user_binary_image: 二值化图像数组
//   - user_image_width: 图像宽度
//   - user_image_height: 图像高度
//
// MODE_BINARY_EDGE 模式需要：
//   - user_binary_image: 二值化图像数组
//   - user_image_width: 图像宽度
//   - user_image_height: 图像高度
//   - user_left_line: 左边界数组
//   - user_mid_line: 中线数组
//   - user_right_line: 右边界数组
//-------------------------------------------------------------------------------------------------------------------

// 【替换位置1】定义你的二值化图像数组
// 示例：uint8 user_binary_image[120][160];
uint8* user_binary_image = nullptr;  // 请替换为你的数组

// 【替换位置2】定义图像尺寸
// 示例：int user_image_width = 160; int user_image_height = 120;
int user_image_width = 0;   // 请替换为你的宽度
int user_image_height = 0;  // 请替换为你的高度

// 【替换位置3】定义边界数组（MODE_BINARY_EDGE模式需要）
// 示例：int user_left_line[120]; int user_mid_line[120]; int user_right_line[120];
int* user_left_line = nullptr;   // 请替换为你的左边界数组
int* user_mid_line = nullptr;    // 请替换为你的中线数组
int* user_right_line = nullptr;  // 请替换为你的右边界数组

//-------------------------------------------------------------------------------------------------------------------

// 静态成员变量定义
const int udp_sender::MAX_UDP_PACKET;

//-------------------------------------------------------------------------------------------------------------------
// 构造函数
//-------------------------------------------------------------------------------------------------------------------
udp_sender::udp_sender()
    : udp_socket_fd(-1)
    , is_enabled(false)
    , send_mode(MODE_COLOR)
{
    memset(&udp_server_addr, 0, sizeof(udp_server_addr));
}

//-------------------------------------------------------------------------------------------------------------------
// 析构函数
//-------------------------------------------------------------------------------------------------------------------
udp_sender::~udp_sender()
{
    close();
}

//-------------------------------------------------------------------------------------------------------------------
// 初始化UDP发送器
//-------------------------------------------------------------------------------------------------------------------
int8 udp_sender::init(const char* server_ip, int port)
{
    // 创建UDP socket
    udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd < 0) {
        printf("UDP socket create failed\n");
        return -1;
    }
    
    // 设置非阻塞模式
    int flags = fcntl(udp_socket_fd, F_GETFL, 0);
    fcntl(udp_socket_fd, F_SETFL, flags | O_NONBLOCK);
    
    // 配置服务器地址
    memset(&udp_server_addr, 0, sizeof(udp_server_addr));
    udp_server_addr.sin_family = AF_INET;
    udp_server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, server_ip, &udp_server_addr.sin_addr) <= 0) {
        printf("UDP invalid address: %s\n", server_ip);
        ::close(udp_socket_fd);
        udp_socket_fd = -1;
        return -1;
    }
    
    // 初始化JPEG编码参数（只创建一次）
    jpeg_params.clear();
    jpeg_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    jpeg_params.push_back(JPEG_QUALITY);
    jpeg_params.push_back(cv::IMWRITE_JPEG_OPTIMIZE);
    jpeg_params.push_back(0);
    jpeg_params.push_back(cv::IMWRITE_JPEG_PROGRESSIVE);
    jpeg_params.push_back(0);
    
    printf("UDP initialized: %s:%d\n", server_ip, port);
    is_enabled = true;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 发送cv::Mat图像（JPEG压缩后发送）
//-------------------------------------------------------------------------------------------------------------------
bool udp_sender::send_image(const cv::Mat& image)
{
    if (!is_enabled || udp_socket_fd < 0 || image.empty()) {
        return false;
    }
    
    // JPEG压缩（复用jpeg_buffer，避免每帧分配内存）
    if (!cv::imencode(".jpg", image, jpeg_buffer, jpeg_params)) {
        return false;
    }
    
    // 检查大小并发送
    if (jpeg_buffer.size() > (size_t)MAX_UDP_PACKET) {
        return false;
    }
    
    ssize_t sent = sendto(udp_socket_fd, jpeg_buffer.data(), jpeg_buffer.size(), 0,
                          (struct sockaddr*)&udp_server_addr, sizeof(udp_server_addr));
    return (sent > 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 发送原始数据
//-------------------------------------------------------------------------------------------------------------------
bool udp_sender::send_data(const void* data, size_t length)
{
    if (!is_enabled || udp_socket_fd < 0 || data == nullptr || length == 0) {
        return false;
    }
    
    if (length > (size_t)MAX_UDP_PACKET) {
        return false;
    }
    
    ssize_t sent = sendto(udp_socket_fd, data, length, 0,
                          (struct sockaddr*)&udp_server_addr, sizeof(udp_server_addr));
    
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        printf("UDP send failed: %s\n", strerror(errno));
        return false;
    }
    
    return (sent == (ssize_t)length);
}

//-------------------------------------------------------------------------------------------------------------------
// 发送原始二值化图像（带包头）
//-------------------------------------------------------------------------------------------------------------------
bool udp_sender::send_raw_image(const uint8* image_data, int width, int height)
{
    if (!is_enabled || udp_socket_fd < 0 || image_data == nullptr) {
        return false;
    }
    
    size_t header_size = sizeof(raw_image_packet_t);
    size_t image_size = width * height;
    size_t total_size = header_size + image_size;
    
    if (total_size > (size_t)MAX_UDP_PACKET) {
        printf("[ERROR] Image too large: %zu bytes (max %d)\n", total_size, MAX_UDP_PACKET);
        return false;
    }
    
    // 构建数据包
    uint8 packet[total_size];
    raw_image_packet_t* header = (raw_image_packet_t*)packet;
    
    header->header[0] = 'B';
    header->header[1] = 'I';
    header->header[2] = 'M';
    header->header[3] = 'G';
    header->image_width = width;
    header->image_height = height;
    
    // 复制图像数据
    memcpy(packet + header_size, image_data, image_size);
    
    // 发送
    ssize_t sent = sendto(udp_socket_fd, packet, total_size, 0,
                          (struct sockaddr*)&udp_server_addr, sizeof(udp_server_addr));
    
    return (sent == (ssize_t)total_size);
}

//-------------------------------------------------------------------------------------------------------------------
// 关闭UDP连接
//-------------------------------------------------------------------------------------------------------------------
void udp_sender::close()
{
    is_enabled = false;
    if (udp_socket_fd >= 0) {
        ::close(udp_socket_fd);
        udp_socket_fd = -1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 获取使能状态
//-------------------------------------------------------------------------------------------------------------------
bool udp_sender::is_enable() const
{
    return is_enabled;
}

//-------------------------------------------------------------------------------------------------------------------
// 设置图像发送模式
//-------------------------------------------------------------------------------------------------------------------
void udp_sender::set_send_mode(ImageSendMode mode)
{
    send_mode = mode;
    const char* mode_names[] = {"COLOR", "BINARY", "BINARY_EDGE"};
    printf("UDP send mode changed to: %s\n", mode_names[mode]);
}

//-------------------------------------------------------------------------------------------------------------------
// 获取当前发送模式
//-------------------------------------------------------------------------------------------------------------------
ImageSendMode udp_sender::get_send_mode() const
{
    return send_mode;
}

//-------------------------------------------------------------------------------------------------------------------
// 处理单帧图像 - 纯图传功能（无检测）
// color_frame 由调用方传入，udp_sender 不再依赖摄像头设备
//-------------------------------------------------------------------------------------------------------------------
int8 udp_sender::process_frame(const cv::Mat& color_frame)
{
    // 根据发送模式决定如何发送图像
    switch(send_mode)
    {
        case MODE_COLOR:
        {
            // 彩色图像模式：直接发送彩色图像
            if (color_frame.empty()) return -1;
            send_image(color_frame);
            break;
        }
            
        case MODE_BINARY:
        {
            if(user_binary_image != nullptr && user_image_width > 0 && user_image_height > 0)
            {
                // 使用用户提供的二值化图像
                send_raw_image(user_binary_image, user_image_width, user_image_height);
            }
            else
            {
                // 默认实现：对传入彩色帧自动二值化（OTSU）
                if (color_frame.empty()) return -1;
                cv::Mat gray, binary;
                cv::cvtColor(color_frame, gray, cv::COLOR_BGR2GRAY);
                cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
                send_raw_image(binary.data, binary.cols, binary.rows);
            }
            break;
        }
        
        case MODE_BINARY_EDGE:
        {
            if(user_binary_image != nullptr && user_image_width > 0 && user_image_height > 0 &&
               user_left_line != nullptr && user_mid_line != nullptr && user_right_line != nullptr)
            {
                // 1. 发送原始二值化图像（带BIMG包头）
                send_raw_image(user_binary_image, user_image_width, user_image_height);
                
                // 2. 构建边界数据包
                edge_data_packet_t edge_packet;
                memset(&edge_packet, 0, sizeof(edge_packet));
                edge_packet.header[0] = 'E'; edge_packet.header[1] = 'D';
                edge_packet.header[2] = 'G'; edge_packet.header[3] = 'E';
                edge_packet.image_width  = user_image_width;
                edge_packet.image_height = user_image_height;
                
                int copy_height = (user_image_height < 120) ? user_image_height : 120;
                for(int y = 0; y < copy_height; y++)
                {
                    edge_packet.left_line[y]  = (int16)user_left_line[y];
                    edge_packet.mid_line[y]   = (int16)user_mid_line[y];
                    edge_packet.right_line[y] = (int16)user_right_line[y];
                }
                
                // 计算校验和
                uint16 checksum = 0;
                uint8* data = (uint8*)&edge_packet;
                for(size_t i = 0; i < sizeof(edge_packet) - sizeof(uint16); i++)
                    checksum ^= data[i];
                edge_packet.checksum = checksum;
                
                send_data(&edge_packet, sizeof(edge_packet));
            }
            else
            {
                // 默认实现：对传入彩色帧自动二值化 + 空边界
                if (color_frame.empty()) return -1;
                cv::Mat gray, binary;
                cv::cvtColor(color_frame, gray, cv::COLOR_BGR2GRAY);
                cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
                
                int width = binary.cols, height = binary.rows;
                send_raw_image(binary.data, width, height);
                
                edge_data_packet_t edge_packet;
                memset(&edge_packet, 0, sizeof(edge_packet));
                edge_packet.header[0] = 'E'; edge_packet.header[1] = 'D';
                edge_packet.header[2] = 'G'; edge_packet.header[3] = 'E';
                edge_packet.image_width  = width;
                edge_packet.image_height = height;
                
                for(int y = 0; y < 120; y++)
                {
                    edge_packet.left_line[y]  = -1;
                    edge_packet.mid_line[y]   = -1;
                    edge_packet.right_line[y] = -1;
                }
                
                uint16 checksum = 0;
                uint8* data = (uint8*)&edge_packet;
                for(size_t i = 0; i < sizeof(edge_packet) - sizeof(uint16); i++)
                    checksum ^= data[i];
                edge_packet.checksum = checksum;
                
                send_data(&edge_packet, sizeof(edge_packet));
            }
            break;
        }
        
        default:
            if (color_frame.empty()) return -1;
            send_image(color_frame);
            break;
    }
    
    return 0;
}
