#include "my_image_transmitter.hpp"

// ============================================================================
// 全局设备对象定义
// ============================================================================

/// @brief TCP客户端设备对象，用于建立TCP网络连接和收发数据
zf_driver_tcp_client tcp_client_dev;

/// @brief RGB图像数据指针，指向摄像头采集的RGB图像首地址
uint16* rgb_image = nullptr;
// ============================================================================
// 函数声明与封装
// ============================================================================

/**
 * @brief TCP发送数据全局包装函数
 * @param buf 要发送的数据缓冲区指针
 * @param len 要发送的数据字节长度
 * @return uint32 实际成功发送的字节数
 * @note 封装tcp_client_dev.send_data成员函数，适配普通函数指针格式要求
 *       供seekfree_assistant_interface_init调用，无需手动调用
 */
uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data_all(buf, len);
}

/**
 * @brief TCP接收数据全局包装函数
 * @param buf 接收数据的缓冲区指针
 * @param len 最大可接收的字节长度
 * @return uint32 实际成功接收的字节数
 * @note 封装tcp_client_dev.read_data成员函数，适配普通函数指针格式要求
 *       供seekfree_assistant_interface_init调用，无需手动调用
 */
uint32 tcp_read_wrap(uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}


/// @brief 图像数据拷贝缓冲区
/// @note 二维数组存储，用于存放格式转换后的摄像头图像数据，供上位机发送使用
uint16 image_copy[UVC_HEIGHT][UVC_WIDTH];
uint8 gray_image_copy[UVC_HEIGHT][UVC_WIDTH];
// ============================================================================
// 初始化函数
// ============================================================================

/**
 * @brief 图像传输初始化函数
 * @return bool 初始化是否成功
 * @retval true  初始化成功
 * @retval false 初始化失败
 * 
 * 主要完成以下初始化工作：
 * 1. 初始化TCP客户端连接到指定服务器
 * 2. 注册数据发送和接收回调函数
 * 3. 分配RGB图像数据缓冲区内存
 * 4. 配置摄像头信息（灰度模式）
 */
bool img_transmitter_init()
{
    // 1. 初始化TCP客户端，连接到指定的服务器IP和端口
    if (tcp_client_dev.init(SERVER_IP, SERVER_PORT) != 0)
    {
        std::cerr << "Failed to init TCP client to " 
                  << SERVER_IP << ":" << SERVER_PORT << "\n";
        return false;  // 初始化失败
    }

    // 2. 注册数据发送和接收回调函数，供底层通信模块使用
    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

    // 3. 分配RGB图像数据缓冲区内存，用于存放摄像头采集到的图像数据
    rgb_image = new uint16[UVC_WIDTH * UVC_HEIGHT];
    if (rgb_image == nullptr)
    {
        std::cerr << "Failed to allocate memory for RGB image buffer\n";
        return false;  // 内存分配失败
    }

    // 4. 配置摄像头信息为RGB565模式
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_RGB565, 
        image_copy[0], 
        UVC_WIDTH, 
        UVC_HEIGHT
    );

    return true;  // 初始化成功
}

// ============================================================================
// 图像发送函数
// ============================================================================

/**
 * @brief 发送RGB565图像数据到上位机（包含字节顺序转换）
 * @param rgb_image_ptr 指向原始RGB565图像数据的指针
 * @param width 图像宽度（像素）
 * @param height 图像高度（像素）
 * @return bool 发送是否成功
 * @note 函数会处理RGB565数据的高低字节交换，确保上位机能正确解析彩色图像
 *       如果图像尺寸与预设尺寸不匹配，会进行适当调整或拒绝发送
 */
bool rgb_img_transmitter(const uint16_t* rgb_image_ptr, uint32_t width, uint32_t height, bool flip_vertical)
{
    // 参数有效性检查
    if (rgb_image_ptr == nullptr || width == 0 || height == 0)
    {
        std::cerr << "Error: Invalid parameters in rgb_img_transmitter: "
                  << "image_ptr=" << static_cast<const void*>(rgb_image_ptr)
                  << ", width=" << width
                  << ", height=" << height << "\n";
        return false;
    }

    uint32_t src_pixels = width * height;
    uint32_t dst_pixels = UVC_WIDTH * UVC_HEIGHT;
    
    // 情况1：源图像尺寸与目标缓冲区完全匹配（最优情况）
    if (width == UVC_WIDTH && height == UVC_HEIGHT)
    {
        if (flip_vertical)
        {
            // 垂直翻转处理：逐行倒序拷贝
            for (uint32_t row = 0; row < height; row++)
            {
                // 计算源图像和目标图像的行偏移
                uint32_t src_row_offset = row * width;  // 源图像从第0行开始
                uint32_t dst_row_offset = (height - 1 - row) * width;  // 目标图像从最后一行开始
                
                for (uint32_t col = 0; col < width; col++)
                {
                    uint16_t original_pixel = rgb_image_ptr[src_row_offset + col];
                    uint16_t swapped_pixel = __builtin_bswap16(original_pixel);
                    ((uint16_t*)image_copy[0])[dst_row_offset + col] = swapped_pixel;
                }
            }
        }
        else
        {
            // 无翻转，直接处理
            for (uint32_t i = 0; i < src_pixels; i++)
            {
                uint16_t original_pixel = rgb_image_ptr[i];
                uint16_t swapped_pixel = __builtin_bswap16(original_pixel);
                ((uint16_t*)image_copy[0])[i] = swapped_pixel;
            }
        }
    }
    // 情况2：源图像像素数小于等于目标缓冲区
    else if (src_pixels <= dst_pixels)
    {
        // 检查是否需要调整尺寸以适应目标缓冲区
        bool need_scaling = (width != UVC_WIDTH) || (height != UVC_HEIGHT);
        
        if (flip_vertical)
        {
            if (!need_scaling && width == UVC_WIDTH && height <= UVC_HEIGHT)
            {
                // 宽度匹配，高度较小，直接垂直翻转并居中
                uint32_t vertical_padding = (UVC_HEIGHT - height) / 2;  // 上下边距
                
                // 清空整个缓冲区
                memset(image_copy[0], 0, dst_pixels * sizeof(uint16_t));
                
                // 逐行处理并垂直翻转
                for (uint32_t row = 0; row < height; row++)
                {
                    uint32_t src_row_offset = row * width;
                    uint32_t dst_row_offset = (UVC_HEIGHT - 1 - row - vertical_padding) * UVC_WIDTH;
                    
                    for (uint32_t col = 0; col < width; col++)
                    {
                        uint16_t original_pixel = rgb_image_ptr[src_row_offset + col];
                        uint16_t swapped_pixel = ((original_pixel & 0x00FF) << 8) | 
                                                 ((original_pixel & 0xFF00) >> 8);
                        ((uint16_t*)image_copy[0])[dst_row_offset + col] = swapped_pixel;
                    }
                }
            }
            else
            {
                // 简单处理：不进行缩放，只处理能放入的部分，并垂直翻转
                uint32_t rows_to_copy = std::min(height, (uint32_t)UVC_HEIGHT);
                uint32_t cols_to_copy = std::min(width, (uint32_t)UVC_WIDTH);
                
                // 清空整个缓冲区
                memset(image_copy[0], 0, dst_pixels * sizeof(uint16_t));
                
                // 垂直翻转并拷贝
                for (uint32_t row = 0; row < rows_to_copy; row++)
                {
                    uint32_t src_row_offset = row * width;
                    uint32_t dst_row_offset = (rows_to_copy - 1 - row) * UVC_WIDTH;
                    
                    for (uint32_t col = 0; col < cols_to_copy; col++)
                    {
                        uint16_t original_pixel = rgb_image_ptr[src_row_offset + col];
                        uint16_t swapped_pixel = ((original_pixel & 0x00FF) << 8) | 
                                                 ((original_pixel & 0xFF00) >> 8);
                        ((uint16_t*)image_copy[0])[dst_row_offset + col] = swapped_pixel;
                    }
                }
            }
        }
        else
        {
            // 无翻转，直接拷贝并交换字节顺序
            for (uint32_t i = 0; i < src_pixels; i++)
            {
                uint16_t original_pixel = rgb_image_ptr[i];
                uint16_t swapped_pixel = ((original_pixel & 0x00FF) << 8) | 
                                         ((original_pixel & 0xFF00) >> 8);
                ((uint16_t*)image_copy[0])[i] = swapped_pixel;
            }
            
            // 清空剩余缓冲区
            if (src_pixels < dst_pixels)
            {
                memset(((uint16_t*)image_copy[0]) + src_pixels, 0, 
                       (dst_pixels - src_pixels) * sizeof(uint16_t));
            }
        }
    }
    // 情况3：源图像像素数大于目标缓冲区
    else
    {
        std::cerr << "Error: Source image (" << width << "x" << height 
                  << " = " << src_pixels << " pixels) is larger than buffer capacity (" 
                  << UVC_WIDTH << "x" << UVC_HEIGHT << " = " << dst_pixels 
                  << " pixels). Operation aborted.\n";
        return false;
    }
    
    // 发送处理后的图像数据到上位机
    seekfree_assistant_camera_send();
    
    return true;
}

/**
 * @brief 灰度图像和边线（左、右、中）发送函数
 * @param gray_image_ptr 灰度图像数据指针 (uint8格式)
 * @param width 图像宽度
 * @param height 图像高度
 * @param Lline 左边线点数组指针 [x, y]格式
 * @param Lline_num 左边线点数
 * @param Rline 右边线点数组指针 [x, y]格式
 * @param Rline_num 右边线点数
 * @param Mline 中线点数组指针 [x, y]格式
 * @param Mline_num 中线点数
 * @param flip_vertical 是否垂直翻转
 * @param flip_horizontal 是否水平翻转
 * @return 发送是否成功
 * @note 左边线、右边线、中线分别对应逐飞助手的三条边界线（通常为蓝色、红色、绿色）
 */
#define BOUNDARY_MAX_LEN   120 
// 三条边线的XY坐标发送缓冲区
uint8 boundary_x1_buffer[BOUNDARY_MAX_LEN];  // 左边线X坐标
uint8 boundary_y1_buffer[BOUNDARY_MAX_LEN];  // 左边线Y坐标
uint8 boundary_x2_buffer[BOUNDARY_MAX_LEN];  // 中线X坐标
uint8 boundary_y2_buffer[BOUNDARY_MAX_LEN];  // 中线Y坐标
uint8 boundary_x3_buffer[BOUNDARY_MAX_LEN];  // 右边线X坐标
uint8 boundary_y3_buffer[BOUNDARY_MAX_LEN];  // 右边线Y坐标
bool gray_img_with_centerline_transmitter(const uint8_t* gray_image_ptr, 
                                          uint32_t width, uint32_t height,
                                          uint8 (*Lline)[2], int Lline_num,
                                          uint8 (*Rline)[2], int Rline_num,
                                          uint8 (*Mline)[2], int Mline_num,
                                          bool flip_vertical,
                                          bool flip_horizontal)
{
    if (!gray_image_ptr || width == 0 || height == 0) return false;

    // --- 1. 计算图像映射参数 ---
    // 实际拷贝的尺寸（不能超过缓冲区）
    uint32_t copy_w = std::min(width, (uint32_t)UVC_WIDTH);
    uint32_t copy_h = std::min(height, (uint32_t)UVC_HEIGHT);
    
    // 垂直居中偏移量：如果图像矮于缓冲区，将其放在中间
    int offset_y = (height < UVC_HEIGHT) ? (int)(UVC_HEIGHT - height) / 2 : 0;
    int offset_x = 0; // 默认左对齐

    // 清空图像缓冲区（背景置黑）
    memset(gray_image_copy, 0, sizeof(gray_image_copy));

    // --- 2. 拷贝图像并处理翻转 ---
    for (uint32_t row = 0; row < copy_h; ++row) {
        // 计算目标行索引：考虑垂直翻转 + 垂直偏移
        uint32_t dst_row = (flip_vertical ? (copy_h - 1 - row) : row) + offset_y;
        
        uint8_t* dst_ptr = &((uint8_t*)gray_image_copy)[dst_row * UVC_WIDTH];
        const uint8_t* src_ptr = &gray_image_ptr[row * width];

        for (uint32_t col = 0; col < copy_w; ++col) {
            // 计算目标列索引：考虑水平翻转
            uint32_t dst_col = (flip_horizontal ? (copy_w - 1 - col) : col) + offset_x;
            dst_ptr[dst_col] = src_ptr[col];
        }
    }

    // --- 3. 映射赛道线坐标 ---
    // 定义一个 lambda 闭包来统一处理坐标映射逻辑，确保线和图像是对齐的
    auto map_points = [&](uint8 (*src_line)[2], int num, uint8* target_x, uint8* target_y) -> int {
        int actual_num = std::min(num, BOUNDARY_MAX_LEN);
        if (src_line == nullptr || actual_num <= 0) return 0;

        for (int i = 0; i < actual_num; ++i) {
            // 获取原始坐标（此时已经是 uint8）
            int sx = src_line[i][0];
            int sy = src_line[i][1];

            // 执行与图像完全一致的翻转和偏移逻辑
            int tx = (flip_horizontal ? ((int)copy_w - 1 - sx) : sx) + offset_x;
            int ty = (flip_vertical ? ((int)copy_h - 1 - sy) : sy) + offset_y;

            // 限制在 UVC 范围内，防止上位机结算错误
            target_x[i] = (uint8)std::max(0, std::min(tx, (int)UVC_WIDTH - 1));
            target_y[i] = (uint8)std::max(0, std::min(ty, (int)UVC_HEIGHT - 1));
        }
        return actual_num;
    };

    // 填充发送缓冲区
    int n1 = map_points(Lline, Lline_num, boundary_x1_buffer, boundary_y1_buffer);
    int n2 = map_points(Mline, Mline_num, boundary_x2_buffer, boundary_y2_buffer);
    int n3 = map_points(Rline, Rline_num, boundary_x3_buffer, boundary_y3_buffer);

    // --- 4. 调用协议发送 ---
    // 计算三条线中最大的点数
    uint16_t max_points = (uint16_t)std::max({n1, n2, n3});

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY, max_points,
        n1 ? boundary_x1_buffer : nullptr, 
        n2 ? boundary_x2_buffer : nullptr, 
        n3 ? boundary_x3_buffer : nullptr,
        n1 ? boundary_y1_buffer : nullptr, 
        n2 ? boundary_y2_buffer : nullptr, 
        n3 ? boundary_y3_buffer : nullptr
    );

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_GRAY, 
        (uint8_t*)gray_image_copy, 
        UVC_WIDTH, 
        UVC_HEIGHT
    );

    seekfree_assistant_camera_send();

    return true;
}

// ============================================================================
// 辅助函数（可根据需要添加）
// ============================================================================

/**
 * @brief 检查传输模块是否已准备好
 * @return bool 模块准备状态
 * 
 * 可用于在发送图像前检查模块初始化状态和连接状态
 */
bool is_transmitter_ready()
{
    // 此处可添加更详细的状态检查逻辑
    return (rgb_image != nullptr);
}

/**
 * @brief 释放传输模块资源
 * 
 * 在程序退出或模块不再使用时调用，避免内存泄漏
 */
void img_transmitter_deinit()
{
    if (rgb_image != nullptr)
    {
        delete[] rgb_image;
        rgb_image = nullptr;
    }
    
    // 可根据需要添加TCP客户端关闭逻辑
    // tcp_client_dev.close();
}