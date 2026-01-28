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
                                          float (*Lline)[2], int Lline_num,
                                          float (*Rline)[2], int Rline_num,
                                          float (*Mline)[2], int Mline_num,
                                          bool flip_vertical,
                                          bool flip_horizontal)
{
    // ==================== 1. 参数有效性检查 ====================
    if (gray_image_ptr == nullptr || width == 0 || height == 0)
    {
        std::cerr << "Error: Invalid image parameters in gray_img_with_centerline_transmitter: "
                  << "image_ptr=" << static_cast<const void*>(gray_image_ptr)
                  << ", width=" << width
                  << ", height=" << height << "\n";
        return false;
    }

    // 边线参数检查（允许部分边线为空，但至少要有一条有效边线才发送边界）
    bool has_valid_lline = (Lline != nullptr && Lline_num > 0 && Lline_num <= BOUNDARY_MAX_LEN);
    bool has_valid_rline = (Rline != nullptr && Rline_num > 0 && Rline_num <= BOUNDARY_MAX_LEN);
    bool has_valid_mline = (Mline != nullptr && Mline_num > 0 && Mline_num <= BOUNDARY_MAX_LEN);
    
    if (!has_valid_lline && !has_valid_rline && !has_valid_mline)
    {
        // 三条线都无效，打印警告但仍发送图像
        std::cerr << "Warning: All boundary lines are invalid. Sending image only.\n";
    }

    uint32_t src_pixels = width * height;
    uint32_t dst_pixels = UVC_WIDTH * UVC_HEIGHT;

    // 下面用于描述 image_copy 中实际写入的 block（用于把 Mline 映射到 image_copy）
    int copy_offset_x = 0;      // 水平偏移（像素）
    int copy_offset_y = 0;      // 垂直偏移（像素）
    uint32_t copy_w = 0;        // 写入宽度（像素）
    uint32_t copy_h = 0;        // 写入高度（像素）
    bool used_center_vertical = false; // 用到了垂直居中逻辑
    uint32_t vertical_padding = 0;
    bool need_scaling = (width != UVC_WIDTH) || (height != UVC_HEIGHT);

    // ==================== 2. 处理灰度图像（并记录 copy block 信息） ====================
    // 情况1：源图像尺寸与目标缓冲区完全匹配（最优情况）
    if (width == UVC_WIDTH && height == UVC_HEIGHT)
    {
        copy_offset_x = 0;
        copy_offset_y = 0;
        copy_w = UVC_WIDTH;
        copy_h = UVC_HEIGHT;
        used_center_vertical = false;

        if (!flip_vertical && !flip_horizontal)
        {
            // 直接整块拷贝最快
            memcpy(gray_image_copy[0], gray_image_ptr, src_pixels);
        }
        else
        {
            // 逐像素拷贝以实现翻转
            for (uint32_t row = 0; row < height; ++row)
            {
                uint32_t dst_row = flip_vertical ? (height - 1 - row) : row;
                uint32_t dst_row_offset = dst_row * UVC_WIDTH;
                for (uint32_t col = 0; col < width; ++col)
                {
                    uint32_t dst_col = flip_horizontal ? (width - 1 - col) : col;
                    ((uint8_t*)gray_image_copy[0])[dst_row_offset + dst_col] = gray_image_ptr[row * width + col];
                }
            }
        }
    }
    // 情况2：源图像像素数小于等于目标缓冲区
    else if (src_pixels <= dst_pixels)
    {
        if (!need_scaling && width == UVC_WIDTH && height <= UVC_HEIGHT)
        {
            // 宽度匹配，高度较小，垂直居中（代码原样）
            vertical_padding = (UVC_HEIGHT - height) / 2;
            copy_offset_x = 0;
            copy_offset_y = 0; // 注意：我们在映射时特别处理 vertical_padding 的影响
            copy_w = width;
            copy_h = height;
            used_center_vertical = true;

            // 清空整个缓冲区
            memset(gray_image_copy[0], 0, dst_pixels);

            // 垂直翻转/水平翻转并拷贝到 gray_image_copy 的合适位置
            for (uint32_t row = 0; row < height; ++row)
            {
                // 目标行（结合原实现的 dst_row_offset 计算）
                uint32_t dst_row;
                if (flip_vertical)
                {
                    // 原实现：dst_row = UVC_HEIGHT - 1 - row - vertical_padding
                    dst_row = UVC_HEIGHT - 1 - row - vertical_padding;
                }
                else
                {
                    // 将源行放到上边缘 + vertical_padding（也可选择放入从 vertical_padding 开始）
                    dst_row = row + vertical_padding;
                }
                uint32_t dst_row_offset = dst_row * UVC_WIDTH;

                if (!flip_horizontal)
                {
                    // 可整行拷贝
                    memcpy(&((uint8_t*)gray_image_copy[0])[dst_row_offset],
                           &gray_image_ptr[row * width],
                           width);
                }
                else
                {
                    // 水平翻转需要逐像素拷贝
                    for (uint32_t col = 0; col < width; ++col)
                    {
                        uint32_t dst_col = width - 1 - col;
                        ((uint8_t*)gray_image_copy[0])[dst_row_offset + dst_col] = gray_image_ptr[row * width + col];
                    }
                }
            }
        }
        else
        {
            // 简单处理：不进行缩放，只处理能放入的部分
            uint32_t rows_to_copy = std::min(height, (uint32_t)UVC_HEIGHT);
            uint32_t cols_to_copy = std::min(width, (uint32_t)UVC_WIDTH);

            copy_offset_x = 0;
            copy_offset_y = 0;
            copy_w = cols_to_copy;
            copy_h = rows_to_copy;
            used_center_vertical = false;

            // 清空整个缓冲区
            memset(gray_image_copy[0], 0, dst_pixels);

            if (!flip_vertical && !flip_horizontal)
            {
                // 可以按行 memcpy
                for (uint32_t row = 0; row < rows_to_copy; ++row)
                {
                    uint32_t src_row_offset = row * width;
                    uint32_t dst_row_offset = row * UVC_WIDTH;
                    memcpy(&((uint8_t*)gray_image_copy[0])[dst_row_offset],
                           &gray_image_ptr[src_row_offset],
                           cols_to_copy);
                }
            }
            else
            {
                // 需要逐像素拷贝以实现任意翻转组合
                for (uint32_t row = 0; row < rows_to_copy; ++row)
                {
                    uint32_t src_row_offset = row * width;
                    uint32_t dst_row_index = flip_vertical ? (rows_to_copy - 1 - row) : row;
                    uint32_t dst_row_offset = dst_row_index * UVC_WIDTH;

                    for (uint32_t col = 0; col < cols_to_copy; ++col)
                    {
                        uint32_t dst_col = flip_horizontal ? (cols_to_copy - 1 - col) : col;
                        ((uint8_t*)gray_image_copy[0])[dst_row_offset + dst_col] = gray_image_ptr[src_row_offset + col];
                    }
                }
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

    // ==================== 3. 处理三条边线数据（映射到 gray_image_copy 的坐标系并填充 boundary buffers） ====================
    // Lambda函数：坐标映射逻辑（源图像坐标 -> 发送缓冲区坐标）
    auto map_coordinate = [&](float fx, float fy, int &mapped_x, int &mapped_y) {
        // 四舍五入到源像素坐标并裁剪到 source 范围
        int src_x = static_cast<int>(std::lround(fx));
        int src_y = static_cast<int>(std::lround(fy));
        src_x = std::max(0, std::min(src_x, (int)width - 1));
        src_y = std::max(0, std::min(src_y, (int)height - 1));

        // 映射 X（水平）
        if (used_center_vertical) {
            uint32_t block_w = copy_w;
            int rel_x = src_x;
            if (flip_horizontal)
                mapped_x = (int)(block_w - 1 - rel_x) + copy_offset_x;
            else
                mapped_x = rel_x + copy_offset_x;
        } else {
            uint32_t block_w = copy_w ? copy_w : UVC_WIDTH;
            int rel_x = src_x;
            if ((uint32_t)rel_x >= block_w) rel_x = block_w - 1;
            if (flip_horizontal)
                mapped_x = (int)(block_w - 1 - rel_x) + copy_offset_x;
            else
                mapped_x = rel_x + copy_offset_x;
        }

        // 映射 Y（垂直）
        if (used_center_vertical) {
            if (flip_vertical)
                mapped_y = (int)(UVC_HEIGHT - 1 - src_y - vertical_padding);
            else
                mapped_y = src_y + (int)vertical_padding;
        } else {
            uint32_t block_h = copy_h ? copy_h : UVC_HEIGHT;
            int rel_y = src_y;
            if ((uint32_t)rel_y >= block_h) rel_y = block_h - 1;
            if (flip_vertical)
                mapped_y = (int)(block_h - 1 - rel_y) + copy_offset_y;
            else
                mapped_y = rel_y + copy_offset_y;
        }

        // 裁剪到 gray_image_copy 范围
        mapped_x = std::max(0, std::min(mapped_x, (int)UVC_WIDTH - 1));
        mapped_y = std::max(0, std::min(mapped_y, (int)UVC_HEIGHT - 1));
    };

    // 清空所有发送缓冲区
    memset(boundary_x1_buffer, 0, BOUNDARY_MAX_LEN);
    memset(boundary_y1_buffer, 0, BOUNDARY_MAX_LEN);
    memset(boundary_x2_buffer, 0, BOUNDARY_MAX_LEN);
    memset(boundary_y2_buffer, 0, BOUNDARY_MAX_LEN);
    memset(boundary_x3_buffer, 0, BOUNDARY_MAX_LEN);
    memset(boundary_y3_buffer, 0, BOUNDARY_MAX_LEN);

    // 处理左边线（对应逐飞助手的第1条边界线，通常为蓝色）
    if (has_valid_lline) {
        int points = std::min(Lline_num, BOUNDARY_MAX_LEN);
        for (int i = 0; i < points; ++i) {
            int mapped_x, mapped_y;
            map_coordinate(Lline[i][0], Lline[i][1], mapped_x, mapped_y);
            boundary_x1_buffer[i] = static_cast<uint8>(mapped_x & 0xFF);
            boundary_y1_buffer[i] = static_cast<uint8>(mapped_y & 0xFF);
        }
    }

    // 处理中线（对应逐飞助手的第2条边界线，通常为绿色）
    if (has_valid_mline) {
        int points = std::min(Mline_num, BOUNDARY_MAX_LEN);
        for (int i = 0; i < points; ++i) {
            int mapped_x, mapped_y;
            map_coordinate(Mline[i][0], Mline[i][1], mapped_x, mapped_y);
            boundary_x2_buffer[i] = static_cast<uint8>(mapped_x & 0xFF);
            boundary_y2_buffer[i] = static_cast<uint8>(mapped_y & 0xFF);
        }
    }

    // 处理右边线（对应逐飞助手的第3条边界线，通常为红色）
    if (has_valid_rline) {
        int points = std::min(Rline_num, BOUNDARY_MAX_LEN);
        for (int i = 0; i < points; ++i) {
            int mapped_x, mapped_y;
            map_coordinate(Rline[i][0], Rline[i][1], mapped_x, mapped_y);
            boundary_x3_buffer[i] = static_cast<uint8>(mapped_x & 0xFF);
            boundary_y3_buffer[i] = static_cast<uint8>(mapped_y & 0xFF);
        }
    }

    // 配置并发送边界（使用XY_BOUNDARY类型，支持任意坐标点）
    if (has_valid_lline || has_valid_mline || has_valid_rline) {
        // 取三条线中最大点数作为boundary_num（逐飞助手会根据实际点数显示）
        uint16_t max_points = 0;
        if (has_valid_lline) max_points = std::max(max_points, (uint16_t)std::min(Lline_num, BOUNDARY_MAX_LEN));
        if (has_valid_mline) max_points = std::max(max_points, (uint16_t)std::min(Mline_num, BOUNDARY_MAX_LEN));
        if (has_valid_rline) max_points = std::max(max_points, (uint16_t)std::min(Rline_num, BOUNDARY_MAX_LEN));

        seekfree_assistant_camera_boundary_config(
            XY_BOUNDARY,
            max_points,
            has_valid_lline ? boundary_x1_buffer : NULL,  // 左边线X
            has_valid_mline ? boundary_x2_buffer : NULL,  // 中线X
            has_valid_rline ? boundary_x3_buffer : NULL,  // 右边线X
            has_valid_lline ? boundary_y1_buffer : NULL,  // 左边线Y
            has_valid_mline ? boundary_y2_buffer : NULL,  // 中线Y
            has_valid_rline ? boundary_y3_buffer : NULL   // 右边线Y
        );
    } else {
        // 三条线都无效，不发送边界
        seekfree_assistant_camera_boundary_config(XY_BOUNDARY, 0,
                                                  NULL, NULL, NULL,
                                                  NULL, NULL, NULL);
    }

    // ==================== 4. 配置并发送图像 ====================
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_GRAY,
                                                 gray_image_copy[0],
                                                 UVC_WIDTH,
                                                 UVC_HEIGHT);

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