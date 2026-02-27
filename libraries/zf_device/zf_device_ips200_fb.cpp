#include "zf_device_ips200_fb.hpp"
#include "zf_common_font.hpp"
#include "zf_common_function.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无参构造函数
//-------------------------------------------------------------------------------------------------------------------
zf_device_ips200::zf_device_ips200(void)
{
    pen_color = DEFAULT_PENCOLOR;
    bg_color = DEFAULT_BGCOLOR;
    width = 0;
    height = 0;
    screen_base = nullptr;
    buffer = nullptr;
    // 初始化脏区域为无效状态
    dirty_min_x = 10000;  // 用一个大于屏幕宽度的值
    dirty_min_y = 10000;
    dirty_max_x = -1;
    dirty_max_y = -1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     析构函数
//-------------------------------------------------------------------------------------------------------------------
zf_device_ips200::~zf_device_ips200()
{
    if (buffer) {
        delete[] buffer;
        buffer = nullptr;
    }
    if (screen_base && screen_base != MAP_FAILED) {
        munmap((void*)screen_base, width * height * sizeof(uint16));
        screen_base = nullptr;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清屏函数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::clear(void)
{
    full(DEFAULT_BGCOLOR);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕填充函数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::full(const uint16 color)
{
    if (!buffer) return;
    // 填充整个缓冲区
    for (int i = 0; i < width * height; i++) {
        buffer[i] = color;
    }
    // 标记全屏为脏
    dirty_min_x = 0;
    dirty_min_y = 0;
    dirty_max_x = width - 1;
    dirty_max_y = height - 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画点函数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::draw_point(uint16 x, uint16 y, const uint16 color)
{
    if (!buffer) return;
    // 简单边界检查（可根据需要添加）
    if (x >= width || y >= height) return;

    buffer[y * width + x] = color;
    mark_dirty_point(x, y);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画线函数 (保留原有算法，但调用 draw_point 以使用缓冲区)
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
    int16 x_dir = (x_start < x_end ? 1 : -1);
    int16 y_dir = (y_start < y_end ? 1 : -1);
    float temp_rate = 0;
    float temp_b = 0;

    do
    {
        if(x_start != x_end)
        {
            temp_rate = (float)(y_start - y_end) / (float)(x_start - x_end);
            temp_b = (float)y_start - (float)x_start * temp_rate;
        }
        else
        {
            while(y_start != y_end)
            {
                draw_point(x_start, y_start, color);
                y_start += y_dir;
            }
            draw_point(x_start, y_start, color);
            break;
        }
        if(func_abs(y_start - y_end) > func_abs(x_start - x_end))
        {
            while(y_start != y_end)
            {
                draw_point(x_start, y_start, color);
                y_start += y_dir;
                x_start = (int16)(((float)y_start - temp_b) / temp_rate);
            }
            draw_point(x_start, y_start, color);
        }
        else
        {
            while(x_start != x_end)
            {
                draw_point(x_start, y_start, color);
                x_start += x_dir;
                y_start = (int16)((float)x_start * temp_rate + temp_b);
            }
            draw_point(x_start, y_start, color);
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示单个字符
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_char(uint16 x, uint16 y, const char dat)
{
    uint8 i = 0, j = 0;
    for(i = 0; 8 > i; i ++)
    {
        uint8 temp_top = ascii_font_8x16[dat - 32][i];
        uint8 temp_bottom = ascii_font_8x16[dat - 32][i + 8];
        for(j = 0; 8 > j; j ++)
        {
            if(temp_top & 0x01)
            {
                draw_point(x + i, y + j, pen_color);
            }
            else
            {
                draw_point(x + i, y + j, bg_color);
            }
            temp_top >>= 1;
        }

        for(j = 0; 8 > j; j ++)
        {
            if(temp_bottom & 0x01)
            {
                draw_point(x + i, y + j + 8, pen_color);
            }
            else
            {
                draw_point(x + i, y + j + 8, bg_color);
            }
            temp_bottom >>= 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示字符串
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_string(uint16 x, uint16 y, const char dat[])
{
    uint16 j = 0;

    while('\0' != dat[j])
    {
        show_char(x + 8 * j,  y, dat[j]);
        j ++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示有符号整型数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_int(uint16 x, uint16 y, const int32 dat, uint8 num)
{
    int32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12] = {0};

    std::memset(data_buffer, 0, 12);
    std::memset(data_buffer, ' ', num+1);

    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_int_to_str(data_buffer, dat_temp);
    show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示无符号整型数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num)
{
    uint32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12] = {0};
    std::memset(data_buffer, 0, 12);
    std::memset(data_buffer, ' ', num);

    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_uint_to_str(data_buffer, dat_temp);
    show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示浮点数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum)
{
    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17] = {0};
    std::memset(data_buffer, 0, 17);
    std::memset(data_buffer, ' ', num+pointnum+2);

    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    show_string(x, y, data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示灰度图像
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_gray_image(uint16 x, uint16 y, const uint8 *image, uint16 img_width, uint16 img_height)
{
    if (!buffer) return;
    // 简单边界裁剪（可选，此处假设都在范围内）
    for (uint16 row = 0; row < img_height; row++) {
        uint16 *dst_line = buffer + (y + row) * width + x;
        const uint8 *src_line = image + row * img_width;
        for (uint16 col = 0; col < img_width; col++) {
            uint8 gray = src_line[col];
            uint16 r = (gray >> 3) & 0b11111;
            uint16 g = (gray >> 2) & 0b111111;
            uint16 b = (gray >> 3) & 0b11111;
            dst_line[col] = (r << 11) | (g << 5) | (b << 0);
        }
    }
    // 标记整个图像区域为脏
    mark_dirty_rect(x, y, x + img_width - 1, y + img_height - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示RGB565图像
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_rgb_image(uint16 x, uint16 y, const uint16 *image, uint16 img_width, uint16 img_height)
{
    if (!buffer) return;
    for (uint16 row = 0; row < img_height; row++) {
        uint16 *dst_line = buffer + (y + row) * width + x;
        const uint16 *src_line = image + row * img_width;
        memcpy(dst_line, src_line, img_width * sizeof(uint16));
    }
    mark_dirty_rect(x, y, x + img_width - 1, y + img_height - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示屏初始化函数
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::init(const char *path, uint8 is_reload_driver)
{
    struct fb_fix_screeninfo fb_fix;
    struct fb_var_screeninfo fb_var;
    unsigned int screen_size;
    int fd;

    if (is_reload_driver)
    {
        printf("ips200: rmmod fb_st7789v driver ...\n");
        system("rmmod fb_st7789v > /dev/null 2>&1");
        usleep(200*1000);
        printf("ips200: insmod fb_st7789v driver ...\n");
        if(system("insmod /lib/modules/4.19.190/fb_st7789v.ko") != 0)
        {
            perror("insmod fb_st7789v error");
            exit(EXIT_FAILURE);
        }
        usleep(200*1000);
    }

    if (0 > (fd = open(path, O_RDWR))) {
        perror("open error");
        exit(EXIT_FAILURE);
    }

    ioctl(fd, FBIOGET_VSCREENINFO, &fb_var);
    ioctl(fd, FBIOGET_FSCREENINFO, &fb_fix);

    screen_size = fb_fix.line_length * fb_var.yres;
    this->width = fb_var.xres;
    this->height = fb_var.yres;

    screen_base = (unsigned short *)mmap(nullptr, screen_size, PROT_WRITE, MAP_SHARED, fd, 0);
    if (MAP_FAILED == (void *)screen_base) {
        perror("mmap error");
        close(fd);
        exit(EXIT_FAILURE);
    }
    close(fd);  // 映射后可以关闭文件描述符

    // 分配后台缓冲区
    buffer = new uint16[width * height];
    if (!buffer) {
        perror("buffer allocation error");
        exit(EXIT_FAILURE);
    }

    // 清屏（填充背景色到缓冲区并更新显存）
    full(DEFAULT_BGCOLOR);
    update();  // 第一次全屏更新
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新屏幕显示（将脏区域复制到显存）
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::update(void)
{
    if (!buffer || !screen_base) return;
    // 如果没有脏区域，直接返回
    if (dirty_min_x > dirty_max_x || dirty_min_y > dirty_max_y) return;

    int x_start = dirty_min_x;
    int x_end   = dirty_max_x;
    int y_start = dirty_min_y;
    int y_end   = dirty_max_y;
    int line_width = x_end - x_start + 1;
    size_t copy_bytes = line_width * sizeof(uint16);

    for (int row = y_start; row <= y_end; row++) {
        uint16 *src = buffer + row * width + x_start;
        uint16 *dst = screen_base + row * width + x_start;
        memcpy(dst, src, copy_bytes);
    }

    // 重置脏区域
    dirty_min_x = width;
    dirty_min_y = height;
    dirty_max_x = -1;
    dirty_max_y = -1;
}