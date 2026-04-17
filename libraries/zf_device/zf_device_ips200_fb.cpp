#include "zf_device_ips200_fb.hpp"

#if WHETHER_USE_TTF
    #define STB_TRUETYPE_IMPLEMENTATION 
    #include "stb_truetype.h"
#endif

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

    #if WHETHER_USE_TTF
    f_info = nullptr; 
    #endif
}

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

    #if WHETHER_USE_TTF
    if (f_info) {
        delete f_info;
        f_info = nullptr;
    }
    #endif
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

// 画框框
void zf_device_ips200::fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color) {
    if (x >= width || y >= height) return;
    if (x + w > width)  w = width - x;
    if (y + h > height) h = height - y;

    // 2. 核心填充逻辑
    for (uint16 j = 0; j < h; j++) {
        // 计算当前行在 buffer 中的起始指针
        uint16 *ptr = &buffer[(y + j) * width + x];
        
        // 使用循环填充
        // 如果追求极致，这里可以用更底层的 memset 处理（但 uint16 需要特殊处理）
        for (uint16 i = 0; i < w; i++) {
            *ptr++ = color;
        }
    }

    // 3. 标记脏区域：这是为了配合你之前的更新机制
    mark_dirty_rect(x, y, x + w - 1, y + h - 1);
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

    #if WHETHER_USE_TTF
    f_info = new stbtt_fontinfo; 
    if (!stbtt_InitFont(f_info, vector_font_data, 0)) {
        printf("TTF Init Failed!\n");
    } else {
        printf("TTF Font Parser Initialized (Pre-loaded).\n");
    }
    #endif

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

static const char* get_next_utf8_char(const char* s, uint32_t* codepoint) {
    unsigned char c = (unsigned char)*s;
    if (c < 0x80) { // 单字节 (ASCII)
        *codepoint = c;
        return s + 1;
    } else if ((c & 0xE0) == 0xC0) { // 双字节
        *codepoint = ((c & 0x1F) << 6) | (s[1] & 0x3F);
        return s + 2;
    } else if ((c & 0xF0) == 0xE0) { // 三字节 (常用汉字)
        *codepoint = ((c & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F);
        return s + 3;
    } else if ((c & 0xF8) == 0xF0) { // 四字节
        *codepoint = ((c & 0x07) << 18) | ((s[1] & 0x3F) << 12) | ((s[2] & 0x3F) << 6) | (s[3] & 0x3F);
        return s + 4;
    }
    *codepoint = '?'; // 无法识别的编码
    return s + 1;
}


#if WHETHER_USE_TTF
// 引用在 font_data.cpp 中生成的全局数组
// extern unsigned char JetBrainsMono_Bold_ttf[]; 
extern unsigned char vector_font_data[]; 
void zf_device_ips200::render_glyph_to_buffer(int x, int y, unsigned char* bitmap, int bw, int bh, uint16 color) {
    for (int j = 0; j < bh; j++) {
        int screen_y = y + j;
        if (screen_y < 0 || screen_y >= height) continue;

        for (int i = 0; i < bw; i++) {
            int screen_x = x + i;
            if (screen_x < 0 || screen_x >= width) continue;

            uint8 alpha = bitmap[j * bw + i];
            if (alpha == 0) continue;

            if (alpha == 255) {
                draw_point(screen_x, screen_y, color);
            } else {
                uint16 bg = buffer[screen_y * width + screen_x];
                
                // 使用传入的 color 进行精确混合
                uint8 r = ((((color >> 11) & 0x1F) * alpha) + (((bg >> 11) & 0x1F) * (255 - alpha))) >> 8;
                uint8 g = ((((color >> 5) & 0x3F) * alpha) + (((bg >> 5) & 0x3F) * (255 - alpha))) >> 8;
                uint8 b = (((color & 0x1F) * alpha) + ((bg & 0x1F) * (255 - alpha))) >> 8;

                draw_point(screen_x, screen_y, (uint16)((r << 11) | (g << 5) | b));
            }
        }
    }
}

void zf_device_ips200::m_print_internal(uint16 x, uint16 y, uint16 color, float size, const char* text) {
    if (!f_info) return;

    float scale = stbtt_ScaleForPixelHeight(f_info, size);
    int cur_x = x;
    int cur_y = y;
    int ascent, descent, lineGap;
    
    stbtt_GetFontVMetrics(f_info, &ascent, &descent, &lineGap);
    int baseline = (int)(ascent * scale);

    const char* ptr = text;
    while (*ptr != '\0') {
        uint32_t codepoint;
        
        // 1. 处理换行
        if (*ptr == '\n') {
            cur_x = x;
            cur_y += (int)((ascent - descent + lineGap) * scale);
            ptr++;
            continue;
        }

        // 2. 解码获取 Unicode 码点
        ptr = get_next_utf8_char(ptr, &codepoint);

        // 3. 获取字符水平度量
        int advance, lsb;
        stbtt_GetCodepointHMetrics(f_info, codepoint, &advance, &lsb);

        // 4. 获取字符包围盒（计算 bitmap 的宽高）
        int x1, y1, x2, y2;
        stbtt_GetCodepointBitmapBox(f_info, codepoint, scale, scale, &x1, &y1, &x2, &y2);

        int bitmap_w = x2 - x1;
        int bitmap_h = y2 - y1;
        
        // 5. 渲染位图到缓冲区
        if (bitmap_w > 0 && bitmap_h > 0) {
            uint8_t* target_ptr = glyph_render_buf; // 指向你在类里定义的静态数组
            bool is_heap = false;

            // 检查预设 buffer (16384 字节, 对应 128x128 像素) 是否够用
            if (bitmap_w * bitmap_h > 16384) {
                target_ptr = new uint8_t[bitmap_w * bitmap_h];
                is_heap = true;
            }

            // 生成灰度位图
            stbtt_MakeCodepointBitmap(f_info, target_ptr, bitmap_w, bitmap_h, bitmap_w, scale, scale, codepoint);

            // 6. 将位图混合到屏幕 Buffer
            render_glyph_to_buffer(
                cur_x + (int)(lsb * scale), 
                cur_y + baseline + y1, 
                target_ptr, 
                bitmap_w, 
                bitmap_h, 
                color
            );

            // 如果用了堆内存，记得释放
            if (is_heap) delete[] target_ptr;
        }

        // 7. 移动光标位置
        cur_x += (int)(advance * scale);
    }
}
#endif
// 重载 1：使用类默认参数
void zf_device_ips200::print(uint16 x, uint16 y, const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

#if WHETHER_USE_TTF
    this->m_print_internal(x, y, this->pen_color, this->current_font_size, buf);
#else
    this->show_string(x, y, buf);
#endif
}

// 重载 2：使用指定颜色和大小
void zf_device_ips200::print(uint16 x, uint16 y, uint16 color, float size, const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

#if WHETHER_USE_TTF
    this->m_print_internal(x, y, color, size, buf);
#else
    // 如果没有开启 TTF，指定的大小将被忽略，只使用指定的颜色（如果 show_string 支持）
    uint16 old_color = this->pen_color;
    this->pen_color = color;
    this->show_string(x, y, buf);
    this->pen_color = old_color; // 恢复颜色
#endif
}
