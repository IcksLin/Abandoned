#ifndef __ZF_DEVICE_IPS200_FB_HPP__
#define __ZF_DEVICE_IPS200_FB_HPP__

#include "zf_common_typedef.hpp"
#include <cstring>      ///< 用于 memcpy
#include <algorithm>    ///< 用于 std::min/std::max (可选)

#define DEFAULT_PENCOLOR         (RGB565_RED)    ///< 默认画笔颜色
#define DEFAULT_BGCOLOR          (RGB565_WHITE)  ///< 默认背景颜色

#define FB_PATH                 "/dev/fb0"       ///< framebuffer 设备节点默认路径

#ifndef WHETHER_USE_TTF
    #define WHETHER_USE_TTF          0  
#endif

#if WHETHER_USE_TTF
    #include "stb_truetype.h"
    extern unsigned char vector_font_data[];
    struct stbtt_fontinfo;
#endif

/**
 * @class zf_device_ips200
 * @brief IPS200 显示屏 framebuffer 驱动 C++ 封装类
 *
 * 分辨率：240x320，RGB565 格式。
 * 适用平台：LS2K0300。
 * 所有成员函数均无 ips200 前缀，封装性强，调用简洁。
 */
class zf_device_ips200
{
private:
    int width;                       ///< 屏幕宽度
    int height;                      ///< 屏幕高度
    unsigned short *screen_base;     ///< 映射后的显存基地址
    unsigned short *buffer;          ///< 后台缓冲区 (大小 = width * height * 2)
    int dirty_min_x, dirty_min_y;    ///< 脏区域左上角坐标
    int dirty_max_x, dirty_max_y;    ///< 脏区域右下角坐标

    /**
     * @brief 标记单个像素点为脏
     * @param x X 坐标
     * @param y Y 坐标
     */
    inline void mark_dirty_point(int x, int y) {
        if (x < dirty_min_x) dirty_min_x = x;
        if (x > dirty_max_x) dirty_max_x = x;
        if (y < dirty_min_y) dirty_min_y = y;
        if (y > dirty_max_y) dirty_max_y = y;
    }

    /**
     * @brief 标记矩形区域为脏
     * @param x1 矩形左边界
     * @param y1 矩形上边界
     * @param x2 矩形右边界
     * @param y2 矩形下边界
     */
    inline void mark_dirty_rect(int x1, int y1, int x2, int y2) {
        if (x1 < dirty_min_x) dirty_min_x = x1;
        if (x2 > dirty_max_x) dirty_max_x = x2;
        if (y1 < dirty_min_y) dirty_min_y = y1;
        if (y2 > dirty_max_y) dirty_max_y = y2;
    }

    // 仅在需要使用字库时链接字库
    #if WHETHER_USE_TTF
        stbtt_fontinfo* f_info = nullptr;
        /**
         * @brief 将字形位图渲染到后台缓冲区
         * @param x 起始 X 坐标
         * @param y 起始 Y 坐标
         * @param bitmap 字形位图数据
         * @param bw 位图宽度
         * @param bh 位图高度
         * @param color RGB565 颜色值
         */
        void render_glyph_to_buffer(int x, int y, unsigned char* bitmap, int bw, int bh, uint16 color);
        /**
         * @brief 核心文本打印逻辑
         * @param x 起始 X 坐标
         * @param y 起始 Y 坐标
         * @param color RGB565 颜色值
         * @param size 字体大小 (像素高度)
         * @param text 待打印的字符串
         */
        void m_print_internal(uint16 x, uint16 y, uint16 color, float size, const char* text);
        alignas(16) uint8_t glyph_render_buf[16384];  ///< 字形渲染缓冲区
    #endif

public:
    uint16 pen_color{0xFFFF};   ///< 当前画笔颜色 (默认为 RGB565_RED)
    uint16 bg_color{0x0000};    ///< 当前背景颜色 (默认为 RGB565_WHITE)
    #if WHETHER_USE_TTF
        float current_font_size = 24.0f; ///< 使用 TTF 时的默认字号
    #endif

    /**
     * @brief 默认构造函数
     *
     * 初始化默认画笔与背景颜色，将显存指针置空，不分配缓冲区，并将脏区域设为无效。
     */
    zf_device_ips200(void);

    /**
     * @brief 析构函数
     *
     * 释放后台缓冲区并解除显存映射。
     */
    ~zf_device_ips200();

    /** @return 屏幕宽度 (像素) */
    int get_width() const { return width; }
    /** @return 屏幕高度 (像素) */
    int get_height() const { return height; }

    /**
     * @brief 使用当前背景颜色清屏
     */
    void clear(void);

    /**
     * @brief 使用单一颜色填充整个屏幕
     * @param color RGB565 格式颜色值 (例如 RGB565_BLACK)
     */
    void full(const uint16 color);

    /**
     * @brief 绘制单个像素点
     * @param x X 坐标 [0, width-1]
     * @param y Y 坐标 [0, height-1]
     * @param color RGB565 格式颜色值
     */
    void draw_point(uint16 x, uint16 y, const uint16 color);

    /**
     * @brief 绘制直线
     * @param x_start 起点 X 坐标 [0, width-1]
     * @param y_start 起点 Y 坐标 [0, height-1]
     * @param x_end   终点 X 坐标 [0, width-1]
     * @param y_end   终点 Y 坐标 [0, height-1]
     * @param color   RGB565 格式颜色值
     */
    void draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);

    /**
     * @brief 填充实心矩形
     * @param x     矩形左上角 X 坐标
     * @param y     矩形左上角 Y 坐标
     * @param w     矩形宽度
     * @param h     矩形高度
     * @param color RGB565 填充颜色
     */
    void fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);

    /**
     * @brief 显示单个 ASCII 字符 (8x16 点阵)
     * @param x   起始 X 坐标 [0, width-1]
     * @param y   起始 Y 坐标 [0, height-1]
     * @param dat 待显示字符 (ASCII 32~127)
     */
    void show_char(uint16 x, uint16 y, const char dat);

    /**
     * @brief 显示 ASCII 字符串 (8x16 点阵)
     * @param x   起始 X 坐标 [0, width-1]
     * @param y   起始 Y 坐标 [0, height-1]
     * @param dat 待显示字符串 (以 '\0' 结尾)
     */
    void show_string(uint16 x, uint16 y, const char dat[]);

    /**
     * @brief 使用类默认设置打印格式化文本
     * @param x   起始 X 坐标
     * @param y   起始 Y 坐标 (对于 TTF 为行顶坐标，内部自动处理基线偏移)
     * @param fmt printf 风格格式化字符串
     * @param ... 可变参数列表
     * @note 使用当前的 pen_color 和 current_font_size 成员变量
     */
    void print(uint16 x, uint16 y, const char* fmt, ...);

    /**
     * @brief 显式指定颜色和大小的样式打印函数
     * @param x     起始 X 坐标
     * @param y     起始 Y 坐标
     * @param color RGB565 格式文字颜色 (如 RGB565_RED 或 0xF800)
     * @param size  TTF 字体大小 (像素高度，如 24.0f)
     * @param fmt   printf 风格格式化字符串
     * @param ...   可变参数列表
     * @note 本次调用不会改变类的全局状态 (pen_color 和 current_font_size)
     */
    void print(uint16 x, uint16 y, uint16 color, float size, const char* fmt, ...);

    /**
     * @brief 显示灰度图像
     * @param x      起始 X 坐标 [0, width-1]
     * @param y      起始 Y 坐标 [0, height-1]
     * @param image  8 位灰度图像数据首地址
     * @param width  图像宽度 (像素)
     * @param height 图像高度 (像素)
     * @note 灰度值自动转换为 RGB565 格式显示
     */
    void show_gray_image(uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height);

    /**
     * @brief 显示 RGB565 格式图像
     * @param x      起始 X 坐标 [0, width-1]
     * @param y      起始 Y 坐标 [0, height-1]
     * @param image  RGB565 图像数据首地址
     * @param width  图像宽度 (像素)
     * @param height 图像高度 (像素)
     * @note 直接写入 RGB565 数据到缓冲区，无逐点转换，速度更快
     */
    void show_rgb_image(uint16 x, uint16 y, const uint16 *image, uint16 width, uint16 height);

    /**
     * @brief 显示屏初始化函数
     * @param path            framebuffer 设备节点路径 (默认为 "/dev/fb0")
     * @param is_reload_driver 是否重载驱动 (1 表示是，0 表示否)
     * @note 打开 fb 设备、获取屏幕参数、映射显存、分配缓冲区并初始化清屏。程序初始化阶段仅需调用一次。
     */
    void init(const char *path = FB_PATH, uint8 is_reload_driver = 1);

    /**
     * @brief 更新屏幕显示
     * @note 将后台缓冲区中的脏区域数据批量复制到显存，并重置脏区域。应在所有绘制操作完成后调用。
     */
    void update(void);
};

#endif // __ZF_DEVICE_IPS200_FB_HPP__