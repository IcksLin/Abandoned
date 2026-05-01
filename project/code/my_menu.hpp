/*********************************************************************************************************************
* 文件名称          my_menu
* 功能说明          屏幕菜单系统、菜单导航和功能页面回调管理
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-05-01        Assistant                  修正文件头并补充说明
********************************************************************************************************************/



#ifndef __MY_MENU_HPP__
#define __MY_MENU_HPP__

#include "my_key.hpp"
#include "zf_common_typedef.hpp"
#include "zf_device_ips200_fb.hpp"

// 菜单显示相关宏定义
#define MENU_MAX_ROW              5
#define ITEM_H                    30      ///< TTF 模式下 24 号字的推荐行高，预留上下间距
#define MENU_FONT_SIZE            24.0f   ///< TTF 模式下的菜单字号
#define MENU_STATIC_FONT_WIDTH    8       ///< 静态 ASCII 字库单字符宽度
#define MENU_STATIC_FONT_HEIGHT   16      ///< 静态 ASCII 字库单字符高度

// 颜色定义（RGB565 格式）。整体采用偏暖色调，避免菜单界面过冷或刺眼。
#define MENU_BG_COLOR             0xFF9C  ///< 暖白背景
#define MENU_TEXT_COLOR           0x4228  ///< 深棕文字
#define MENU_SELECT_COLOR         0xF4A5  ///< 暖橙选中框背景
#define MENU_SELECT_TEXT          0xFFFF  ///< 选中项文字颜色
#define MENU_DIVIDER_COLOR        0xDEDB  ///< 浅暖灰分隔线

#define IPS_DEVICE_PATH     "/dev/fb0"

/**
 * @brief 单个菜单项定义
 */
typedef struct Menu {
    int id;             ///< 菜单项唯一 ID
    int parent_id;      ///< 父菜单 ID，-1 表示根节点

    const char *name;               ///< 菜单显示名称
    void (*handler)(uint8 action);  ///< 叶子节点功能回调，非叶子节点一般为空
} Menu;

// 前向声明
class MyMenu;

extern MyMenu* g_menu_instance;

/**
 * @brief 按键驱动的 IPS 菜单系统
 * @details 负责菜单层级导航、页面绘制和叶子功能回调调度。
 */
class MyMenu {
private:
    MyKey* key_manager;           // 按键管理器
    Menu* current_menu;           // 当前菜单指针
    zf_device_ips200* ips_display; // IPS显示屏对象指针
    uint8 mode_inter_flag;        // 模式交互标志位

    // 菜单项定义
    static Menu menu_table[];
    static const int MENU_TABLE_SIZE;

    MyMenu(const MyMenu&) = delete;
    MyMenu& operator=(const MyMenu&) = delete;

    // 根据 ID 查找菜单项指针
    Menu* find_menu_by_id(int id);

    Menu* get_first_child(int parent_id);

    Menu* menu_navigate(Menu *current, MenuAction action);

    /**
     * @brief 使用 TTF 动态字库绘制菜单页面
     * @param selected_menu 当前选中的菜单项
     * @note 仅在 WHETHER_USE_TTF 为 1 时由 draw_menu 调用。
     */
    void draw_menu_ttf(Menu *selected_menu);

    /**
     * @brief 使用 8x16 静态 ASCII 字库绘制菜单页面
     * @param selected_menu 当前选中的菜单项
     * @note 仅在 WHETHER_USE_TTF 为 0 时由 draw_menu 调用；菜单名称需保持 ASCII 字符。
     */
    void draw_menu_static(Menu *selected_menu);

public:
    MyMenu(MyKey* key_mgr, zf_device_ips200* ips_disp);

    void init();
    //-------------------------------------------------------------------------------------------------------------------
    // 函数简介 菜单系统主循环
    // 参数说明 无
    // 返回参数 无
    // 使用示例 menu_system.menu_system();
    // 备注信息 处理按键输入和菜单显示，需要在主循环中调用
    //-------------------------------------------------------------------------------------------------------------------
    void menu_system(void);

    void draw_menu(Menu *selected_menu);

    Menu* get_current_menu(void);

    //-------------------------------------------------------------------------------------------------------------------
    // 函数简介 设置模式交互标志
    // 参数说明 flag 标志值
    // 返回参数 无
    // 使用示例 menu_system.set_mode_inter_flag(1);
    // 备注信息 设置是否进入模式交互状态
    //-------------------------------------------------------------------------------------------------------------------
    void set_mode_inter_flag(uint8 flag);

    //-------------------------------------------------------------------------------------------------------------------
    // 函数简介 获取模式交互标志
    // 参数说明 无
    // 返回参数 uint8 当前标志值
    // 使用示例 uint8 flag = menu_system.get_mode_inter_flag();
    // 备注信息 获取当前模式交互状态
    //-------------------------------------------------------------------------------------------------------------------
    uint8 get_mode_inter_flag(void);

    // 菜单回调函数声明
    static void menu_mode_0(uint8 cl_action);
    static void menu_mode_1(uint8 cl_action);
    static void menu_mode_2(uint8 cl_action);
    static void menu_mode_3(uint8 cl_action);
    static void key_remap_test(uint8 cl_action);
    static void menu_mode_6(uint8 cl_action);
    static void imu_angle_display(uint8 cl_action);
    static void get_map(uint8 cl_action);
    static void tracking_by_map(uint8 cl_action);
    static void brushless_calibration(uint8 cl_action);
    static void option_func(void);
    
};

#endif
