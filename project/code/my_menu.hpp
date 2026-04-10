/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/



#ifndef __MY_MENU_HPP__
#define __MY_MENU_HPP__

#include "my_key.hpp"
#include "zf_common_typedef.hpp"
#include "zf_device_ips200_fb.hpp"

// 菜单显示相关宏定义
#define MENU_MAX_ROW 10
#define FONT_H 16
#define FONT_W 8

// 颜色定义（RGB565格式）
#define MENU_BG_COLOR       0xFFFF  // 白色背景
#define MENU_TEXT_COLOR     0x0000  // 黑色文字
#define MENU_SELECT_COLOR   0x001F  // 蓝色选中框

#define IPS_DEVICE_PATH     "/dev/fb0"

// 菜单结构体
typedef struct Menu {
    int id;
    int parent_id;

    const char *name;
    void (*handler)(uint8 action);
} Menu;

// 前向声明
class MyMenu;

extern MyMenu* g_menu_instance;

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