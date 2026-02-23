/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/


#include "my_menu.hpp"
#include "my_task_function.hpp"

// 全局菜单实例指针
MyMenu* g_menu_instance = nullptr;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 key_mgr 按键管理器指针
// 参数说明 ips_disp IPS显示屏对象指针
// 返回参数 无
// 使用示例 MyMenu menu_system(&key_manager, &ips200);
// 备注信息 初始化菜单系统和菜单结构
//-------------------------------------------------------------------------------------------------------------------
MyMenu::MyMenu(MyKey* key_mgr, zf_device_ips200* ips_disp) : key_manager(key_mgr), ips_display(ips_disp), mode_inter_flag(0)
{
    // 设置全局实例指针
    g_menu_instance = this;
    
    // 初始化菜单项
    main_menu = {"Main Menu", -1, nullptr, nullptr, nullptr, nullptr};
    
    mode1 = {"Run By Image", 0, static_option_func, nullptr, nullptr, nullptr};
    mode2 = {"Inertial navigation", 1, static_option_func, nullptr, nullptr, nullptr};
    mode3 = {"Mode3", 2, static_option_func, nullptr, nullptr, nullptr};
    mode4 = {"Mode4", 3, static_option_func, nullptr, nullptr, nullptr};
    mode5 = {"Mode5", 4, static_option_func, nullptr, nullptr, nullptr};
    
    Pid_Set = {"Set_PID", 5, static_option_func, nullptr, nullptr, nullptr};
    car_pid = {"Car_PID", 6, static_option_func, nullptr, nullptr, nullptr};
    tripod_pid = {"Tripod_PID", 7, static_option_func, nullptr, nullptr, nullptr};
    gray_calibration = {"Gray_Calibration", 8, static_option_func, nullptr, nullptr, nullptr};
    IMU_angle = {"IMU_Angle", 9, static_option_func, nullptr, nullptr, nullptr};
    map_record = {"Map record",10,static_option_func,nullptr,nullptr,nullptr};

    current_menu = &main_menu;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用
// 备注信息 清理资源
//-------------------------------------------------------------------------------------------------------------------
MyMenu::~MyMenu()
{
    // 清除全局实例指针
    if(g_menu_instance == this)
    {
        g_menu_instance = nullptr;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 初始化菜单父子关系
// 参数说明 无
// 返回参数 无
// 使用示例 内部使用
// 备注信息 设置每个菜单项的parent指针
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::init_menu_parents(void)
{
    // 设置主菜单的子菜单链表
    main_menu.child = &mode1;
    
    // 设置模式菜单的兄弟关系和父菜单
    mode1.parent = &main_menu;
    mode1.sibling = &mode2;
    
    mode2.parent = &main_menu;
    mode2.sibling = &mode3;
    
    mode3.parent = &main_menu;
    mode3.sibling = &mode4;
    
    mode4.parent = &main_menu;
    mode4.sibling = &mode5;
    
    mode5.parent = &main_menu;
    mode5.child = &Pid_Set;
    mode5.sibling = nullptr;
    
    // 设置PID设置菜单
    Pid_Set.parent = &mode5;
    Pid_Set.child = &car_pid;
    Pid_Set.sibling = &gray_calibration;
    
    car_pid.parent = &Pid_Set;
    car_pid.sibling = &tripod_pid;
    
    tripod_pid.parent = &Pid_Set;
    tripod_pid.sibling = nullptr;
    
    gray_calibration.parent = &mode5;
    gray_calibration.sibling = &IMU_angle;
    
    IMU_angle.parent = &mode5;
    IMU_angle.sibling = nullptr;

    map_record.parent = &mode2;
    map_record.sibling = nullptr;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取同级菜单的上一个
// 参数说明 current 当前菜单项
// 返回参数 Menu* 上一个同级菜单项指针
// 使用示例 内部使用
// 备注信息 从parent的child链表遍历查找
//-------------------------------------------------------------------------------------------------------------------
Menu* MyMenu::menu_get_prev_sibling(Menu *current)
{
    if (!current || !current->parent || current->parent->child == current)
        return nullptr;
        
    Menu *p = current->parent->child;
    while (p && p->sibling != current)
    {
        p = p->sibling;
    }
    return p;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 菜单导航函数
// 参数说明 current 当前菜单项
// 参数说明 action 菜单动作
// 返回参数 Menu* 导航后的菜单项指针
// 使用示例 内部使用
// 备注信息 根据动作进行菜单导航
//-------------------------------------------------------------------------------------------------------------------
Menu* MyMenu::menu_navigate(Menu *current, MenuAction action)
{
    if (!current)
        return nullptr;

    switch (action)
    {
    case MENU_UP:
    {
        Menu *prev = menu_get_prev_sibling(current);
        if (prev)
            return prev;
        break;
    }
    case MENU_DOWN:
        if (current->sibling)
            return current->sibling;
        break;
    case MENU_OK:
        if (current->child)
        {
            return current->child;
        }
        else
        {
            if (current->func)
                current->func();
            return current;
        }
        break;
    case MENU_BACK:
        if (current->parent)
            return current->parent;
        break;
    default:
        return current;
    }

    // 若无可跳转项，返回自身
    return current;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 初始化菜单系统
// 参数说明 无
// 返回参数 无
// 使用示例 menu_system.init_menu();
// 备注信息 初始化菜单结构体和显示
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::init_menu(void)
{
    // 初始化菜单结构体
    init_menu_parents();
    current_menu = &main_menu;
    ips_display->clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 绘制菜单显示
// 参数说明 selected_menu 选中的菜单项
// 返回参数 无
// 使用示例 内部使用或外部调用显示
// 备注信息 在IPS屏幕上绘制菜单界面
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::draw_menu(Menu *selected_menu)
{
    // 1. 找到同级菜单链表头
    Menu *head = selected_menu;
    while (head->parent && head->parent->child && head->parent->child != head)
    {
        head = head->parent->child;
    }

    // 2. 遍历同级链表，统计总数和选中项序号
    int total = 0;
    int selected_index = -1;
    Menu *iter = head;
    while (iter)
    {
        if (iter == selected_menu)
            selected_index = total;
        total++;
        iter = iter->sibling;
    }

    // 3. 计算分页起点
    uint8 page_start = 0;
    if (selected_index >= MENU_MAX_ROW)
        page_start = selected_index - (MENU_MAX_ROW - 1);

    // 4. 定位到page_start
    iter = head;
    for (int i = 0; i < page_start && iter; ++i)
        iter = iter->sibling;

    // 5. 清屏并显示菜单项
    ips_display->clear();
    
    for (int row = 0; row < MENU_MAX_ROW && iter; row++)
    {
        uint16 y = row * FONT_H;
        
        // 如果是选中项，绘制选中框
        if ((page_start + row) == selected_index)
        {
            // 绘制选中框背景
            for(uint16 i = 0; i < FONT_W * 20; i++) // 假设菜单项最长20个字符
            {
                for(uint16 j = 0; j < FONT_H; j++)
                {
                    ips_display->draw_point(i, y + j, MENU_SELECT_COLOR);
                }
            }
            // 显示选中项文字（白色）
            ips_display->show_string(2, y + 2, iter->name);
        }
        else
        {
            // 显示普通菜单项（黑色文字）
            ips_display->show_string(2, y + 2, iter->name);
        }
        
        iter = iter->sibling;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 菜单系统主循环
// 参数说明 无
// 返回参数 无
// 使用示例 menu_system.menu_system();
// 备注信息 处理按键输入和菜单显示，需要在主循环中调用
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::menu_system(void)
{
    static uint8 cl_action = 0;
    cl_action = key_manager->get_menu_action(); // 获取菜单动作

    if (mode_inter_flag == 0)
    {
        current_menu = menu_navigate(current_menu, (MenuAction)cl_action);
        draw_menu(current_menu);
    }
    else
    {
        // 处理菜单回调函数
        switch (current_menu->id)
        {
        case 0:
            menu_mode_0(cl_action);
            break;
        case 1:
            menu_mode_1(cl_action);
            break;
        case 2:
            menu_mode_2(cl_action);
            break;
        case 3:
            menu_mode_3(cl_action);
            break;
        case 4:
            menu_mode_4(cl_action);
            break;
        case 9:
            menu_mode_9(cl_action);
            break;
        default:
            mode_inter_flag = 0; // 重置标志
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前菜单
// 参数说明 无
// 返回参数 Menu* 当前菜单指针
// 使用示例 Menu* current = menu_system.get_current_menu();
// 备注信息 获取当前选中的菜单项
//-------------------------------------------------------------------------------------------------------------------
Menu* MyMenu::get_current_menu(void)
{
    return current_menu;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置模式交互标志
// 参数说明 flag 标志值
// 返回参数 无
// 使用示例 menu_system.set_mode_inter_flag(1);
// 备注信息 设置是否进入模式交互状态
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::set_mode_inter_flag(uint8 flag)
{
    mode_inter_flag = flag;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取模式交互标志
// 参数说明 无
// 返回参数 uint8 当前标志值
// 使用示例 uint8 flag = menu_system.get_mode_inter_flag();
// 备注信息 获取当前模式交互状态
//-------------------------------------------------------------------------------------------------------------------
uint8 MyMenu::get_mode_inter_flag(void)
{
    return mode_inter_flag;
}

// 菜单回调函数实现（示例）
void MyMenu::option_func(void)
{
    if (mode_inter_flag == 0)
        mode_inter_flag = 1;
    return;
}

// 静态包装函数
void MyMenu::static_option_func(void)
{
    if(g_menu_instance != nullptr)
    {
        g_menu_instance->option_func();
    }
}

void MyMenu::menu_mode_0(uint8 cl_action)
{
    // 模式0的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式0
        break;
    case 1: // 上键
        break;
    case 2: // 下键
        break;
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式0信息
    // show_all_of_the_component_without_ips();
    tracking();
}

void MyMenu::menu_mode_1(uint8 cl_action)
{
    // 模式1的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式1
        break;
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式1信息
    // send_picture_to_Serve();
    tracking();
}

void MyMenu::menu_mode_2(uint8 cl_action)
{
    // 模式2的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式2
        break;
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式2信息
    tracking();
}

void MyMenu::menu_mode_3(uint8 cl_action)
{
    // 模式3的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式3
        break;
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式3信息
    ips_display->clear();
    ips_display->show_string(0, 0, "Mode 3 Active");
}

void MyMenu::menu_mode_4(uint8 cl_action)
{
    // 模式4的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式4
        break;
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式4信息
    ips_display->clear();
    ips_display->show_string(0, 0, "Mode 4 Active");
}

void MyMenu::menu_mode_9(uint8 cl_action)
{
    // IMU角度显示模式
    switch (cl_action)
    {
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示IMU角度信息（需要用户提供实际的IMU数据变量）
    ips_display->clear();
    ips_display->show_string(0, 0, "IMU Angle");
    ips_display->show_string(0, 1 * 16, "Roll:");
    // ips_display->show_float(5 * 8, 1 * 16, Roll_copy, 3, 2);
    ips_display->show_string(0, 2 * 16, "Pitch:");
    // ips_display->show_float(5 * 8, 2 * 16, Pitch_copy, 3, 2);
    ips_display->show_string(0, 3 * 16, "Yaw:");
    // ips_display->show_float(5 * 8, 3 * 16, Yaw_copy, 3, 2);
}

//惯性导航地图记录界面
void MyMenu::menu_mode_10(uint8 cl_action){
    //记录期间静默显示屏，全力确保编码器正常
    if(!path_tracker_component.is_recording){}

    // IMU角度显示模式
    switch (cl_action)
    {
    case 0://记录开关

        
    break;

    case 1://清空地图

    break;

    case 2://转换并保存地图

    break;

    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
}