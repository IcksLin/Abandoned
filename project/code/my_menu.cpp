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
    mode3 = {"Config_calibration", 2, static_option_func, nullptr, nullptr, nullptr};
    mode4 = {"Mode4", 3, static_option_func, nullptr, nullptr, nullptr};
    mode5 = {"Mode5", 4, static_option_func, nullptr, nullptr, nullptr};
    
    Pid_Set = {"Set_PID", 5, static_option_func, nullptr, nullptr, nullptr};
    car_pid = {"Car_PID", 6, static_option_func, nullptr, nullptr, nullptr};
    tripod_pid = {"Tripod_PID", 7, static_option_func, nullptr, nullptr, nullptr};
    gray_calibration = {"Gray_Calibration", 8, static_option_func, nullptr, nullptr, nullptr};
    IMU_angle = {"IMU_Angle", 9, static_option_func, nullptr, nullptr, nullptr};
    map_record = {"Map record",10,static_option_func,nullptr,nullptr,nullptr};
    path_reproduction = {"Path reproduction",11,static_option_func,nullptr,nullptr,nullptr};
    brushless_motor_set = {"Brushless_motor_set",12,static_option_func,nullptr,nullptr,nullptr};
    
    current_menu = &main_menu;
}

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
    mode2.child  = &map_record;
    mode2.sibling = &mode3;
    
    mode3.parent = &main_menu;
    mode3.sibling = &mode4;
    mode3.child = &brushless_motor_set;
    
    mode4.parent = &main_menu;
    mode4.sibling = &mode5;
    
    mode5.parent = &main_menu;
    mode5.child = &Pid_Set;
    mode5.sibling = nullptr;
    
    // 设置PID设置菜单
    Pid_Set.parent = &mode5;
    Pid_Set.child = &tripod_pid;
    Pid_Set.sibling = &gray_calibration;
    
    car_pid.parent = &mode3;
    car_pid.sibling = nullptr;
    
    tripod_pid.parent = &mode5;
    tripod_pid.sibling = nullptr;
    
    gray_calibration.parent = &mode5;
    gray_calibration.sibling = &IMU_angle;
    
    // 陀螺仪演示
    IMU_angle.parent = &mode5;
    IMU_angle.sibling = nullptr;

    // 路径录制器
    map_record.parent = &mode2;
    map_record.sibling = &path_reproduction;

    // 路径复现器
    path_reproduction.parent = &mode2;
    path_reproduction.sibling = nullptr;

    //无刷电机校准
    brushless_motor_set.parent = &mode3;
    brushless_motor_set.sibling = &car_pid;
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
    ips_display->update();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 菜单系统主循环
// 参数说明 无
// 返回参数 无
// 使用示例 menu_system.menu_system();
// 备注信息 处理按键输入和菜单显示，需要在主循环中调用
//         当一个菜单下有挂载子菜单后，该菜单对应id的任务函数需要沉默
//         否则无法进入子菜单
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
        case 4:
            menu_mode_4(cl_action);
            break;
        case 9:
            menu_mode_9(cl_action);
            break;
        case 10:
            menu_mode_10(cl_action);
            break;
        case 11:
            menu_mode_11(cl_action);
            break;
        case 12:
            menu_mode_12(cl_action);
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

// 菜单功能函数，命名规范为 menu_mode_X，其中X为菜单项id-------------------------------------------------------------------
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
    // tracking();
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
    // tracking();
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

void MyMenu::menu_mode_6(uint8 cl_action)
{

    // 模式4的具体实现
    switch (cl_action)
    {
    case 0: // 切换键
        break;
    case 1: // +
        break;
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
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
    
    // // 显示IMU角度信息（需要用户提供实际的IMU数据变量）
    // ips_display->clear();
    // ips_display->show_string(0, 0, "IMU Angle");
    // ips_display->show_string(0, 1 * 16, "Roll:");
    // // ips_display->show_float(5 * 8, 1 * 16, Roll_copy, 3, 2);
    // ips_display->show_string(0, 2 * 16, "Pitch:");
    // // ips_display->show_float(5 * 8, 2 * 16, Pitch_copy, 3, 2);
    // ips_display->show_string(0, 3 * 16, "Yaw:");
    // // ips_display->show_float(5 * 8, 3 * 16, Yaw_copy, 3, 2);
    printf("yaw: %f\r", ahrs.getYaw());
}

// 惯性导航地图记录界面
void MyMenu::menu_mode_10(uint8 cl_action) {
    if (!path_tracker_component.is_recording) {
        ips200.show_string(1,1,"1, record");
        ips200.show_string(1,1+1*16,"2, clear_map");
        ips200.show_string(1,1+2*16,"3, insert_map");
        ips200.show_string(1,1+4*16,"4, return");
        ips200.update();
    }

    switch (cl_action) {
        case 0: // 记录开关
            if (!path_tracker_component.is_recording) {
                ahrs.reset(); // 重置姿态，确保航向角从0开始
                system_delay_ms(500);
                path_tracker_component.start_remember();
            } else {
                path_tracker_component.stop_remember(false);
            }
            break;

        case 1: // 清空内存中的地图数据
            path_tracker_component.stop_remember(true);
            break;

        case 2: // 转换、平滑并保存地图
            if(!path_tracker_component.is_recording){
                if (path_tracker_component.current_index > 5) {
                    // 1. 先进行高斯滤波，消除编码器和传感器的原始噪点
                    path_tracker_component.apply_gaussian_filter(1.2f, 5);

                    // 2. 将 PathTracker 中的原始点提取并转换为“里程-坐标”序列
                    std::vector<double> s_list; // 累计里程（自变量）
                    std::vector<double> x_list; // X 坐标
                    std::vector<double> y_list; // Y 坐标
                    double total_s = 0;

                    // 插入起点
                    s_list.push_back(0);
                    x_list.push_back(path_tracker_component.path_array[0].x);
                    y_list.push_back(path_tracker_component.path_array[0].y);

                    // 遍历记录的所有点，计算累计里程 s
                    for (int i = 1; i < path_tracker_component.current_index; i++) {
                        double dx = path_tracker_component.path_array[i].x - path_tracker_component.path_array[i-1].x;
                        double dy = path_tracker_component.path_array[i].y - path_tracker_component.path_array[i-1].y;
                        total_s += std::sqrt(dx * dx + dy * dy);

                        s_list.push_back(total_s);
                        x_list.push_back(path_tracker_component.path_array[i].x);
                        y_list.push_back(path_tracker_component.path_array[i].y);
                    }

                    // 3. 使用 Akima 进行插值计算
                    AkimaInterpolator x_engine, y_engine;
                    if (x_engine.compute(s_list, x_list) && y_engine.compute(s_list, y_list)) {
                        // 4. 等距采样并保存为文件
                        AkimaInterpolator::save_as_tracking_map(
                            x_engine, 
                            y_engine, 
                            total_s, 
                            10, 
                            "tracking_map.txt"
                        );
                        AkimaInterpolator::convert_txt_to_bin("tracking_map.txt","tracking_map.bin");
                        // 可以在屏幕上提示 "Save OK!"
                        printf("------save successful------\n");
                    }
                }
                else
                {
                     ips200.show_string(1,1,"map too short");
                }
                
            }
            break;

        case 3: // 返回键
            // 重置所有状态
            ahrs.reset();
            path_tracker_component.reset();
            
            mode_inter_flag = 0;
            break;
    }
}

// 惯导路径复现
void MyMenu::menu_mode_11(uint8 cl_action){
    if (!path_tracker_component.is_reproduction) {
        ips200.show_string(1,1,"1, reproduction");
        ips200.show_string(1,1+1*16,"2, check_and_load_map");
        ips200.show_string(1,1+2*16,"3, reset_status");
        ips200.show_string(1,1+4*16,"4, return");
        ips200.update();
    }else{
        path_tracker_component.find_closest_index();
        path_tracker_component.get_look_ahead_point(3);
        // path_tracker_component.calculate_target_yaw();
    }

    switch (cl_action)
    {
    case 0: // 开始复现
        // 开始前检查一下内存里的 map 是否为空
        if (path_tracker_component.tracking_map.empty()) {
            printf("Error: Map not loaded. Press 'Confirm' first.\n");
        } else {
            printf("Resuming path following...\n");
            ahrs.reset();
            path_tracker_component.reset();
            // 切换小车状态机到自动驾驶模式

            path_tracker_component.is_reproduction = true;
            
        }
        break;

    case 1: // 确认地图并加载地图
        printf("Checking map file...\n");
        
        // 尝试从磁盘加载二进制地图到内存
        if (path_tracker_component.load_binary_map("tracking_map.bin")) {
            // 加载成功后，打印地图信息
            printf("Map is available!\n");
            printf("Points: %d\n", (int)path_tracker_component.tracking_map.size());
            // 重置搜索起始点，确保从头开始
            path_tracker_component.last_closest_idx = 0;
             
        } else {
            // 加载失败，打印具体原因
            printf("Can't loading map");
        }
        break;

    case 2: // 重置位置，角度，路径点
        path_tracker_component.reset(); // 清除坐标累积
        path_tracker_component.last_closest_idx = 0; // 重置搜索索引
        ahrs.reset(); //解算器重置

        printf("Position and Index Reset Done.\n");
        break;

    case 3: // 返回键
        //清除所有状态
        path_tracker_component.reset();
        mode_inter_flag = 0; // 返回菜单
        break;

    default:
        break;
    }
}

// 无刷电调校准菜单
void MyMenu::menu_mode_12(uint8 cl_action)
{
    switch (cl_action)
    {
    case 0: 
        esc_set_power(100);
        break;
    case 1:
        esc_set_power(0);
    case 3: // 返回键
        mode_inter_flag = 0; // 返回菜单
        esc_set_power(0);    // 安全处理，关闭无刷电机
        break;
    default:
        break;
    }
    ips200.show_string(1,1,"1.full power");
    ips200.show_string(1,1+16*1,"2.min power");
    ips200.update();

}