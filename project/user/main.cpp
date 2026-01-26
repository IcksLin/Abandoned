
/*********************************************************************************************************************
 * LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是LS2K0300 开源库的一部分
 *
 * LS2K0300 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 适用平台          LS2K0300
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者           备注
 * 2025-12-27        大W            first version
 * 2025-01-20        Assistant      添加编码器功能和优化显示
 ********************************************************************************************************************/

#include "my_global.hpp"

static volatile sig_atomic_t g_stop = 0;
void handle_sigint(int)
{
    g_stop = 1;
}

void camera_task_func()
{
    send_picture_to_Serve();
}

int main()
{

    // 1. 初始化显示屏
    printf("1. 初始化显示屏...\n");
    ips200.init("/dev/fb0");
    ips200.clear();
    ips200.show_string(10, 10, "Camera & Encoder");
    ips200.show_string(10, 30, "System Ready");
    system_delay_ms(1000);

    // 2. 初始化编码器（清零采样值）
    printf("2. 初始化编码器...\n");
    encoder1.clear_count();
    encoder2.clear_count();
    printf("编码器已清零\n");
    system_delay_ms(100);

    // 3.初始化DRV驱动
    printf("3. 初始化DRV驱动...\n");
    motor_init();
    left_speed = 0;
    right_speed = 0;
    motor_set_speed(left_speed, right_speed);

    // 4. 初始化摄像头
    printf("4. 初始化摄像头...\n");
    // zf_device_uvc uvc;
    uvc.init("/dev/video0");
    printf("4.1. HTTP图像传输服务器初始化...\n");
    if (camera_server.start_server(8080) != 0) {
        printf("HTTP Server启动失败\n");
    } else {
        printf("HTTP Server启动成功，端口: 8080\n");
    }

    // 5. 初始化PID控制器
    printf("5. 初始化PID控制器...\n");
    pid_r.init(1.5, 0.03, 0.1, 1, 120, -120, 60, -60);
    pid_l.init(1.5, 0.03, 0.1, 1, 120, -120, 60, -60);

    // 6. 初始化菜单系统
    printf("6. 初始化菜单系统...\n");
    menu_system.init_menu();

    // 初始化线程，可开始参数获取任务调度
    zf_driver_pit_rt key_scan;
    zf_driver_pit_rt encoder_get;
    zf_driver_pit_rt pid_control_thread;
    zf_driver_pit_rt camera_thread;

    if (key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 98, true) != 0)
    {
        printf("定时器初始化失败\n");
        return -1;
    }
    else
    {
        printf("key scaning thread init successfully,period: %dms\n", KEY_SCAN_PERIOD);
    }

    if (encoder_get.init_ms(ENCODER_SAMPLING_PERIOD, encoder_get_count_handler, 99, true) != 0)
    {
        printf("编码器获取线程初始化失败\n");
        return -1;
    }
    else
    {
        printf("encoder geting count thread init successfully,period: %dms\n", ENCODER_SAMPLING_PERIOD);
        my_timer.start(); // 启动时间戳检测
    }

    if (pid_control_thread.init_ms(PID_CONTROL_PERIOD, pid_contol_handle, 97, true) != 0)
    {
        printf("PID控制器线程初始化失败");
        return -1;
    }
    else
    {
        printf("pid control thread init successfully,period:%dms\n", PID_CONTROL_PERIOD);
    }

    if (camera_thread.init_ms(30, camera_task_func, 96, true) != 0)
    {
        printf("Camera thread init failed\n");
    }
    else
    {
        printf("Camera thread init successfully, period: 30ms\n");
    }

    // 主循环，运行菜单系统
    while (true)
    {
        menu_system.menu_system();
        system_delay_ms(20);
    }
    return 0;
}