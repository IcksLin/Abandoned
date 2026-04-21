#include "my_global.hpp"

int main()
{
    car_init();
    TCPClient client;
    if (!client.connect("192.168.43.94", 8086)) {
        return -1;
    }
    onto_pd_control_enable = 1;
    
    // if (key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 95, true) != 0)
    // {
    //     printf("定时器初始化失败\n");
    //     return false;
    // }
    // else
    // {
    //     printf("key scaning thread init successfully,period: %dms\n", KEY_SCAN_PERIOD);
    // }

    // menu_system.init();


    while (true) {
        my_timer.start();
        if (client.receiveLine(received, 100)) {
            // 去除末尾换行符
            received.erase(received.find_last_not_of("\r\n") + 1);
            
            printf("收到: %s\n", received.c_str());
            
            if (received == "WRITE") {
                handleWriteCommand(client);
            }
            else if (received == "READ") {
                handleReadCommand();
            }
            else if(received == "turn flag"){
                onto_pd_control_enable = onto_pd_control_enable ? 0 : 1;
                printf("onto control flag: %d  \n",onto_pd_control_enable);
            }
            else {
                // 尝试解析参数更新
                parseAndUpdateParameter(received);
            }
        }
        tracking();

        client.sendFormattedData("Onto_control,yaw,angle_speed_pd,onto_control_pd,l_pwm,r_pwm,angle_speed:%f,%f,%f,%f,%d,%d,%f\r\n",
        onto,ahrs.getYaw(),angle_speed_pd.control(onto_control_pd.control(0,onto),cur_angle_speed),onto_control_pd.control(0,onto)
            ,speed_to_pwm_l,speed_to_pwm_r,cur_angle_speed);
        // menu_system.menu_system();
        // system_delay_ms(10);
        my_timer.stop();
        printf("onto:   %f     ,middle_line_length: %d   ,time:  %lld \r",onto,middle_line_length,my_timer.elapsed_us());
    }

    return 0;
}
