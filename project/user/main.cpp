#include "my_global.hpp"

int main()
{
    car_init();
    TCPClient client;
    if (!client.connect("192.168.43.94", 8086)) {
        return -1;
    }
    onto_pd_control_enable = 1;
    while (true) {
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
            else {
                // 尝试解析参数更新
                parseAndUpdateParameter(received);
            }
        }

        // client.sendFormattedData("Onto_control,yaw,speed_l,speed_r,l_pwm,r_pwm:%f,%f,%f,%f,%d,%d\r\n",onto_control,ahrs.getYaw(),left_speed,right_speed,speed_to_pwm_l,speed_to_pwm_r);
        // menu_system.menu_system();
        system_delay_ms(10);
    }

    return 0;
}
