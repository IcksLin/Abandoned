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

        client.sendFormattedData("Onto_control,yaw,speed_l,speed_r,l_pwm,r_pwm:%f,%f,%f,%f,%d,%d\r\n",onto_control,ahrs.getYaw(),left_speed,right_speed,speed_to_pwm_l,speed_to_pwm_r);
        // menu_system.menu_system();
        system_delay_ms(10);
    }

    return 0;
}

void get_b0(){
    static uint64_t tick_count = 0;
    // 用于暂存速度数据，避免在循环内频繁操作文件IO导致时序偏移
    static std::vector<float> speed_data_left;
    static std::vector<float> speed_data_right;
    speed_data_left.reserve(300);
    speed_data_right.reserve(300);
    std::cout << "Starting b0 Test in 2 seconds..." << std::endl;
    system_delay_ms(2000);
    while (tick_count < 300) {
        // 1. 给定阶跃信号
        motor_set_speed_ladrc(500, 500);

        // 2. 记录当前真实速度
        speed_data_left.push_back(left_speed);
        speed_data_right.push_back(right_speed);

        // 3. 严格控制采样周期 (10ms)
        system_delay_ms(10);
        
        tick_count++;
    }

    std::ofstream outFile("speed_linear.txt");
    if (outFile.is_open()) {
        outFile << "Tick,Time(s),LeftSpeed,RightSpeed,U\n";
        for (size_t i = 0; i < speed_data_left.size(); ++i) {
            outFile << i << "," << i * 0.01f << "," 
                    << speed_data_left[i] << "," 
                    << speed_data_right[i] << "," 
                    << 500 << "\n";
        }
        outFile.close();
        std::cout << "Test complete. Data saved to speed_linear.txt" << std::endl;
    } else {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }

}