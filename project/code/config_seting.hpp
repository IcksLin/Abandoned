#ifndef _CONFIG_SETTING_HPP_
#define _CONFIG_SETTING_HPP_
/**
 * @file config_seting.hpp
 * @brief 控制器参数缓存、文件读写与 TCP 调参接口声明
 * @note 文件名 seting 为历史拼写，当前 CMake 会直接包含该文件，不建议单独改名。
 */
#include "my_global.hpp"
// ==================== 控制模式标志 ====================
extern uint8_t control_model;  // 0: PID模式, 1: LADRC模式

// ==================== LADRC 右轮参数 ====================
extern float lardc_r_h;
extern float lardc_r_r;
extern float lardc_r_wc;
extern float lardc_r_w0;
extern float lardc_r_b0;
extern float lardc_r_pwm_min;
extern float lardc_r_pwm_max;

// ==================== LADRC 左轮参数 ====================
extern float lardc_l_h;
extern float lardc_l_r;
extern float lardc_l_wc;
extern float lardc_l_w0;
extern float lardc_l_b0;
extern float lardc_l_pwm_min;
extern float lardc_l_pwm_max;

// ==================== PID 右轮参数 ====================
extern float pid_r_kp;
extern float pid_r_ts;
extern float pid_r_ki;
extern float pid_r_kd;
extern float pid_r_error_filter;
extern float pid_r_output_max;
extern float pid_r_output_min;
extern float pid_r_integral_max;
extern float pid_r_integral_min;

// ==================== PID 左轮参数 ====================
extern float pid_l_kp;
extern float pid_l_ts;
extern float pid_l_ki;
extern float pid_l_kd;
extern float pid_l_error_filter;
extern float pid_l_output_max;
extern float pid_l_output_min;
extern float pid_l_integral_max;
extern float pid_l_integral_min;

// ==================== 方向环 PD 参数 ====================
extern float onto_kp;
extern float onto_kp2;
extern float onto_kd;
extern float onto_limit;

//
// ==================== 三串pd参数缓存 ====================
extern float angle_speed_kd;
extern float angle_speed_kp;
extern float onto_control_kp;
extern float onto_control_kd;

// 接收缓存区
extern std::string received;



class TCPClient {
private:
    int sockfd;
    struct sockaddr_in server_addr;
    
public:
    TCPClient() : sockfd(-1) {}
    
    ~TCPClient() {
        if (sockfd >= 0) {
            close(sockfd);
        }
    }
    
    /**
     * @brief 连接上位机 TCP 服务端
     * @param ip 服务端 IP 地址
     * @param port 服务端端口
     * @return true 连接成功，false 连接失败
     */
    bool connect(const std::string& ip, int port) {
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            std::cerr << "Socket creation failed" << std::endl;
            return false;
        }
        
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid address" << std::endl;
            return false;
        }
        
        if (::connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Connection failed" << std::endl;
            return false;
        }
        
        std::cout << "Connected to " << ip << ":" << port << std::endl;
        return true;
    }
    
    /**
     * @brief 发送原始字符串数据
     * @param data 待发送字符串
     */
    void sendData(const std::string& data) {
        if (sockfd >= 0) {
            send(sockfd, data.c_str(), data.length(), 0);
        }
    }
    
    /**
     * @brief 发送函数，语法与printf基本相同
     */
    void sendFormattedData(const char* format, ...) {
        char buffer[512];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        sendData(std::string(buffer));
    }

    /**
     * @brief 接受一行以\n结尾的数据
     * @param line 数据缓存区
     * @param timeout_ms 超时时间
     */
    bool receiveLine(std::string& line, int timeout_ms = 10) {
        static std::string buffer;
        
        if (timeout_ms > 0) {
            // 设置超时
            struct timeval tv;
            tv.tv_sec = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }
        
        char ch;
        while (true) {
            int ret = recv(sockfd, &ch, 1, 0);
            if (ret <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return false;  // 无数据
                }
                return false;  // 连接断开
            }
            
            if (ch == '\n') {
                line = buffer;
                buffer.clear();
                return true;
            } else if (ch != '\r') {
                buffer += ch;
            }
        }
    }

    /**
     * @brief 接收一行数据并原样返回（应答测试）
     * @param timeout_ms 超时时间（毫秒）
     * @return true 接收到数据并应答成功，false 超时或无数据
     */
    bool echoTest(int timeout_ms = 100) {
        std::string received;
        
        if (receiveLine(received, timeout_ms)) {
            std::cout << "收到: " << received << std::endl;
            
            sendFormattedData("%s\n", received.c_str());
            std::cout << "应答: " << received << std::endl;
            
            return true;
        }
        
        return false;
    }

    /**
     * @brief 持续应答测试
     */
    void echoTestLoop() {        
        while (true) {
            if (echoTest(100)) {
                // 成功接收并应答，继续
            }
            usleep(10000);  // 10ms，避免CPU占用过高
        }
    }
};

/**
 * @brief 从配置文件读取参数并写入控制器
 * @param config_file 配置文件路径
 */
void param_loading_from_file(const char* config_file);

/**
 * @brief 将当前缓存参数重新写入控制器对象
 */
void param_setting();

/**
 * @brief 将当前缓存参数写入配置文件
 * @return true 写入成功，false 写入失败
 */
bool write_param_into_file();

/**
 * @brief 打印当前控制器参数
 */
void param_print();

/**
 * @brief 解析上位机传入的一行参数并更新缓存
 * @param line 参数行，格式为 key:value
 * @return true 解析成功，false 未识别或格式错误
 */
bool parseAndUpdateParameter(const std::string& line);

/**
 * @brief 处理 READ 命令，重新从配置文件加载参数
 */
void handleReadCommand();

/**
 * @brief 处理 WRITE 命令，将当前参数写入文件并回传上位机
 * @param client TCP 客户端对象
 * @return true 处理成功，false 写文件失败
 */
bool handleWriteCommand(TCPClient& client);

#endif
