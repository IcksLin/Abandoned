# LS2K0300开源库 完整接口文档

**项目名称**：逐飞LS2K0300开源库  
**平台**：LS2K0300 龙芯处理器  
**操作系统**：Ubuntu 24.04  
**交叉编译器**：loongson-gnu-toolchain-8.3  
**文档生成日期**：2026年1月25日

---

## 📚 文档导航

### 快速入门
- [项目架构概览](#项目架构概览) - 了解库的整体结构
- [完整应用示例](#完整应用示例) - 基于实际项目的代码示例

### API接口手册
- [公共层接口](#公共层接口) - FIFO、字体、通用函数
- [驱动层接口](#驱动层接口) - GPIO、PWM、ADC、编码器、网络等
- [设备层接口](#设备层接口) - IMU、屏幕、摄像头、测距传感器
- [组件层接口](#组件层接口) - 逐飞助手、上位机通信

### 实战示例索引（基于实际项目）

| 示例编号 | 功能模块 | 示例项目路径 | 说明 |
|---------|---------|------------|------|
| [示例1](#示例1gpio控制---按键读取与蜂鸣器控制) | GPIO基础 | `E01_01_button_switch_buzzer_demo` | 按键输入、蜂鸣器输出 |
| [示例2](#示例2adc采样---电池电压检测) | ADC采样 | `E04_01_adc_battery_volatage_detection_demo` | 电池电压检测、分压计算 |
| [示例3](#示例3编码器---正交编码器速度采集) | 编码器 | `E03_02_encoder_quad_demo` | 正交编码器速度测量 |
| [示例4](#示例4pwm控制---双电机控制drv8701e驱动) | 电机控制 | `E02_04_drv8701e_double_motor_control_demo` | 双路PWM电机驱动 |
| [示例5](#示例5舵机控制---pwm角度控制) | 舵机控制 | `E02_06_servo_control_demo` | 舵机角度控制 |
| [示例6](#示例6imu传感器---六轴九轴数据采集) | IMU传感器 | `E05_07_imu_demo` | 加速度、陀螺仪、磁力计 |
| [示例7](#示例7ips200屏幕---图形和文字显示) | 屏幕显示 | `E06_04_ips200_display_demo` | 文字、图形、颜色显示 |
| [示例8](#示例8usb摄像头---图像采集与屏幕显示) | 摄像头 | `E09_01_uvc_ips200_display_demo` | 图像采集、灰度/彩色显示 |
| [示例9](#示例9udp网络通信---数据收发) | UDP通信 | `E07_01_udp_demo` | UDP客户端数据收发 |
| [示例10](#示例10tcp网络通信---客户端连接) | TCP通信 | `E07_03_tcp_client_demo` | TCP客户端连接与通信 |
| [示例11](#示例11dl1x激光测距传感器---tof距离测量) | 测距传感器 | `E07_01_tof_dl1x_demo` | TOF激光测距 |
| [示例12](#示例12定时器应用---周期中断控制) | 定时器 | `E07_pit_demo` | 周期中断、定时任务 |
| [示例13](#示例13综合应用---智能小车控制系统) | 综合应用 | - | 多模块融合应用 |

### 参考资料
- [宏定义和常量](#宏定义和常量)
- [数据类型定义](#数据类型定义)
- [常见问题和注意事项](#常见问题和注意事项)
- [进阶应用场景](#进阶应用场景)
- [附录：设备树配置参考](#附录设备树配置参考)

---

## 项目架构概览

该库采用分层架构设计，从下到上分为四层：

```
┌─────────────────────────────────────────┐
│     应用组件层 (zf_components)          │  应用辅助工具
├─────────────────────────────────────────┤
│     外接设备层 (zf_device)              │  I2C/USB/IIO设备
├─────────────────────────────────────────┤
│     芯片驱动层 (zf_driver)              │  GPIO/PWM/ADC/网络/文件
├─────────────────────────────────────────┤
│     公共工具层 (zf_common)              │  数据结构/字体/函数
└─────────────────────────────────────────┘
```


### 编译和运行

**单个项目编译**：
```bash
cd Motherboard_Demo/E1_motherboard/E01_01_button_switch_buzzer_demo/user
./build.sh
```

### 开发流程

1. **选择示例模板**：根据需求选择最接近的示例项目
2. **复制项目**：复制整个项目文件夹作为开发起点
3. **修改代码**：在 `user/main.cpp` 中实现您的功能
4. **编译测试**：运行 `build.sh` 编译并测试
5. **部署运行**：将编译好的程序部署到目标板

---

## 公共层接口

### 1. 通用函数 (`zf_common_function.hpp`)

#### 宏定义函数

```cpp
// 绝对值函数，数据范围[-32767, 32767]
#define func_abs(x)

// 单边限幅函数，数据被限制在[-y, +y]范围内
#define func_limit(x, y)

// 双边限幅函数，数据被限制在[a, b]范围内
#define func_limit_ab(x, a, b)
```

#### 数学运算函数

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|--------|------|--------|
| `func_get_greatest_common_divisor` | 求最大公约数 | `uint32 num1, uint32 num2` | `uint32` 最大公约数 |
| `func_soft_delay` | 软件延时 | `volatile long t` | `void` |

#### 字符串转换函数

| 函数名 | 功能描述 | 示例 |
|--------|--------|------|
| `func_str_to_int` | 字符串转有符号整数 | `"123"` → `123` |
| `func_int_to_str` | 有符号整数转字符串 | `123` → `"123"` |
| `func_str_to_uint` | 字符串转无符号整数 | `"456"` → `456` |
| `func_uint_to_str` | 无符号整数转字符串 | `456` → `"456"` |
| `func_str_to_float` | 字符串转浮点数 | `"3.14"` → `3.14f` |
| `func_float_to_str` | 浮点数转字符串 | `3.14f` → `"3.14"` |
| `func_str_to_double` | 字符串转双精度浮点数 | `"3.14"` → `3.14` |
| `func_double_to_str` | 双精度浮点数转字符串 | `3.14` → `"3.14"` |
| `func_str_to_hex` | 字符串转十六进制数 | `"FF"` → `255` |
| `func_hex_to_str` | 十六进制数转字符串 | `255` → `"FF"` |

#### 格式化输出函数

```cpp
// 类似sprintf的格式化输出函数
uint32 zf_sprintf(int8 *buff, const int8 *format, ...);
```

---

### 2. FIFO缓冲区 (`zf_common_fifo.hpp`)

#### 状态枚举

```cpp
typedef enum {
    FIFO_SUCCESS,           // 操作成功
    FIFO_RESET_UNDO,        // 重置操作未执行
    FIFO_CLEAR_UNDO,        // 清空操作未执行
    FIFO_BUFFER_NULL,       // 缓冲区异常
    FIFO_WRITE_UNDO,        // 写入操作未执行
    FIFO_SPACE_NO_ENOUGH,   // 缓冲区空间不足
    FIFO_READ_UNDO,         // 读取操作未执行
    FIFO_DATA_NO_ENOUGH     // 数据长度不足
} fifo_state_enum;
```

#### 操作枚举

```cpp
typedef enum {
    FIFO_READ_AND_CLEAN,    // 读取后清空释放缓冲区
    FIFO_READ_ONLY          // 仅读取
} fifo_operation_enum;
```

#### 数据类型枚举

```cpp
typedef enum {
    FIFO_DATA_8BIT,         // 8位数据
    FIFO_DATA_16BIT,        // 16位数据
    FIFO_DATA_32BIT         // 32位数据
} fifo_data_type_enum;
```

#### FIFO结构体

```cpp
typedef struct {
    uint8               execution;  // 执行状态
    fifo_data_type_enum type;       // 数据类型
    void                *buffer;    // 缓存指针
    uint32              head;       // 缓存头指针（指向空的缓存）
    uint32              end;        // 缓存尾指针（指向非空缓存）
    uint32              size;       // 缓存剩余大小
    uint32              max;        // 缓存总大小
} fifo_struct;
```

#### FIFO操作函数

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|--------|------|--------|
| `fifo_init` | 初始化FIFO | `fifo_struct *fifo, fifo_data_type_enum type, void *buffer_addr, uint32 size` | `fifo_state_enum` |
| `fifo_clear` | 清空FIFO | `fifo_struct *fifo` | `fifo_state_enum` |
| `fifo_used` | 获取FIFO已使用大小 | `fifo_struct *fifo` | `uint32` |
| `fifo_write_element` | 写入单个元素 | `fifo_struct *fifo, uint32 dat` | `fifo_state_enum` |
| `fifo_write_buffer` | 写入缓冲区 | `fifo_struct *fifo, void *dat, uint32 length` | `fifo_state_enum` |
| `fifo_read_element` | 读取单个元素 | `fifo_struct *fifo, void *dat, fifo_operation_enum flag` | `fifo_state_enum` |
| `fifo_read_buffer` | 读取缓冲区 | `fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag` | `fifo_state_enum` |
| `fifo_read_tail_buffer` | 读取FIFO尾部 | `fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag` | `fifo_state_enum` |

---

### 3. 字体和颜色 (`zf_common_font.hpp`)

#### RGB565颜色枚举

```cpp
typedef enum {
    RGB565_WHITE    = 0xFFFF,   // 白色
    RGB565_BLACK    = 0x0000,   // 黑色
    RGB565_BLUE     = 0x001F,   // 蓝色
    RGB565_RED      = 0xF800,   // 红色
    RGB565_GREEN    = 0x07E0,   // 绿色
    RGB565_YELLOW   = 0xFFE0,   // 黄色
    RGB565_CYAN     = 0x07FF,   // 青色
    RGB565_PURPLE   = 0xF81F,   // 紫色
    RGB565_PINK     = 0xFE19,   // 粉色
    RGB565_BROWN    = 0xBC40,   // 棕色
    RGB565_GRAY     = 0x8430    // 灰色
} rgb565_color_enum;
```

#### 外部字体资源

```cpp
extern const uint8 ascii_font_8x16[][16];      // ASCII 8x16字体
extern const uint8 ascii_font_6x8[][6];        // ASCII 6x8字体
extern const uint8 chinese_test[8][16];        // 中文字体测试
extern const uint8 oled_16x16_chinese[][16];   // 16x16中文字体
extern const uint8 gImage_seekfree_logo[38400];// 逐飞Logo图像
```

---

## 驱动层接口

### 1. 延时驱动 (`zf_driver_delay.hpp`)

```cpp
// 毫秒延时宏
#define system_delay_ms(time)  (usleep(time*1000))

// 微秒延时宏
#define system_delay_us(time)  (usleep(time))
```

---

### 2. 文件操作基类 (`zf_driver_file_buffer.hpp`)

#### 类声明

```cpp
class zf_driver_file_buffer
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_file_buffer()` | 构造函数 | `const char* path, int flags` | - |
| `~zf_driver_file_buffer()` | 析构函数 | - | - |
| `set_path()` | 重新设置文件路径 | `const char* path, int flags` | `void` |
| `read_buff()` | 读取数据 | `uint8 *buf, int32 len` | `int8` (0-成功，-1-失败) |
| `write_buff()` | 写入数据 | `uint8 *buf, int32 len` | `int8` (0-成功，-1-失败) |

**使用示例**：
```cpp
zf_driver_file_buffer file_obj("/dev/data", O_RDWR);
uint8 data[100];
file_obj.read_buff(data, 100);
file_obj.write_buff(data, 100);
```

---

### 3. 文件字符串操作 (`zf_driver_file_string.hpp`)

#### 类声明

```cpp
class zf_driver_file_string
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_file_string()` | 构造函数 | `const char* path, const char* mode = "r+"` | - |
| `~zf_driver_file_string()` | 析构函数 | - | - |
| `set_path()` | 重新设置文件路径 | `const char* path, const char* mode = "r+"` | `void` |
| `rewind_file()` | 重置文件指针 | - | `void` |
| `read_string()` | 读取字符串 | `char *str` | `int8` (0-成功，-1-失败) |
| `write_string()` | 写入字符串 | `char *str` | `int8` (0-成功，-1-失败) |

**使用示例**：
```cpp
zf_driver_file_string file_obj("/sys/class/adc/value", "r");
char buffer[64];
file_obj.read_string(buffer);
```

---

### 4. GPIO驱动 (`zf_driver_gpio.hpp`)

#### 设备节点定义

```cpp
#define ZF_GPIO_BEEP              "/dev/zf_gpio_beep"           // 蜂鸣器
#define ZF_GPIO_HALL_DETECTION    "/dev/zf_gpio_hall_detection" // 霍尔检测
#define ZF_GPIO_KEY_1             "/dev/zf_gpio_key_1"          // 按键1
#define ZF_GPIO_KEY_2             "/dev/zf_gpio_key_2"          // 按键2
#define ZF_GPIO_KEY_3             "/dev/zf_gpio_key_3"          // 按键3
#define ZF_GPIO_KEY_4             "/dev/zf_gpio_key_4"          // 按键4
#define ZF_GPIO_MOTOR_1           "/dev/zf_gpio_motor_1"        // 电机1
#define ZF_GPIO_MOTOR_2           "/dev/zf_gpio_motor_2"        // 电机2
```

#### 类声明

```cpp
class zf_driver_gpio : public zf_driver_file_buffer
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_gpio()` | 构造函数 | `const char* path, int flags = O_RDWR` | - |
| `set_level()` | 设置GPIO电平 | `uint8 level` (0或1) | `void` |
| `get_level()` | 获取GPIO电平 | - | `uint8` (0或1) |

**使用示例**：
```cpp
zf_driver_gpio gpio_beep(ZF_GPIO_BEEP);
gpio_beep.set_level(1);  // 蜂鸣器打开
uint8 level = gpio_beep.get_level();
```

---

### 5. 编码器驱动 (`zf_driver_encoder.hpp`)

#### 设备节点定义

```cpp
#define ZF_ENCODER_QUAD_1  "/dev/zf_encoder_quad_1"  // 正交编码器1（推荐）
#define ZF_ENCODER_QUAD_2  "/dev/zf_encoder_quad_2"  // 正交编码器2（推荐）
#define ZF_ENCODER_DIR_1   "/dev/zf_encoder_dir_1"   // 方向编码器1
#define ZF_ENCODER_DIR_2   "/dev/zf_encoder_dir_2"   // 方向编码器2
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_encoder()` | 构造函数 | `const char* path, int flags = O_RDWR` | - |
| `get_count()` | 获取编码器计数 | - | `int16` 计数值 |
| `clear_count()` | 清零编码器计数 | - | `void` |

**使用示例**：
```cpp
zf_driver_encoder encoder(ZF_ENCODER_QUAD_1);
int16 count = encoder.get_count();
encoder.clear_count();
```

---

### 6. PWM驱动 (`zf_driver_pwm.hpp`)

#### 设备节点定义

```cpp
#define ZF_PWM_ESC_1       "/dev/zf_pwm_esc_1"       // 电调1
#define ZF_PWM_MOTOR_1     "/dev/zf_pwm_motor_1"     // PWM电机1
#define ZF_PWM_MOTOR_2     "/dev/zf_pwm_motor_2"     // PWM电机2
#define ZF_PWM_SERVO_1     "/dev/zf_pwm_servo_1"     // 舵机1
```

#### PWM信息结构体

```cpp
struct pwm_info {
    uint32 freq;        // PWM频率
    uint32 duty;        // PWM占空比
    uint32 duty_max;    // PWM占空比最大值
    uint32 duty_ns;     // PWM高电平时间（纳秒）
    uint32 period_ns;   // PWM周期时间（纳秒）
    uint32 clk_freq;    // 时钟频率
};
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_pwm()` | 构造函数 | `const char* path, int flags = O_RDWR` | - |
| `get_dev_info()` | 获取PWM设备信息 | `pwm_info *pwm_info` | `void` |
| `set_duty()` | 设置占空比 | `uint16 duty` | `void` |

**使用示例**：
```cpp
zf_driver_pwm pwm(ZF_PWM_MOTOR_1);
pwm.set_duty(500);  // 设置占空比为500
struct pwm_info info;
pwm.get_dev_info(&info);
```

---

### 7. ADC驱动 (`zf_driver_adc.hpp`)

#### ADC通道路径定义

```cpp
#define ADC_CH0_PATH        "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define ADC_CH1_PATH        "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
// ... ADC_CH2_PATH ~ ADC_CH7_PATH
#define ADC_SCALE_PATH      "/sys/bus/iio/devices/iio:device0/in_voltage_scale"
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_adc()` | 构造函数 | `const char* adc_path, const char* mode = "r"` | - |
| `~zf_driver_adc()` | 析构函数 | - | - |
| `convert()` | 读取ADC原始值 | - | `uint16` (0-2047) |
| `get_scale()` | 获取校准系数 | - | `float` 比例系数 |

**使用示例**：
```cpp
zf_driver_adc battery_adc(ADC_CH7_PATH);
uint16 raw_value = battery_adc.convert();
float scale = battery_adc.get_scale();
float voltage = raw_value * scale;
```

---

### 8. 定时器驱动 (`zf_driver_pit.hpp`)

#### 定时器限制

```cpp
#define PIT_MIN_PERIOD_MS    1      // 最小周期1ms
#define PIT_MAX_PERIOD_MS    1000   // 最大周期1000ms
```

#### 回调函数类型

```cpp
typedef void (*pit_callback_fun)(void);
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_pit()` | 构造函数 | - | - |
| `~zf_driver_pit()` | 析构函数 | - | - |
| `init_ms()` | 初始化定时器 | `uint32_t period_ms, pit_callback_fun callback` | `int` (0-成功，-1-失败) |
| `stop()` | 停止定时器 | - | `void` |

**使用示例**：
```cpp
void timer_callback() {
    // 定时器中断处理
}

zf_driver_pit pit_timer;
pit_timer.init_ms(10, timer_callback);  // 10ms周期
// ...
pit_timer.stop();
```

---

### 9. 高精度定时器 (`zf_driver_pit_fd.hpp`)

#### 类声明

```cpp
class timer_fd
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `timer_fd()` | 构造函数 | `int interval, const std::function<void()>& func` | - |
| `~timer_fd()` | 析构函数 | - | - |
| `start()` | 启动定时器 | - | `void` |
| `stop()` | 停止定时器 | - | `void` |

**使用示例**：
```cpp
auto callback = []() {
    // 定时任务
};
timer_fd timer(10, callback);  // 10ms定时
timer.start();
// ...
timer.stop();
```

---

### 10. UDP网络驱动 (`zf_driver_udp.hpp`)

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_udp()` | 构造函数 | - | - |
| `~zf_driver_udp()` | 析构函数 | - | - |
| `init()` | 初始化UDP客户端 | `const char *ip_addr, uint32 port` | `int8` (0-成功，-1-失败) |
| `send_data()` | 发送数据 | `const uint8 *buff, uint32 length` | `uint32` 实际发送字节数 |
| `read_data()` | 接收数据 | `uint8 *buff, uint32 length` | `uint32` 实际接收字节数 |

**使用示例**：
```cpp
zf_driver_udp udp_client;
udp_client.init("192.168.1.10", 8000);
uint8 send_buf[100] = "Hello";
udp_client.send_data(send_buf, 5);
uint8 recv_buf[100];
udp_client.read_data(recv_buf, 100);
```

---

### 11. TCP网络驱动 (`zf_driver_tcp_client.hpp`)

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_driver_tcp_client()` | 构造函数 | - | - |
| `~zf_driver_tcp_client()` | 析构函数 | - | - |
| `init()` | 初始化TCP客户端并连接 | `const char *ip_addr, uint32 port` | `int8` (0-成功，-1-失败) |
| `send_data()` | 发送数据 | `const uint8 *buff, uint32 length` | `uint32` 实际发送字节数 |
| `read_data()` | 接收数据 | `uint8 *buff, uint32 length` | `uint32` 实际接收字节数 |

**使用示例**：
```cpp
zf_driver_tcp_client tcp_client;
tcp_client.init("192.168.1.100", 8080);
uint8 send_buf[100] = "Hello Server";
tcp_client.send_data(send_buf, 12);
uint8 recv_buf[100];
tcp_client.read_data(recv_buf, 100);
```

---

## 设备层接口

### 1. IMU传感器驱动 (`zf_device_imu.hpp`)

#### IMU IIO框架路径定义

```cpp
#define IMU_EVENT_PATH      "/sys/bus/iio/devices/iio:device1/events/in_voltage_change_en"
#define IMU_ACC_X_PATH      "/sys/bus/iio/devices/iio:device1/in_accel_x_raw"
#define IMU_ACC_Y_PATH      "/sys/bus/iio/devices/iio:device1/in_accel_y_raw"
#define IMU_ACC_Z_PATH      "/sys/bus/iio/devices/iio:device1/in_accel_z_raw"
#define IMU_GYRO_X_PATH     "/sys/bus/iio/devices/iio:device1/in_anglvel_x_raw"
#define IMU_GYRO_Y_PATH     "/sys/bus/iio/devices/iio:device1/in_anglvel_y_raw"
#define IMU_GYRO_Z_PATH     "/sys/bus/iio/devices/iio:device1/in_anglvel_z_raw"
#define IMU_MAG_X_PATH      "/sys/bus/iio/devices/iio:device1/in_magn_x_raw"
#define IMU_MAG_Y_PATH      "/sys/bus/iio/devices/iio:device1/in_magn_y_raw"
#define IMU_MAG_Z_PATH      "/sys/bus/iio/devices/iio:device1/in_magn_z_raw"
```

#### IMU设备类型枚举

```cpp
typedef enum {
    DEV_NO_FIND    = 0,    // 未识别到设备
    DEV_IMU660RA   = 1,    // IMU660RA型号
    DEV_IMU660RB   = 2,    // IMU660RB型号
    DEV_IMU660RC   = 3,    // IMU660RC型号
    DEV_IMU963RA   = 4     // IMU963RA型号（带磁力计）
} imu_device_type_enum;
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_device_imu()` | 构造函数 | - | - |
| `~zf_device_imu()` | 析构函数 | - | - |
| `get_dev_type()` | 获取设备类型 | - | `imu_device_type_enum` |
| `get_acc_x()` | 获取加速度X轴 | - | `int16` |
| `get_acc_y()` | 获取加速度Y轴 | - | `int16` |
| `get_acc_z()` | 获取加速度Z轴 | - | `int16` |
| `get_gyro_x()` | 获取角速度X轴 | - | `int16` |
| `get_gyro_y()` | 获取角速度Y轴 | - | `int16` |
| `get_gyro_z()` | 获取角速度Z轴 | - | `int16` |
| `get_mag_x()` | 获取磁力计X轴 | - | `int16` (仅963RA有效) |
| `get_mag_y()` | 获取磁力计Y轴 | - | `int16` (仅963RA有效) |
| `get_mag_z()` | 获取磁力计Z轴 | - | `int16` (仅963RA有效) |
| `init()` | 初始化IMU设备 | - | `void` |

**使用示例**：
```cpp
zf_device_imu imu_dev;
imu_dev.init();
int16 ax = imu_dev.get_acc_x();
int16 gx = imu_dev.get_gyro_x();
```

---

### 2. LCD屏幕驱动 (`zf_device_ips200_fb.hpp`)

#### 屏幕定义

```cpp
#define DEFAULT_PENCOLOR    RGB565_RED           // 默认画笔颜色
#define DEFAULT_BGCOLOR     RGB565_WHITE         // 默认背景颜色
#define FB_PATH             "/dev/fb0"           // 帧缓冲设备
// 屏幕分辨率：240x320，RGB565格式
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_device_ips200()` | 构造函数 | - | - |
| `init()` | 初始化屏幕设备 | `const char* fb_path` | `void` |
| `clear()` | 清屏 | - | `void` |
| `full()` | 屏幕填充 | `const uint16 color` | `void` |
| `draw_point()` | 画点 | `uint16 x, uint16 y, const uint16 color` | `void` |
| `draw_line()` | 画线 | `uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color` | `void` |
| `show_char()` | 显示单个字符 | `uint16 x, uint16 y, const char dat` | `void` |
| `show_string()` | 显示字符串 | `uint16 x, uint16 y, const char dat[]` | `void` |
| `show_int()` | 显示有符号整数 | `uint16 x, uint16 y, const int32 dat, uint8 num` | `void` |
| `show_uint()` | 显示无符号整数 | `uint16 x, uint16 y, const uint32 dat, uint8 num` | `void` |
| `show_float()` | 显示浮点数 | `uint16 x, uint16 y, const float dat, uint8 point_bit, uint8 num` | `void` |
| `show_gray_image()` | 显示灰度图像 | `uint16 x, uint16 y, uint8 *image, uint16 width, uint16 height` | `void` |
| `show_rgb_image()` | 显示RGB565图像 | `uint16 x, uint16 y, uint16 *image, uint16 width, uint16 height` | `void` |

**使用示例**：
```cpp
zf_device_ips200 lcd;
lcd.init(FB_PATH);
lcd.clear();
lcd.show_string(10, 20, "Hello LS2K0300");
lcd.show_int(10, 40, -123, 4);

// 显示图像
uint8 gray_img[160*120];
lcd.show_gray_image(0, 0, gray_img, 160, 120);
```

---

### 3. USB摄像头驱动 (`zf_device_uvc.hpp`)

#### 摄像头定义

```cpp
#define UVC_WIDTH     160      // 图像宽度
#define UVC_HEIGHT    120      // 图像高度
#define UVC_FPS       60       // 帧率
#define UVC_PATH      "/dev/video0"  // 设备路径
#define UVC_AUTO_EXPOSURE_ENABLE   3   // 开启自动曝光
#define UVC_AUTO_EXPOSURE_DISABLE  1   // 关闭自动曝光
#define UVC_DEFAULT_EXPOSURE       300 // 默认曝光值
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_device_uvc()` | 构造函数 | - | - |
| `~zf_device_uvc()` | 析构函数 | - | - |
| `init()` | 初始化摄像头 | `const char* dev_path` | `int8` (0-成功，负数-失败) |
| `wait_image_refresh()` | 等待图像刷新（阻塞） | - | `int8` (0-成功，-1-失败) |
| `get_gray_image_ptr()` | 获取灰度图指针 | - | `uint8_t*` |
| `get_rgb_image_ptr()` | 获取RGB565图像指针 | - | `uint16_t*` |

**使用示例**：
```cpp
zf_device_uvc uvc_camera;
uvc_camera.init(UVC_PATH);

if (uvc_camera.wait_image_refresh() == 0) {
    // 获取灰度图
    uint8_t *gray_img = uvc_camera.get_gray_image_ptr();
    
    // 获取彩色图
    uint16_t *rgb_img = uvc_camera.get_rgb_image_ptr();
    
    // 处理图像数据
}
```

---

### 4. DL1X测距传感器驱动 (`zf_device_dl1x.hpp`)

#### 传感器路径定义

```cpp
#define DL1X_EVENT_PATH     "/sys/bus/iio/devices/iio:device2/events/in_voltage_change_en"
#define DL1X_DISTANCE_PATH  "/sys/bus/iio/devices/iio:device2/events/in_distance_raw"
```

#### 设备类型枚举

```cpp
enum dl1x_device_type_enum {
    NO_FIND_DEVICE = 0,
    ZF_DEVICE_DL1A = 1,
    ZF_DEVICE_DL1B = 2
};
```

#### 主要方法

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `zf_device_dl1x()` | 构造函数 | - | - |
| `~zf_device_dl1x()` | 析构函数 | - | - |
| `get_dev_type()` | 获取设备类型 | - | `enum dl1x_device_type_enum` |
| `get_distance()` | 获取距离数据 | - | `int16` |
| `init()` | 初始化传感器 | - | `void` |

---

## 组件层接口

### 1. 逐飞助手 (`seekfree_assistant.hpp`)

#### 功能启用定义

```cpp
#define SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE      1  // 启用参数调节
```

#### 缓冲区定义

```cpp
#define SEEKFREE_ASSISTANT_BUFFER_SIZE              0x80  // 接收FIFO大小
#define SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT   0x08  // 示波器最大通道数
#define SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT       0x08  // 参数调试最大通道数
#define SEEKFREE_ASSISTANT_CAMERA_MAX_BOUNDARY      0x08  // 图像边线最大数量
```

#### 协议定义

```cpp
#define SEEKFREE_ASSISTANT_SEND_HEAD                0xAA  // 发送帧头
#define SEEKFREE_ASSISTANT_RECEIVE_HEAD             0x55  // 接收帧头
```

#### 功能字定义

```cpp
#define SEEKFREE_ASSISTANT_CAMERA_FUNCTION          0x02  // 摄像头功能
#define SEEKFREE_ASSISTANT_CAMERA_DOT_FUNCTION      0x03  // 摄像头点绘制功能
#define SEEKFREE_ASSISTANT_CAMERA_OSCILLOSCOPE      0x10  // 摄像头示波器功能
#define SEEKFREE_ASSISTANT_RECEIVE_SET_PARAMETER    0x20  // 参数设置功能
```

#### 图像类型枚举

```cpp
typedef enum {
    SEEKFREE_ASSISTANT_OV7725_BIN = 1,
    SEEKFREE_ASSISTANT_MT9V03X,
    SEEKFREE_ASSISTANT_SCC8660,
    
    SEEKFREE_ASSISTANT_BINARY = 1,
    SEEKFREE_ASSISTANT_GRAY,
    SEEKFREE_ASSISTANT_RGB565
} seekfree_assistant_image_type_enum;
```

#### 边界类型枚举

```cpp
typedef enum {
    X_BOUNDARY,    // 仅X坐标
    Y_BOUNDARY,    // 仅Y坐标
    XY_BOUNDARY,   // X和Y坐标
    NO_BOUNDARY    // 无边线信息
} seekfree_assistant_boundary_type_enum;
```

#### 数据结构

**示波器结构体**：
```cpp
typedef struct {
    uint8 head;               // 帧头
    uint8 channel_num;        // 高四位为功能字，低四位为通道数量
    uint8 check_sum;          // 和校验
    uint8 length;             // 包长度
    float data[8];            // 通道数据
} seekfree_assistant_oscilloscope_struct;
```

**摄像头结构体**：
```cpp
typedef struct {
    uint8 head;               // 帧头
    uint8 function;           // 功能字
    uint8 camera_type;        // 边界数量和是否有图像
    uint8 length;             // 包长度
    uint16 image_width;       // 图像宽度
    uint16 image_height;      // 图像高度
} seekfree_assistant_camera_struct;
```

**参数结构体**：
```cpp
typedef struct {
    uint8 head;               // 帧头
    uint8 function;           // 功能字
    uint8 channel;            // 通道
    uint8 check_sum;          // 和校验
    float data;               // 数据
} seekfree_assistant_parameter_struct;
```

#### 回调函数类型

```cpp
// 传输回调函数
typedef uint32 (*seekfree_assistant_transfer_callback_function) (const uint8 *buff, uint32 length);

// 接收回调函数
typedef uint32 (*seekfree_assistant_receive_callback_function)  (uint8 *buff, uint32 length);
```

---

## 宏定义和常量

### 系统延时宏

```cpp
#define system_delay_ms(time)  (usleep(time*1000))  // 毫秒延时
#define system_delay_us(time)  (usleep(time))       // 微秒延时
```

### 文件操作标志位

```cpp
O_RDWR      // 读写模式
O_WRONLY    // 仅写模式
O_RDONLY    // 仅读模式
```

### 屏幕相关

```cpp
#define DEFAULT_PENCOLOR    RGB565_RED      // 默认画笔颜色
#define DEFAULT_BGCOLOR     RGB565_WHITE    // 默认背景颜色
#define FB_PATH             "/dev/fb0"      // 帧缓冲设备
```

---

## 数据类型定义

### 基础数据类型

在 `zf_common_typedef.hpp` 中定义：

```cpp
typedef int8_t      int8;       // 有符号8位整数
typedef uint8_t     uint8;      // 无符号8位整数
typedef int16_t     int16;      // 有符号16位整数
typedef uint16_t    uint16;     // 无符号16位整数
typedef int32_t     int32;      // 有符号32位整数
typedef uint32_t    uint32;     // 无符号32位整数
```

### 系统常用库

```cpp
#include <stdio.h>              // 标准输入输出
#include <stdlib.h>             // 标准库
#include <string.h>             // 字符串处理
#include <stdint.h>             // 固定宽度整数类型
#include <pthread.h>            // POSIX线程
#include <unistd.h>             // UNIX系统调用
#include <fcntl.h>              // 文件控制
#include <sys/mman.h>           // 内存映射
#include <sys/ioctl.h>          // 设备控制
#include <sys/socket.h>         // 网络套接字
#include <linux/fb.h>           // 帧缓冲
```

---

## 完整应用示例

### 示例1：GPIO控制 - 按键读取与蜂鸣器控制

**硬件连接**：
- KEY_1 ~ KEY_4：主板按键（默认GPIO77-80）
- BEEP：蜂鸣器（默认GPIO设备节点）

**功能说明**：循环读取4个按键状态，并通过串口输出

```cpp
#include "zf_common_headfile.hpp"

#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4

// 创建GPIO对象
zf_driver_gpio key_1(KEY_1_PATH, O_RDWR);
zf_driver_gpio key_2(KEY_2_PATH, O_RDWR);
zf_driver_gpio key_3(KEY_3_PATH, O_RDWR);
zf_driver_gpio key_4(KEY_4_PATH, O_RDWR);

int main(int, char**) 
{
    while(1)
    {
        // 读取按键电平状态
        printf("key_1 = %d\r\n", key_1.get_level());
        printf("key_2 = %d\r\n", key_2.get_level());
        printf("key_3 = %d\r\n", key_3.get_level());
        printf("key_4 = %d\r\n", key_4.get_level());
        
        system_delay_ms(100);
    }
}
```

---

### 示例2：ADC采样 - 电池电压检测

**硬件连接**：
- ADC_CH7：电池分压检测通道（硬件分压比11:1）

**功能说明**：采集电池电压，通过分压电路计算实际电压值

```cpp
#include "zf_common_headfile.hpp"

uint16  adc_reg      = 0;        // ADC采集原始数字量
float   adc_scale    = 0.0f;     // ADC电压校准系数 (单位: mv/bit)
uint16  battery_vol  = 0;        // 电池实际电压值 (单位: mv 毫伏)

// 初始化ADC对象，指定电池分压检测通道 ADC_CH7
zf_driver_adc battery_adc(ADC_CH7_PATH);

// 宏定义硬件参数 - 抽离为宏，方便修改，可读性强
#define BATTERY_DIV_RATIO      (11)    // 硬件分压比 (R37+R38)/R38 = 11

int main(int, char**) 
{
    // ADC校准系数只读取一次，放在while循环外面，程序上电初始化时读1次即可
    adc_scale = battery_adc.get_scale();

    while(1)
    {
        // 采集ADC原始值
        adc_reg = battery_adc.convert();

        // 电压计算公式(adc_reg*adc_scale=采样点电压mv  ×11=电池实际电压mv)
        battery_vol = adc_reg * adc_scale * BATTERY_DIV_RATIO;

        printf("adc_reg = %d\r\n", adc_reg);
        printf("adc_scale = %f\r\n", adc_scale);
        printf("battery vol = %d mv\r\n", battery_vol);

        system_delay_ms(1000);
    }
    return 0;
}
```

---

### 示例3：编码器 - 正交编码器速度采集

**硬件连接**：
- ENCODER_QUAD_1：编码器1（A-GPIO28, B-GPIO29）
- ENCODER_QUAD_2：编码器2（A-GPIO34, B-GPIO35）

**功能说明**：使用定时器10ms周期采集编码器计数，实现速度测量

```cpp
#include "zf_common_headfile.hpp"

int16 encoder_left;
int16 encoder_right;

#define ENCODER_QUAD_1_PATH           ZF_ENCODER_QUAD_1
#define ENCODER_QUAD_2_PATH           ZF_ENCODER_QUAD_2

// 创建编码器对象，传入文件路径
zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH);
zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH);

void pit_callback(void)
{
    // 定时器数值获取
    encoder_left = encoder_quad_1.get_count();
    encoder_right = encoder_quad_2.get_count();
    
    // 定时器清零
    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();
}

int main(int, char**) 
{
    // 创建一个定时器10ms周期，回调函数为pit_callback
    zf_driver_pit pit_timer;
    pit_timer.init_ms(10, pit_callback);

    while(1)
    {
        printf("encoder_left = %d\r\n", encoder_left);
        printf("encoder_right = %d\r\n", encoder_right);
        system_delay_ms(100);
    }
}
```

---

### 示例4：PWM控制 - 双电机控制（DRV8701E驱动）

**硬件连接**：
- MOTOR1_DIR：电机1方向（GPIO73）
- MOTOR1_PWM：电机1 PWM（GPIO86）
- MOTOR2_DIR：电机2方向（GPIO76）
- MOTOR2_PWM：电机2 PWM（GPIO87）

**功能说明**：控制双路电机PWM占空比和方向，实现正反转控制

```cpp
#include "zf_common_headfile.hpp"

#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1
#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2

zf_driver_pit pit_timer;
struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;

zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);

int8 duty = 0;
bool dir = true;

// 获取PWM设备信息中设置的duty_max值
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)       
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)        
#define MAX_DUTY               (30)   // 最大 30% 占空比

void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机
    pit_timer.stop();
    
    // 关闭电机，设置占空比为0
    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);
    printf("程序异常退出，执行清理操作\n");
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

// 定时器回调函数，控制电机占空比渐变
void pit_callback()
{
    if(dir)
    {
        duty++;
        if(duty >= MAX_DUTY)
        {
            dir = false;
        }
    }
    else
    {
        duty--;
        if(duty <= -MAX_DUTY)
        {
            dir = true;
        }
    }
    
    // 根据占空比正负设置方向
    if(duty >= 0)
    {
        drv8701e_dir_1.set_level(1);
        drv8701e_dir_2.set_level(1);
        drv8701e_pwm_1.set_duty((uint16)(MOTOR1_PWM_DUTY_MAX * duty / 100.0));
        drv8701e_pwm_2.set_duty((uint16)(MOTOR2_PWM_DUTY_MAX * duty / 100.0));
    }
    else
    {
        drv8701e_dir_1.set_level(0);
        drv8701e_dir_2.set_level(0);
        drv8701e_pwm_1.set_duty((uint16)(MOTOR1_PWM_DUTY_MAX * (-duty) / 100.0));
        drv8701e_pwm_2.set_duty((uint16)(MOTOR2_PWM_DUTY_MAX * (-duty) / 100.0));
    }
}

int main(int, char**) 
{
    // 注册清理函数和信号处理
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    // 获取PWM设备信息
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    // 创建定时器，100ms周期
    pit_timer.init_ms(100, pit_callback);

    while(1)
    {
        printf("duty = %d%%\r\n", duty);
        system_delay_ms(100);
    }
}
```

---

### 示例5：舵机控制 - PWM角度控制

**硬件连接**：
- SERVO_PWM：舵机PWM信号（GPIO86）
- VCC：舵机电源（需外部供电）
- GND：共地

**功能说明**：控制舵机在75°-105°范围内来回摆动

```cpp
#include "zf_common_headfile.hpp"

#define SERVO_PATH        ZF_PWM_SERVO_1

struct pwm_info servo_info;
zf_driver_pwm   servo_pwm(SERVO_PATH);

// 定义主板上舵机频率（50-300Hz范围）
#define SERVO_MOTOR_FREQ            (servo_info.freq)                       
#define PWM_DUTY_MAX                (servo_info.duty_max)      

// 定义舵机活动角度范围                                                     
#define SERVO_MOTOR_L_MAX           (75)                       
#define SERVO_MOTOR_R_MAX           (105)  

// 舵机占空比计算宏
// 舵机 0-180° 对应 0.5ms-2.5ms 高电平
// 公式：PWM_DUTY_MAX/(1000/freq)*(0.5+angle/90.0)
#define SERVO_MOTOR_DUTY(x)  ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

float servo_motor_duty = 90.0;      // 舵机当前角度
float servo_motor_dir = 1;          // 舵机动作方向

int main(int, char**) 
{
    // 获取PWM设备信息
    servo_pwm.get_dev_info(&servo_info);

    printf("servo pwm freq = %d Hz\r\n", servo_info.freq);
    printf("servo pwm duty_max = %d\r\n", servo_info.duty_max);
    
    while(1)
    {
        // 设置舵机占空比
        servo_pwm.set_duty((uint16)SERVO_MOTOR_DUTY(servo_motor_duty));

        // 角度渐变控制
        if(servo_motor_dir)
        {
            servo_motor_duty++;
            if(servo_motor_duty >= SERVO_MOTOR_R_MAX)
                servo_motor_dir = 0;
        }
        else
        {
            servo_motor_duty--;
            if(servo_motor_duty <= SERVO_MOTOR_L_MAX)
                servo_motor_dir = 1;
        }
        
        system_delay_ms(50);         
    }
}
```

---

### 示例6：IMU传感器 - 六轴/九轴数据采集

**硬件连接**：
- SCL/SPC：SPI时钟（GPIO60）
- SDA/DSI：SPI数据（GPIO62）
- SA0/SDO：SPI MISO（GPIO61）
- CS：片选（GPIO63）

**功能说明**：自动识别IMU660RA/IMU660RB/IMU963RA，定时采集加速度和陀螺仪数据

```cpp
#include "zf_common_headfile.hpp"

zf_driver_pit pit_timer;

// IMU六轴/九轴传感器原始数据存储变量
int16 imu_acc_x, imu_acc_y, imu_acc_z;
int16 imu_gyro_x, imu_gyro_y, imu_gyro_z;
int16 imu_mag_x, imu_mag_y, imu_mag_z;

// 定义IMU传感器设备对象
zf_device_imu imu_dev;

// PIT定时器10ms周期中断回调函数
void pit_callback(void)
{
    // 读取加速度数据
    imu_acc_x = imu_dev.get_acc_x();
    imu_acc_y = imu_dev.get_acc_y();
    imu_acc_z = imu_dev.get_acc_z();
    
    // 读取陀螺仪数据
    imu_gyro_x = imu_dev.get_gyro_x();
    imu_gyro_y = imu_dev.get_gyro_y();
    imu_gyro_z = imu_dev.get_gyro_z();
    
    // 读取磁力计数据（仅IMU963RA有效）
    if(DEV_IMU963RA == imu_dev.get_dev_type())
    {
        imu_mag_x = imu_dev.get_mag_x();
        imu_mag_y = imu_dev.get_mag_y();
        imu_mag_z = imu_dev.get_mag_z();
    }
}

int main(int, char**) 
{
    // 初始化IMU设备
    imu_dev.init();
    
    // 获取并显示设备类型
    imu_device_type_enum dev_type = imu_dev.get_dev_type();
    printf("IMU Device Type: %d\r\n", dev_type);
    
    // 创建定时器，10ms周期采集
    pit_timer.init_ms(10, pit_callback);

    while(1)
    {
        // 输出加速度数据
        printf("Acc: x=%d, y=%d, z=%d\r\n", imu_acc_x, imu_acc_y, imu_acc_z);
        
        // 输出陀螺仪数据
        printf("Gyro: x=%d, y=%d, z=%d\r\n", imu_gyro_x, imu_gyro_y, imu_gyro_z);
        
        // 输出磁力计数据（如果是963RA）
        if(DEV_IMU963RA == dev_type)
        {
            printf("Mag: x=%d, y=%d, z=%d\r\n", imu_mag_x, imu_mag_y, imu_mag_z);
        }
        
        system_delay_ms(100);
    }
}
```

---

### 示例7：IPS200屏幕 - 图形和文字显示

**硬件连接**：
- 2寸SPI屏幕（240x320分辨率，RGB565格式）
- SCL、SDA、RST、DC、CS、BL等引脚参考设备树配置

**功能说明**：演示屏幕清屏、显示文字、数字、画线、画点等功能

```cpp
#include "zf_common_headfile.hpp"

int16_t data_index = 0;
zf_device_ips200 ips200;

int main(int, char**) 
{
    // 初始化屏幕
    ips200.init(FB_PATH);

    while(1)
    {
        // 清屏并填充灰色背景
        ips200.clear();
        ips200.full(RGB565_GRAY);
        
        // 显示字符串
        ips200.show_string(0, 16*7, "SEEKFREE");
        
        // 显示浮点数 (整数位, 小数位)
        ips200.show_float(0, 16*8, -13.141592, 1, 6);   // -3.141592
        ips200.show_float(80, 16*8, 13.141592, 8, 4);   // 13.1415
        
        // 显示整数
        ips200.show_int(0, 16*9, -127, 2);
        ips200.show_uint(80, 16*9, 255, 4);
        
        // 显示更大的整数
        ips200.show_int(0, 16*10, -32768, 4);
        ips200.show_uint(80, 16*10, 65535, 6);
        
        system_delay_ms(1000);

        // 清屏并画线动画
        ips200.clear();
        for(data_index = 0; 240 > data_index; data_index += 10)
        {
            ips200.draw_line(0, 0, data_index, 319, RGB565_BLUE);
            system_delay_ms(20);
        }
        
        system_delay_ms(1000);

        // 彩色填充演示
        ips200.full(RGB565_RED);
        system_delay_ms(500);
        ips200.full(RGB565_GREEN);
        system_delay_ms(500);
        ips200.full(RGB565_BLUE);
        system_delay_ms(500);
    }
}
```

---

### 示例8：USB摄像头 - 图像采集与屏幕显示

**硬件连接**：
- UVC摄像头（USB接口）
- IPS200屏幕（用于图像显示）

**功能说明**：采集160x120分辨率图像，交替显示灰度图和RGB彩色图

```cpp
#include "zf_common_headfile.hpp"

zf_device_ips200 ips200;        // 屏幕设备对象
zf_device_uvc uvc_dev;          // UVC摄像头设备对象
uint16 count = 0;               // 帧计数器
uint8  show_flag = 0;           // 显示模式标志：0-灰度，1-彩色

int main() 
{
    // 初始化屏幕
    ips200.init(FB_PATH);
    
    // 初始化UVC摄像头
    if(uvc_dev.init(UVC_PATH) < 0)
    {
        printf("UVC init failed!\r\n");
        return -1;
    }

    while(1)
    {
        // 等待新一帧图像刷新
        if(uvc_dev.wait_image_refresh() == 0)
        {
            // 每200帧切换显示模式
            if(count++ >= 200)
            {
                show_flag = !show_flag;
                count = 0;
            }

            if(show_flag) // 显示RGB彩色图像
            {
                uint16* rgb_image = uvc_dev.get_rgb_image_ptr();
                if(NULL != rgb_image)
                {
                    ips200.show_rgb_image(0, 0, rgb_image, UVC_WIDTH, UVC_HEIGHT);
                }
            }
            else // 显示灰度图像
            {
                uint8* gray_image = uvc_dev.get_gray_image_ptr();
                if(NULL != gray_image)
                {
                    ips200.show_gray_image(0, 0, gray_image, UVC_WIDTH, UVC_HEIGHT);
                }
            }
        }
        system_delay_ms(10);
    }
    return 0;
}
```

---

### 示例9：UDP网络通信 - 数据收发

**硬件连接**：
- 网络连接（有线或WiFi）

**功能说明**：通过UDP协议与上位机通信，实现数据双向传输

```cpp
#include "zf_common_headfile.hpp"

// 目标IP地址和端口号
#define SERVER_IP "192.168.2.49"
#define PORT 8086

zf_driver_udp udp_dev;

uint32  read_len = 0;
uint8   recv_buff[1024];
uint8   temp_str[] = "seekfree this is udp demo.\r\n";
uint8   read_str[] = "read data:\r\n";

int main() 
{
    // 初始化UDP客户端
    if(udp_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("UDP client ok\r\n");
    }
    else
    {
        printf("UDP client error\r\n");
        return -1;
    }

    // 先发送消息，让对方知道本机IP
    udp_dev.send_data(temp_str, sizeof(temp_str));
    udp_dev.send_data(temp_str, sizeof(temp_str));
    udp_dev.send_data(temp_str, sizeof(temp_str));

    while (true) 
    {
        // 非阻塞式读取UDP数据
        read_len = udp_dev.read_data(recv_buff, 1024);
        
        // 回显UDP数据
        if(read_len > 0)
        {
            printf("read_len = %d\r\n", read_len);
            udp_dev.send_data(read_str, sizeof(read_str));
            udp_dev.send_data(recv_buff, read_len);
        }
  
        system_delay_ms(1);
    }

    return 0;
}
```

---

### 示例10：TCP网络通信 - 客户端连接

**硬件连接**：
- 网络连接（有线或WiFi）

**功能说明**：通过TCP协议连接服务器，实现可靠数据传输

```cpp
#include "zf_common_headfile.hpp"

// 服务器IP地址和端口号
#define SERVER_IP "192.168.2.49"
#define PORT 8086

zf_driver_tcp_client tcp_client_dev;

uint32  read_len = 0;
uint8   recv_buff[1024];
uint8   temp_str[] = "seekfree this is tcp_client demo.\r\n";
uint8   read_str[] = "read data:\r\n";

int main() 
{
    // 初始化TCP客户端并连接服务器
    if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    // 发送测试数据
    tcp_client_dev.send_data(temp_str, sizeof(temp_str));
    tcp_client_dev.send_data(temp_str, sizeof(temp_str));
    tcp_client_dev.send_data(temp_str, sizeof(temp_str));

    while (true) 
    {
        // 非阻塞式读取TCP数据
        read_len = tcp_client_dev.read_data(recv_buff, 1024);
        
        // 回显TCP数据
        if(read_len > 0)
        {
            printf("read_len = %d\r\n", read_len);
            tcp_client_dev.send_data(read_str, sizeof(read_str));
            tcp_client_dev.send_data(recv_buff, read_len);
        }
  
        system_delay_ms(1);
    }

    return 0;
}
```

---

### 示例11：DL1X激光测距传感器 - TOF距离测量

**硬件连接**：
- SCL：I2C时钟（GPIO49）
- SDA：I2C数据（GPIO48）
- XS：中断引脚（GPIO38）

**功能说明**：自动识别DL1A/DL1B传感器，定时采集距离数据

```cpp
#include "zf_common_headfile.hpp"

zf_device_dl1x dl1x_dev;                      // DL1X设备对象
enum dl1x_device_type_enum dl1x_dev_type;     // DL1X设备类型
zf_driver_pit dl1x_pit_timer;                 // PIT定时器对象
volatile int16 dl1x_distance_raw = 0;          // 距离原始数据

// PIT定时器100ms周期回调函数
void dl1x_pit_callback(void)
{
    // 读取DL1X距离数据
    if(NO_FIND_DEVICE != dl1x_dev_type)
    {
        dl1x_distance_raw = dl1x_dev.get_distance();
    }
}

int main(int, char**) 
{
    printf("=====================================\r\n");
    printf("DL1X 100ms 定时采集 Demo\r\n");
    printf("=====================================\r\n");

    // DL1X传感器初始化
    dl1x_dev_type = dl1x_dev.init();

    // 判断初始化结果
    if(NO_FIND_DEVICE == dl1x_dev_type)
    {
        printf("错误：未找到DL1X设备，请检查硬件连接！\r\n");
        return -1;
    }
    else
    {
        printf("DL1X 初始化成功！识别到：");
        if(ZF_DEVICE_DL1A == dl1x_dev_type) 
            printf("ZF_DEVICE_DL1A\r\n");
        else if(ZF_DEVICE_DL1B == dl1x_dev_type) 
            printf("ZF_DEVICE_DL1B\r\n");
    }

    // 初始化PIT定时器，100ms周期
    int pit_init_ret = dl1x_pit_timer.init_ms(100, dl1x_pit_callback);
    if(pit_init_ret != 0)
    {
        printf("错误：PIT定时器初始化失败！\r\n");
        return -1;
    }
    printf("PIT定时器初始化成功，100ms周期采集DL1X数据...\r\n");
    printf("=====================================\r\n");

    // 主循环，周期性打印距离数据
    while(1)
    {
        printf("DL1X 原始距离数据：%d mm\r\n", dl1x_distance_raw);
        system_delay_ms(1000);
    }

    return 0;
}
```

---

### 示例12：定时器应用 - 周期中断控制

**功能说明**：创建10ms周期定时器，在中断回调中执行周期任务

```cpp
#include "zf_common_headfile.hpp"

zf_driver_pit pit_timer;

void pit_callback()
{
    printf("pit_callback\r\n");
    // 在这里执行周期任务
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    // 停止定时器线程
    pit_timer.stop();
    printf("程序异常退出，执行清理操作\n");
}

int main(int, char**) 
{
    // 注册清理函数
    atexit(cleanup);
    
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    // 创建定时器，10ms周期，回调函数为pit_callback
    pit_timer.init_ms(10, pit_callback);

    while(1)
    {
        printf("main\r\n");
        system_delay_ms(100);
    }
}
```

---

## 常见问题和注意事项

### 硬件连接注意事项

1. **核心板安装**：确保LS2K0300核心板完全插入主板插座，无缝隙
2. **供电要求**：主板测试时必须使用电池供电，USB供电可能导致电压不足
3. **屏幕初始化**：屏幕的初始化在开机时完成，需在上电前插入屏幕
4. **舵机连接**：务必注意舵机线序（红-VCC、黑-GND、黄/橙/棕/白-信号）
5. **编码器接线**：注意A、B相引脚对应关系，参考设备树配置
6. **摄像头使用**：UVC摄像头支持热插拔，重新插入即可识别

### 软件使用注意事项

1. **内核依赖**：必须使用逐飞科技提供的内核，否则找不到设备节点
2. **设备树配置**：所有GPIO、PWM等引脚定义在 `seekfree_2k0300_coreboard.dts` 文件中
3. **PWM占空比设置**：
   - PWM的duty_max值在设备树中定义（通常为10000）
   - 设置占空比时需要计算：`实际占空比 = (duty / duty_max) * 100%`
   - 舵机控制需根据频率计算高电平时间
4. **定时器使用**：
   - 定时器周期范围：1ms ~ 1000ms
   - 回调函数在独立线程执行，避免耗时操作
   - 程序退出前必须调用 `stop()` 停止定时器
5. **网络通信**：
   - 需确保设备和目标主机在同一网络
   - UDP需先发送数据让对方知道本机IP
   - TCP需确保服务器已启动并监听指定端口
6. **图像处理**：
   - UVC摄像头默认分辨率160x120，帧率60fps
   - 灰度图为uint8格式，RGB为uint16(RGB565)格式
   - `wait_image_refresh()` 为阻塞函数，等待新帧到来

### 故障排查清单

**问题1：终端提示未找到xxx文件**
- 解决：使用逐飞科技提供的内核，否则设备节点不存在

**问题2：屏幕不显示**
- 检查屏幕供电引脚电压是否正常
- 确认屏幕插座位置和方向正确
- 检查是否在上电前插入屏幕

**问题3：摄像头没找到**
- 重新插入UVC摄像头（支持热插拔）
- 检查 `/dev/video0` 设备节点是否存在

**问题4：电机不转或无输出**
- 确认主板使用电池供电
- 检查模块供电连接是否正确
- 用万用表测量PWM引脚电压变化
- 检查程序是否正常烧录并运行

**问题5：编码器读数异常**
- 确认编码器类型（正交/方向）与代码匹配
- 检查A、B相引脚是否接反
- 测试编码器供电电压是否正常

**问题6：网络连接失败**
- 检查网络连接状态（有线或WiFi）
- 确认IP地址和端口号配置正确
- 使用 `ping` 测试网络连通性
- 检查防火墙设置

---

## 进阶应用场景

### 1. 智能小车控制系统

结合编码器、电机、IMU、摄像头等模块，实现：
- 速度闭环控制
- 姿态稳定
- 视觉巡线
- 远程监控

### 2. 机器人姿态控制

使用IMU传感器实现：
- 角度解算
- 姿态补偿
- 平衡控制
- 轨迹记录

### 3. 无线数据采集系统

通过TCP/UDP网络：
- 多传感器数据上传
- 远程参数调节
- 实时波形显示
- 数据存储分析

### 4. 视觉处理应用

使用UVC摄像头：
- 图像采集
- 边缘检测
- 目标识别
- 图像传输

---

## 附录：设备树配置参考

所有硬件资源的引脚分配和参数配置都定义在设备树文件 `seekfree_2k0300_coreboard.dts` 中。

**常用节点查询**：
- GPIO设备：`zf_gpio_*` 节点
- PWM设备：`zf_pwm_*` 节点
- 编码器：`zf_encoder_*` 节点
- SPI设备：`&spi1` 节点
- I2C设备：`&i2c4` 节点
- 屏幕驱动：`st7789v` 节点

**修改设备树后需要**：
1. 重新编译设备树
2. 替换系统中的dtb文件
3. 重启系统使配置生效

---

## 版本历史

### v1.0 (2026-01-25)
- ✅ 初始版本发布
- ✅ 完整的驱动层接口（GPIO、PWM、ADC、编码器、定时器）
- ✅ 网络通信支持（TCP/UDP）
- ✅ 设备层驱动（IMU、屏幕、摄像头、测距传感器）
- ✅ 基于实际项目的13个完整示例
- ✅ 详细的API文档和使用说明

### 计划功能
- 🔄 更多传感器驱动支持
- 🔄 图像处理算法库
- 🔄 PID控制算法优化
- 🔄 更多综合应用示例

---

## 技术支持

**官方支持渠道**：
- 🏪 **淘宝店铺**：https://seekfree.taobao.com/
- 💬 **技术论坛**：联系店铺客服获取论坛链接
- 📧 **技术支持**：通过淘宝旺旺联系技术客服
- 📖 **官方教程**：随产品附带完整教程文档

**常见问题**：
1. 硬件连接问题 → 参考[硬件连接注意事项](#硬件连接注意事项)
2. 编译错误 → 检查交叉编译工具链版本
3. 运行时错误 → 参考[故障排查清单](#故障排查清单)
4. 性能优化 → 参考[进阶应用场景](#进阶应用场景)

---

## 贡献指南

本开源库遵循 **GPL 3.0** 协议，欢迎社区贡献！

**贡献方式**：
1. **报告问题**：通过官方渠道反馈bug和改进建议
2. **提交代码**：Fork项目后提交Pull Request
3. **完善文档**：帮助改进文档和示例代码
4. **分享经验**：在社区分享您的应用案例

**代码规范**：
- 遵循现有代码风格
- 添加必要的注释说明
- 提供使用示例
- 保留逐飞科技版权声明

---

### 示例13：综合应用 - 智能小车控制系统

**功能说明**：整合多个模块，实现带速度闭环、姿态监控、远程通信的智能小车系统

```cpp
#include "zf_common_headfile.hpp"

// ========== 硬件对象定义 ==========
// 电机驱动
zf_driver_pwm   motor_left_pwm(ZF_PWM_MOTOR_1);
zf_driver_pwm   motor_right_pwm(ZF_PWM_MOTOR_2);
zf_driver_gpio  motor_left_dir(ZF_GPIO_MOTOR_1, O_RDWR);
zf_driver_gpio  motor_right_dir(ZF_GPIO_MOTOR_2, O_RDWR);

// 编码器
zf_driver_encoder encoder_left(ZF_ENCODER_QUAD_1);
zf_driver_encoder encoder_right(ZF_ENCODER_QUAD_2);

// IMU传感器
zf_device_imu imu_dev;

// 屏幕显示
zf_device_ips200 ips200;

// 网络通信
zf_driver_tcp_client tcp_client;

// 定时器
zf_driver_pit control_timer;

// ========== 全局变量 ==========
struct pwm_info motor_info;
int16 encoder_left_count = 0;
int16 encoder_right_count = 0;
int16 imu_acc_x = 0, imu_acc_y = 0, imu_acc_z = 0;
int16 imu_gyro_x = 0, imu_gyro_y = 0, imu_gyro_z = 0;

float target_speed_left = 0;    // 左轮目标速度
float target_speed_right = 0;   // 右轮目标速度
float current_speed_left = 0;   // 左轮当前速度
float current_speed_right = 0;  // 右轮当前速度

// PID参数
float kp = 0.5, ki = 0.1, kd = 0.05;
float error_left = 0, error_left_last = 0, error_left_sum = 0;
float error_right = 0, error_right_last = 0, error_right_sum = 0;

// ========== 电机控制函数 ==========
void set_motor_duty(zf_driver_pwm &pwm, zf_driver_gpio &dir, int16 duty)
{
    if(duty >= 0)
    {
        dir.set_level(1);
        pwm.set_duty((uint16)duty);
    }
    else
    {
        dir.set_level(0);
        pwm.set_duty((uint16)(-duty));
    }
}

// ========== PID速度控制 ==========
int16 pid_control(float target, float current, float &error, float &error_last, float &error_sum)
{
    error = target - current;
    error_sum += error;
    
    // 积分限幅
    if(error_sum > 100) error_sum = 100;
    if(error_sum < -100) error_sum = -100;
    
    float output = kp * error + ki * error_sum + kd * (error - error_last);
    error_last = error;
    
    // 输出限幅
    if(output > motor_info.duty_max) output = motor_info.duty_max;
    if(output < -motor_info.duty_max) output = -motor_info.duty_max;
    
    return (int16)output;
}

// ========== 10ms控制周期回调函数 ==========
void control_callback()
{
    // 读取编码器
    encoder_left_count = encoder_left.get_count();
    encoder_right_count = encoder_right.get_count();
    encoder_left.clear_count();
    encoder_right.clear_count();
    
    // 计算当前速度（脉冲/10ms）
    current_speed_left = encoder_left_count;
    current_speed_right = encoder_right_count;
    
    // PID速度控制
    int16 duty_left = pid_control(target_speed_left, current_speed_left, 
                                   error_left, error_left_last, error_left_sum);
    int16 duty_right = pid_control(target_speed_right, current_speed_right, 
                                    error_right, error_right_last, error_right_sum);
    
    // 设置电机PWM
    set_motor_duty(motor_left_pwm, motor_left_dir, duty_left);
    set_motor_duty(motor_right_pwm, motor_right_dir, duty_right);
    
    // 读取IMU数据
    imu_acc_x = imu_dev.get_acc_x();
    imu_acc_y = imu_dev.get_acc_y();
    imu_acc_z = imu_dev.get_acc_z();
    imu_gyro_x = imu_dev.get_gyro_x();
    imu_gyro_y = imu_dev.get_gyro_y();
    imu_gyro_z = imu_dev.get_gyro_z();
}

// ========== 网络数据发送 ==========
void send_telemetry_data()
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "Speed:L=%d,R=%d;Enc:L=%d,R=%d;IMU:ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d\n",
             (int)current_speed_left, (int)current_speed_right,
             encoder_left_count, encoder_right_count,
             imu_acc_x, imu_acc_y, imu_acc_z,
             imu_gyro_x, imu_gyro_y, imu_gyro_z);
    
    tcp_client.send_data((uint8*)buffer, strlen(buffer));
}

// ========== 屏幕显示更新 ==========
void update_display()
{
    ips200.clear();
    ips200.show_string(0, 0, "Smart Car System");
    
    ips200.show_string(0, 20, "Speed L:");
    ips200.show_int(80, 20, (int)current_speed_left, 4);
    
    ips200.show_string(0, 40, "Speed R:");
    ips200.show_int(80, 40, (int)current_speed_right, 4);
    
    ips200.show_string(0, 60, "Accel X:");
    ips200.show_int(80, 60, imu_acc_x, 5);
    
    ips200.show_string(0, 80, "Gyro Z:");
    ips200.show_int(80, 80, imu_gyro_z, 5);
}

// ========== 主函数 ==========
int main()
{
    printf("========== Smart Car System ==========\n");
    
    // 初始化PWM
    motor_left_pwm.get_dev_info(&motor_info);
    
    // 初始化IMU
    imu_dev.init();
    printf("IMU initialized\n");
    
    // 初始化屏幕
    ips200.init(FB_PATH);
    printf("Display initialized\n");
    
    // 初始化TCP客户端（可选）
    if(tcp_client.init("192.168.2.49", 8086) == 0)
    {
        printf("TCP client connected\n");
    }
    
    // 启动控制定时器
    control_timer.init_ms(10, control_callback);
    printf("Control timer started (10ms period)\n");
    
    uint32 loop_count = 0;
    
    while(1)
    {
        // 每100ms更新一次屏幕
        if(loop_count % 10 == 0)
        {
            update_display();
        }
        
        // 每500ms发送一次遥测数据
        if(loop_count % 50 == 0)
        {
            send_telemetry_data();
        }
        
        // 示例：简单的速度控制逻辑
        // 实际应用中可从网络接收控制指令
        if(loop_count < 500)
        {
            target_speed_left = 50;
            target_speed_right = 50;
        }
        else if(loop_count < 1000)
        {
            target_speed_left = 30;
            target_speed_right = 70;  // 右转
        }
        else
        {
            target_speed_left = 0;
            target_speed_right = 0;
            loop_count = 0;
        }
        
        loop_count++;
        system_delay_ms(10);
    }
    
    return 0;
}
```

**系统特点**：
1. **多传感器融合**：编码器测速 + IMU姿态检测
2. **闭环控制**：PID速度控制算法
3. **实时显示**：屏幕显示运行状态
4. **远程通信**：TCP发送遥测数据
5. **定时任务**：10ms控制周期，保证实时性
6. **模块化设计**：各功能独立，易于扩展

**应用扩展方向**：
- 添加摄像头实现视觉巡线
- 接入遥控器实现远程控制
- 增加避障传感器
- 实现路径规划算法
- 数据记录和回放功能

---

---

## 许可证

本库使用 **GPL 3.0** 开源许可证  
版权所有 © 2022-2026 SEEKFREE 逐飞科技  
官方网站：https://seekfree.taobao.com/

---

**文档完成时间**：2026年1月25日  
**库版本**：LS2K0300 Opensource Library 1.0  
**适用平台**：LS2K0300 龙芯处理器  
**开发语言**：C/C++
