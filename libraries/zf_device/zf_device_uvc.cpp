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
* 2026-01-14        补充           添加自动曝光相关接口
********************************************************************************************************************/

#include "zf_device_uvc.hpp"

// 函数简介 等待摄像头真正就绪
// 参数说明 timeout_ms 超时时间（毫秒）
// 返回参数 bool true-摄像头就绪 false-超时
// 备注信息 内部使用，确保摄像头可以读取帧
//-------------------------------------------------------------------------------------------------------------------
bool zf_device_uvc::wait_camera_ready(int timeout_ms) {
    if (!cap.isOpened()) {
        return false;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        // 检查超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count();
        
        if (elapsed > timeout_ms) {
            printf("等待摄像头就绪超时 (%d ms)\n", timeout_ms);
            return false;
        }
        
        // 尝试读取一帧
        cv::Mat test_frame;
        if (cap.read(test_frame) && !test_frame.empty()) {
            printf("摄像头真正就绪，耗时 %ld ms\n", elapsed);
            // 将读取的帧保存下来，避免浪费
            frame_mjpg = test_frame;
            return true;
        }
        
        // 等待一段时间再试
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数，初始化成员变量
// 参数说明 无
// 返回参数 无
// 使用示例 zf_device_uvc uvc_obj;
// 备注信息 初始化指针为空，摄像头状态为未打开
//-------------------------------------------------------------------------------------------------------------------
zf_device_uvc::zf_device_uvc()
    : gray_image(nullptr), rgb_image(nullptr), is_opened(false)
{

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数，释放摄像头资源
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用，无需手动调用
// 备注信息 摄像头打开时才执行释放操作，防止重复释放
//-------------------------------------------------------------------------------------------------------------------
zf_device_uvc::~zf_device_uvc()
{
    if(cap.isOpened())
    {
        cap.release();
        is_opened = false;
        gray_image = nullptr;
        rgb_image = nullptr;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 阻塞式等待刷新图像帧
// 参数说明 无
// 返回参数 int8 0-获取图像成功  -1-获取图像失败/帧为空/未初始化
// 使用示例 int8 res = uvc_obj.wait_image_refresh();
// 备注信息 仅采集MJPG格式图像帧，不做格式转换，提高采集效率
//-------------------------------------------------------------------------------------------------------------------
int8 zf_device_uvc::wait_image_refresh()
{
    if(!is_opened)
    {
        std::cerr << "camera not init, can not get frame!" << std::endl;
        return -1;
    }

    try 
    {
        cap >> frame_mjpg;
        if (frame_mjpg.empty()) 
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            gray_image = nullptr;
            rgb_image = nullptr;
            return -1;
        }
    } 
    catch (const cv::Exception& e) 
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        gray_image = nullptr;
        rgb_image = nullptr;
        return -1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取灰度图像数据指针
// 参数说明 无
// 返回参数 uint8_t* 灰度图像首地址指针，NULL-未获取到有效图像
// 使用示例 uint8_t *p_img = uvc_obj.get_gray_image_ptr();
// 备注信息 内部完成BGR转灰度，指针指向灰度图首地址，数据格式为uint8灰度值
//-------------------------------------------------------------------------------------------------------------------
uint8_t* zf_device_uvc::get_gray_image_ptr()
{
    cv::cvtColor(frame_mjpg, frame_gray, cv::COLOR_BGR2GRAY);
    gray_image = reinterpret_cast<uint8_t*>(frame_gray.ptr(0));

    return gray_image;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取RGB彩色图像数据指针
// 参数说明 无
// 返回参数 uint16_t* RGB彩色图像首地址指针，NULL-未获取到有效图像
// 使用示例 uint16_t *p_img = uvc_obj.get_rgb_image_ptr();
// 备注信息 内部完成BGR转BGR565，指针指向彩色图首地址，数据格式为uint16三通道连续存储
//-------------------------------------------------------------------------------------------------------------------
uint16_t* zf_device_uvc::get_rgb_image_ptr()
{
    cv::cvtColor(frame_mjpg, frame_rgb, cv::COLOR_BGR2BGR565);
    rgb_image = reinterpret_cast<uint16_t*>(frame_rgb.ptr(0));

    return rgb_image;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取摄像头当前的打开状态
// 参数说明 无
// 返回参数 bool true-已打开  false-未打开
// 使用示例 bool status = uvc_obj.isCameraOpened();
// 备注信息 直接返回摄像头状态标志位，非阻塞快速查询
//-------------------------------------------------------------------------------------------------------------------
bool zf_device_uvc::is_camera_opened() const
{
    return is_opened;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置自动曝光模式（新增实现）
// 参数说明 auto_exposure_mode 自动曝光模式（推荐使用宏：UVC_AUTO_EXPOSURE_ENABLE/UVC_AUTO_EXPOSURE_DISABLE）
// 返回参数 int8 0-设置成功  -1-设置失败/摄像头未打开
// 使用示例 int8 res = uvc_obj.set_auto_exposure(UVC_AUTO_EXPOSURE_DISABLE);
// 备注信息 适配LS2K0300平台，3=开启自动曝光，1=关闭自动曝光（手动模式）
//-------------------------------------------------------------------------------------------------------------------
int8 zf_device_uvc::set_auto_exposure(int32_t auto_exposure_mode)
{
    // 检查摄像头是否已打开
    if(!is_opened || !cap.isOpened())
    {
        std::cerr << "camera not opened, can not set auto exposure!" << std::endl;
        return -1;
    }

    try
    {
        // 设置自动曝光模式
        bool set_result = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, static_cast<double>(auto_exposure_mode));
        if(!set_result)
        {
            std::cerr << "set auto exposure mode failed!" << std::endl;
            return -1;
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置手动曝光值（新增实现）
// 参数说明 exposure_value 曝光值（范围：0-1000，具体以摄像头支持为准，默认使用UVC_DEFAULT_EXPOSURE）
// 返回参数 int8 0-设置成功  -1-设置失败/摄像头未打开/处于自动曝光模式
// 使用示例 int8 res = uvc_obj.set_exposure_value(UVC_DEFAULT_EXPOSURE);
// 备注信息 仅在手动曝光模式下生效，自动曝光模式下此设置无效
//-------------------------------------------------------------------------------------------------------------------
int8 zf_device_uvc::set_exposure_value(int32_t exposure_value)
{
    // 检查摄像头是否已打开
    if(!is_opened || !cap.isOpened())
    {
        std::cerr << "camera not opened, can not set exposure value!" << std::endl;
        return -1;
    }

    // 检查当前是否为手动曝光模式（可选：增加校验，提高鲁棒性）
    double current_auto_mode = get_auto_exposure_mode();
    if(current_auto_mode == UVC_AUTO_EXPOSURE_ENABLE)
    {
        std::cerr << "current is auto exposure mode, can not set fixed exposure value!" << std::endl;
        return -1;
    }

    try
    {
        // 设置手动曝光值
        bool set_result = cap.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(exposure_value));
        if(!set_result)
        {
            std::cerr << "set exposure value failed!" << std::endl;
            return -1;
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前自动曝光模式（新增实现）
// 参数说明 无
// 返回参数 double 当前自动曝光模式值（-1.0=获取失败/摄像头未打开）
// 使用示例 double mode = uvc_obj.get_auto_exposure_mode();
// 备注信息 用于验证自动曝光模式是否设置成功
//-------------------------------------------------------------------------------------------------------------------
double zf_device_uvc::get_auto_exposure_mode()
{
    // 检查摄像头是否已打开
    if(!is_opened || !cap.isOpened())
    {
        std::cerr << "camera not opened, can not get auto exposure mode!" << std::endl;
        return -1.0;
    }

    try
    {
        return cap.get(cv::CAP_PROP_AUTO_EXPOSURE);
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1.0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前曝光值（新增实现）
// 参数说明 无
// 返回参数 double 当前曝光值（-1.0=获取失败/摄像头未打开）
// 使用示例 double value = uvc_obj.get_current_exposure();
// 备注信息 手动模式下返回设置的曝光值，自动模式下返回摄像头自动调节的值
//-------------------------------------------------------------------------------------------------------------------
double zf_device_uvc::get_current_exposure()
{
    // 检查摄像头是否已打开
    if(!is_opened || !cap.isOpened())
    {
        std::cerr << "camera not opened, can not get current exposure value!" << std::endl;
        return -1.0;
    }

    try
    {
        return cap.get(cv::CAP_PROP_EXPOSURE);
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1.0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 UVC摄像头初始化配置
// 参数说明 path 摄像头设备节点路径，例如："/dev/video0"
// 返回参数 int8 0-初始化成功  -1-初始化失败
// 使用示例 int8 res = uvc_obj.uvc_camera_init("/dev/video0");
// 备注信息 配置MJPG格式、指定分辨率、帧率，打开摄像头设备
//-------------------------------------------------------------------------------------------------------------------
int8 zf_device_uvc::init(const char *path)
{
    printf("find uvc camera Successfully.\n");
    
    // 关闭之前可能打开的摄像头
    if (cap.isOpened()) {
        cap.release();
        is_opened = false;
    }
    
    // 重置图像缓存
    frame_mjpg = cv::Mat();
    frame_gray = cv::Mat();
    frame_rgb = cv::Mat();
    
    // 使用V4L2后端显式打开
    cap.open(path, cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
        printf("camera not opened!\n");
        return -1;
    }else
    {
        is_opened = true;
    }
    
    
    // 设置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, UVC_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, UVC_HEIGHT);
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    cap.set(cv::CAP_PROP_FPS, UVC_FPS);
    
    // 优先尝试MJPG格式
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    // 等待摄像头真正就绪（关键修复）
    printf("等待摄像头就绪...\n");
    if (!wait_camera_ready(5000)) {
        printf("警告：摄像头未能完全就绪，但继续尝试\n");
    }
    
    // 验证参数设置
    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    
    printf("get uvc width = %.0f\n", width);
    printf("get uvc height = %.0f\n", height);
    printf("get uvc fps = %.0f\n", fps);
    
    // 尝试设置曝光（但不要因为失败而返回错误）
    int8 exposure_result = set_exposure_value(UVC_DEFAULT_EXPOSURE);
    if (exposure_result != 0) {
        printf("曝光设置警告（非致命错误），继续初始化\n");
    }
    
    // 获取当前曝光模式
    double exposure_mode = get_auto_exposure_mode();
    printf("get uvc auto exposure mode = %.0f\n", exposure_mode);
    
    // 确保至少有一帧图像
    if (frame_mjpg.empty()) {
        if (!cap.read(frame_mjpg) || frame_mjpg.empty()) {
            printf("警告：无法读取初始帧\n");
            // 不返回错误，继续尝试
        }
    }
    
    is_opened = true;
    return 0;
}