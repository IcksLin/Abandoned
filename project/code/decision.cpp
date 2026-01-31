#include "decision.hpp"
/******************其他变量*/
std::atomic<bool> circle_flag(0);      //圆环减速标志位
/*其他变量******************/

/*--------------------------功能函数----------------------------*/

// //去畸变后Mat
// cv::Mat De_distortion_image;

// // 一次图像处理
// void image_proc(){   
//     /*----------图像处理----------*/
//     // 等待采样数据
//     uvc.wait_image_refresh();
//     cv::remap(
//             uvc.frame_mjpg,            // 输入畸变图像
//             De_distortion_image,                   // 输出去畸变图像
//             ud_map_cv,                  // 转换后的OpenCV整型映射表
//             cv::Mat(),
//             cv::INTER_NEAREST,        // 最近邻插值
//             cv::BORDER_REPLICATE      // 边界填充
//         );
//     cv::cvtColor(De_distortion_image, frame_gray, cv::COLOR_BGR2GRAY);

//     img_gray = reinterpret_cast<uint8_t*>(frame_gray.ptr(0));
//     start_thre = get_otsu_thres(img_gray,0,IMG_W,TRACK_HEIGHT_MAX,IMG_H);      // 二值化
//     line_process(IMG_H,IMG_H/2);
//     // sta_decision();

// }