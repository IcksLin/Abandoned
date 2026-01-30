#include "decision.hpp"


// 一次图像处理
void image_proc(){   
    /*----------图像处理----------*/
    // 等待采样数据
    uvc.wait_image_refresh();
    cv::remap(
            uvc.frame_mjpg,            // 输入畸变图像
            De_distortion_image,                   // 输出去畸变图像
            ud_map_cv,                  // 转换后的OpenCV整型映射表
            cv::Mat(),
            cv::INTER_NEAREST,        // 最近邻插值
            cv::BORDER_REPLICATE      // 边界填充
        );
    cv::cvtColor(De_distortion_image, frame_gray, cv::COLOR_BGR2GRAY);

    img_gray = reinterpret_cast<uint8_t*>(frame_gray.ptr(0));
    uint8_t image_thereshold = get_otsu_thres(img_gray, 0, IMG_W - 1, 0, IMG_H - 1);
    image_binarization(img_gray, img_bin, image_thereshold);
    processTrackSearch(img_bin, image_thereshold);
    onto = calculate_weighted_offset_angle();
    onto = onto * 180.0f / 3.14159f; // 转为角度
}
/*-------------------------功能函数-----------------------------*/