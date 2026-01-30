#ifndef _IMGPROC_HPP__
#define _IMGPROC_HPP_
#include "zf_common_typedef.hpp"
#include "zf_device_uvc.hpp"
#include <iostream>      
#include <fstream>       
#include <iomanip>       
#include <string>       
#include <opencv2/opencv.hpp>
#include <cmath>

extern zf_device_uvc uvc;


#define PI 3.14159265358979323846f
#define clip(value, low, high) \
    ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// 基本图像参数
#define IMG_W               160             // 图像宽
#define IMG_H               120             // 图像高

#define BLACK               0
#define WHITE               255
#define BORDER              0

//巡线宏
#define PROHIBITED_AREA_WITH_X  70            //视野中有轮胎，故设置禁止区域
#define FIND_SEED_START_X       IMG_W/2
#define FIND_RIGHT_START_X      86
#define FIND_LEFT_START_X       55
#define FIND_START_Y            IMG_H-3
#define FIND_END_Y              2*IMG_H/3
#define LEFT_BOUNDARY           2//图像左边界
#define RIGHT_BOUNDARY          158//图像右边界
// 扫线状态枚举
typedef enum {
    SEARCH_OK = 0,
    SEARCH_FAIL = 10
} SearchResult;

/*------------------------其他宏定义------------------------*/
#define IMG_AT(img, x, y)   (img[(y) * (IMG_W) + (x)])     // 用于访问图像数据
/*--图片去畸映射表（由load_undistort_map初始化）--*/
extern cv::Mat ud_map_cv;
//去畸变后Mat
extern cv::Mat De_distortion_image;
/*去畸变矩阵*/
extern float undistort_map_x[IMG_H][IMG_W];
extern float undistort_map_y[IMG_H][IMG_W];
/*-------------------------全局变量-------------------------*/
extern cv::Mat frame_color;         // 用于处理的图像帧
extern cv::Mat frame_gray;          // 灰度图像帧
extern cv::Mat frame_bin;           // 二值图像帧
extern uint8_t* img_gray;           // 灰度图像指针
extern uint8_t img_bin[IMG_W * IMG_H]; // 二值化图像

extern uint8_t l_border[IMG_H];    // 左线数组
extern uint8_t r_border[IMG_H];    // 右线数组
extern uint8_t center_line[IMG_H]; // 中线数组
extern uint8_t lukuan[IMG_H];    // 路宽数组
extern uint8_t top_point;              // 赛道搜索到的最高点（最远点）所在的行号
extern uint8_t left_lenth; // 左侧边界长度
extern uint8_t right_lenth; // 右侧边界长度
// void save_per_map(void);
extern float onto;



uint8 get_otsu_thres(uint8 *img, int x0, int x1, int y0, int y1);
void image_binarization(uint8_t* input_img, uint8_t* output_image, uint8_t best_threshold);
void blur_line(uint8_t line_in[], int num, uint8_t line_out[], int kernel_size);
SearchResult processTrackSearch(uint8_t* img_ptr,uint8_t image_thereshold);

void load_undistort_map(void);

void get_image();
float calculate_weighted_offset_angle();
#endif