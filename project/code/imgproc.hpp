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
#include "my_timestamp.hpp"
#include "udp_sender.hpp"
#include "zf_device_uvc.hpp"

extern zf_device_uvc uvc;
//图像处理变量
extern float onto;                  // 最终处理方向，已限制幅度在-30~30.0f之间
extern float angle_compensation;    // 方向补偿量(静态最中间
extern int middle_line_length; // 中线长度
extern float max_angle;        // 最大角点值,用于调试

extern udp_sender udp;

#define STATE_TIME_LOCKING     500              //状态时间锁定，防止抖动,循环周期改动之后记得调整
//巡线决策机
typedef struct{
    float max_L_angle;          //左边线最大角点值
    float max_R_angle;          //右边线最大角点值
    float max_angle;            //最大角点值
    

    uint8_t state;                    //当前状态，0直道，1十字，2左圆环，3右圆环
    uint8_t element_processing_flage; //处理位，检测到元素后置1，处理完成后清0，防止重复检测状态位乱飞
    uint32_t state_time_locking;

    uint8_t longest_side;        //最长边线 0-左边线 1-右边线
    uint8_t left_length;          //左边线长度
    uint8_t right_length;         //右边线长度

    uint8_t target_boundary;     //最终巡线 0-左边线 1-右边线

    // --- 冷却时间相关 ---
    TimerClockGetTime cooldown_timer; // 用于记录冷却时间的计时器
    bool is_cooling;                  // 冷却状态标志
    float cooldown_threshold_sec;    // 冷却时间阈值（默认5.0s）
} Tracking_Decision_Machine_TypeDef;
//圆环状态机
typedef struct {
    uint8_t state;              //0初始状态，1进圆环路口，2圆环路口未进环岛，3环岛中无岔路部分，4出环岛路口
    uint8_t state_locking;      //状态锁定，防止重判乱飞,0未上锁，1上锁
    uint8_t side;               //圆环边，2左边，3右边，0初始
    float start_angle;          //入环时角度
    float current_angle; 
} Circle_Tracking_Machine_TypeDef;


#define PI 3.14159265358979323846f
#define clip(value, low, high) \
    ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

/*---------------------图像参数宏定义---------------------*/
// #define IMG_W               UVC_WIDTH            //图像宽
// #define IMG_H               UVC_HEIGHT           //图像高

#define IMG_W               160            //图像宽
#define IMG_H               120           //图像高

/*---------------------边线参数宏定义---------------------*/
#define POINTS_MAX_LEN      100             //巡线最大长度
#define TRACK_HEIGHT_MAX    IMG_H/3              //迷宫巡线最大高度
#define CAR_IMGAGE_W        IMG_W*4/8                        
#define CHECK_DIS           6               //检测是否为噪音的距离

#define M2PIX               100             //米转像素  
#define sampled_dist        0.02f           //重采样距离
#define ROAD_W              0.45f           //道路宽度
#define angle_idx           10              //角度采样索引距离
#define approx_idx          15              //平移采样索引距离

/*-------------------------角点识别参数-------------------------*/
#define CORNER_ANGLE_THRE      75.0f           //角点角度阈值
#define CORNER_THRE_MAX        96.0f          //十字角点最大阈值
#define CIRCLE_ANGLE_THRE      100.0f          //圆环角度阈值
#define STRAIGHT_ROAD_THRE     50               
#define LOST_LINE              10
/*------------------------正常巡线状态参数------------------------*/
#define __N_DFT_LEN_THRE        30          /* 切换巡线长度差阈值 sample_line*/
/*------------------------预瞄点参数宏定义------------------------*/


/*------------------------其他宏定义------------------------*/
#define IMG_AT(img, x, y)   (img[(y) * (IMG_W) + (x)])     // 用于访问图像数据
extern cv::Mat ud_map_cv;
/*去畸变矩阵*/
extern float undistort_map_x[IMG_H][IMG_W];
extern float undistort_map_y[IMG_H][IMG_W];
/*-------------------------全局变量-------------------------*/
extern cv::Mat frame_color;         // 用于处理的图像帧
extern cv::Mat frame_gray;          // 灰度图像帧
extern cv::Mat frame_bin;           // 二值图像帧
extern uint8_t* img_gray;           // 灰度图像指针

extern uint8 all_block_size,start_thre;    //调整参数
extern float avg_angle;

//预瞄点结构体
typedef struct{
    bool flag;      //预瞄点标志位 决定是否巡线
    float angle;    //预瞄点偏差角
    int idx;       //预瞄点索引
}AimPoint_TypeDef;

/**************图像处理变量*/
extern int Lline_num, Rline_num;
extern float per_Lline[][2], per_Rline[][2];            // 透视边线
extern float blurred_Lline[][2], blurred_Rline[][2];    // 滤波
extern int sampled_Lline_num, sampled_Rline_num;
extern int Mline_num;
// 边线数组声明
extern int Lline[][2], Rline[][2];                      // 原始边线
extern float sampled_Lline[][2],sampled_Rline[][2];     // 等距采样
extern float L2Mline[][2], R2Mline[][2];                // 左右得到的中线
extern float (*Mline)[2];                               // 最终中线
// 非极大抑制
extern float nms_Lline,nms_Rline;          // 角点值
extern int nms_Lline_idx,nms_Rline_idx;    // 索引
extern AimPoint_TypeDef aim_point; 
/*图像处理变量**************/

void image_proc();

void* img_consume(void* args);

uint8 get_otsu_thres(uint8 *img, int x0, int x1, int y0, int y1);
void save_per_map(void);

void point_per(const cv::Mat& M, float x, float y, int& x_out, int& y_out);

void line_process(uint8_t height_start, uint8_t height_min);
void search_Lline(int height_start, int height_min);
void search_Rline(int height_start, int height_min);
void perspective_transform_points(int pts_in[][2],int num,float pts_out[][2]);
void blur_points(float pts_in[][2], int num, float pts_out[][2]);   //三角滤波
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2);   //点集等距采样
void local_angle_points(float pts_in[][2], int num, float angle_out[]);   //点集局部角度变化率
void nms_angle(float angle_in[], int num, float *angle_max, int *idx);   //角度变化率非极大抑制
void track_leftline(float dist=ROAD_W*M2PIX/2);    //从左边线跟踪中线
void track_rightline(float dist=ROAD_W*M2PIX/2);   //从右边线跟踪中线 

void element_status();
void no_element_process();
void crossing_process();
void circle_process();
void auto_tracking();

void left_path_adjust(void);
void right_path_adjust(void);


void supplement_line(float pts_in[][2],int* num,int corner_index,float dist);   //补线
void load_undistort_map(void);
#endif