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
//角点识别
typedef enum {
    LOST_PT = 0x01,    //丢线     000001
    S_PT    = 0x02,    //直道     000010
    T_PT    = 0x04,    //弯道     000100
    L_PT    = 0x08,    //L角点    001000
    Y_PT    = 0x10,    //Y角点    010000
    INVAL_PT= 0x20,    //无效角点 100000

    //角点组掩码
    SUPPABLE_PT = L_PT | Y_PT,                  //可补线的边线--L角点 Y角点
    NORMAL_PT   = S_PT | T_PT,                  //无元素的普通边线--直道 弯道
    TRABLE_PT   = S_PT | T_PT | L_PT | Y_PT,    //可巡线的边线--直到 弯道 L角点 Y角点
    INTRABLE_PT = LOST_PT | INVAL_PT            //不可巡线的边线--丢失 无效
} PT_Judge_TypeDef;

typedef struct{
    PT_Judge_TypeDef pt,per_pt;
    uint8 pt_cnt;

    void pt_identify(PT_Judge_TypeDef &out, float nms_val, int linelen);
} PT_Identify_Typedef;

//状态机
typedef struct{
    char id;
    uint8 flag;
    void (*func)(void); // 函数指针
} ElementFlag_TypeDef;

//环岛内部状态机
typedef enum{
    init_circle=0,
    begin_circle,
    into_circle,
    in_circle,
    out_circle,
    outof_circle
}Circle_Flag_TypeDef;


#define PI 3.14159265358979323846f
#define clip(value, low, high) \
    ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

/*---------------------图像参数宏定义---------------------*/
#define IMG_W               160             //图像宽
#define IMG_H               120             //图像高

/*---------------------边线参数宏定义---------------------*/
#define POINTS_MAX_LEN      100             //巡线最大长度
#define TRACK_HEIGHT_MAX    40              //迷宫巡线最大高度
#define CAR_IMGAGE_W        40                        
#define CHECK_DIS           6               //检测是否为噪音的距离

#define M2PIX               100             //米转像素  
#define sampled_dist        0.02f           //重采样距离
#define ROAD_W              0.45f           //道路宽度
#define angle_idx           10              //角度采样索引距离
#define approx_idx          15              //平移采样索引距离

/*-------------------------角点识别参数-------------------------*/
//turn在直线和直角之间 Ypt在Ltp_up上
#define STRAIGHT_UP             15
#define LPT_LOW                 75
#define LPT_UP                  93

#define LINE_LOST_LENGTH        16          /* 丢线阈值 sample_line */
#define LINE_ST_MIN_LEN         25          /* 直道最小长度 sample_line */

#define __JUDGE_CNT             2           /* 判断次数阈值 从1开始 等于时切换 */ 


/*------------------------正常巡线状态参数------------------------*/
#define __N_DFT_LEN_THRE        30          /* 切换巡线长度差阈值 sample_line*/

/*-------------------------右圆环状态参数-------------------------*/
#define __RC_BEGIN_LOSE_THRE    1           /* 驶入圆环阶段 左边线丢失计数阈值 从0开始 等于时有效*/
#define __RC_OUT_FIXED_ANGLE    20          /* 驶出圆环阶段 左边丢失 给固定角度 */

/*------------------------预瞄点参数宏定义------------------------*/
#define AIM_IDX                 30          //预瞄点索引


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

void supplement_line(float pts_in[][2],int* num,int corner_index,float dist);   //补线
void load_undistort_map(void);
#endif