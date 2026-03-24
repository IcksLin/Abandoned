#ifndef __RED_RECT_HPP__
#define __RED_RECT_HPP__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "tflm_model_process.hpp"

using namespace cv;
using namespace std;

// --- 基础参数定义 ---
#define MAZE_MAX_LEN 1000     // 迷宫追踪最大步数

class RedRectDetector {
public:
    RedRectDetector(float r_threshold = 0.48f);
    ~RedRectDetector();

    Mat target_roi;         // 存储裁剪后的图像
    Rect target_rect;       // 存储裁剪方框在原图中的位置(x, y, width, height)
    
    /**
     * @brief 获取当前识别到的目标方框位置
     * @return Rect 返回矩形区域，若未识别到，则各字段为 0
     */
    Rect get_target_location() { return target_rect; }

    /**
     * @brief 提取红色方框区域内的ROI
     * @param img 输入原始图像
     * @param roi 输出裁剪后的图像
     * @param is_draw 是否在原图上绘制绿色识别框
     * @return bool: true 表示成功识别并裁剪，false 表示未找到或裁剪失败
     */
    bool model_roi_cut(Mat& img, Mat& roi, bool is_draw = false);

private:
    float r_thres;
    
    // 方向步进定义 (迷宫寻路算法)
    const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    /**
     * @brief 判定是否为红色点
     */
    bool is_red(Vec3b color);

    /**
     * @brief 寻找方框边界的种子点
     */
    void find_seed_point(int* out_x, int* out_y, Mat& img);

    /**
     * @brief 沿边缘走迷宫算法，计算方框中心
     */
    void maze_walk(int x, int y, Mat& frame, int* x_avg, int* y_avg);
};

#endif