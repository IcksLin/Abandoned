#include "red_rect.hpp"

RedRectDetector::RedRectDetector(float r_threshold) {
    r_thres = r_threshold;
}

RedRectDetector::~RedRectDetector() {}

bool RedRectDetector::is_red(Vec3b color) {
    int b = color[0];
    int g = color[1];
    int r = color[2];
    float total = (float)(b + g + r);
    if (total != 0) {
        float r_ratio = r / total;
        if (r_ratio > r_thres) return true;
    }
    return false;
}

void RedRectDetector::find_seed_point(int* out_x, int* out_y, Mat& img) {
    // 搜索范围：高度 20%~80%，宽度 20%~80% (避免边缘噪声)
    int h_start = (int)(img.rows * 0.8);
    int h_end = (int)(img.rows * 0.2);
    int w_start = (int)(img.cols * 0.2);
    int w_end = (int)(img.cols * 0.8);

    for (int y = h_start; y > h_end; y--) {
        for (int x = w_start; x < w_end; x++) {
            Vec3b color = img.at<Vec3b>(y, x);
            if (is_red(color)) {
                // 简单的去噪检查：上方连续3个像素是否也是红色
                bool is_noise = false;
                if (y < 3) continue; 
                for (int val_y = y; val_y > y - 3; val_y--) {
                    if (!is_red(img.at<Vec3b>(val_y, x))) {
                        is_noise = true;
                        break;
                    }
                }
                if (!is_noise) {
                    *out_x = x;
                    *out_y = y;
                    return;
                }
            }
        }
    }
    *out_x = 0; *out_y = 0;
}

void RedRectDetector::maze_walk(int x, int y, Mat& frame, int* x_avg, int* y_avg) {
    if (x == 0 && y == 0) return;
    int step = 0, dir = 0, turn = 0;
    int min_x = x, max_x = x, start_x = x;
    int min_y = y, max_y = y, start_y = y;

    while (step < MAZE_MAX_LEN && x > 0 && x < frame.cols - 1 && y > 0 && y < frame.rows - 1 && turn < 4) {
        int front_x = x + dir_front[dir][0];
        int front_y = y + dir_front[dir][1];
        int frontleft_x = x + dir_frontleft[dir][0];
        int frontleft_y = y + dir_frontleft[dir][1];

        if (is_red(frame.at<Vec3b>(front_y, front_x))) {
            dir = (dir + 1) % 4;
            turn++;
        } else {
            if (is_red(frame.at<Vec3b>(frontleft_y, frontleft_x))) {
                x = front_x;
                y = front_y;
            } else {
                x = frontleft_x;
                y = frontleft_y;
                dir = (dir + 3) % 4;
            }
            step++;
            turn = 0;
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
        }
        // 闭环检查：如果回到起点附近则停止
        if (step > 10) {
            if (abs(start_x - x) <= 2 && abs(start_y - y) <= 2) break;
        }
    }
    *x_avg = (min_x + max_x) / 2;
    *y_avg = (min_y + max_y) / 2;
}

/**
 * @brief 提取红色方框区域内的ROI
 * @return bool: true 表示成功识别并裁剪，false 表示未找到或裁剪失败
 */
bool RedRectDetector::model_roi_cut(Mat& img, Mat& roi, bool is_draw) {
    int seed_x = 0, seed_y = 0, x_avg = 0, y_avg = 0;

    // 1. 寻找种子点
    find_seed_point(&seed_x, &seed_y, img);
    if (seed_x == 0 && seed_y == 0) {
        target_rect = Rect(0, 0, 0, 0); // 未找到时清空位置信息
        return false;
    }

    // 2. 迷宫寻路计算中心
    maze_walk(seed_x, seed_y, img, &x_avg, &y_avg);
    if (x_avg == 0 && y_avg == 0) {
        target_rect = Rect(0, 0, 0, 0);
        return false;
    }
    
    int roi_w = MODEL_INPUT_WIDTH;
    int roi_h = MODEL_INPUT_WIDTH;

    // 3. 计算裁剪区域
    int x1 = x_avg - roi_w / 2;
    int x2 = x_avg + roi_w / 2;
    int y1 = y_avg - roi_h * 3 / 4; 
    int y2 = y_avg + roi_h / 4;

    // 4. 边界检查
    x1 = max(0, x1);
    x2 = min(img.cols - 1, x2);
    y1 = max(0, y1);
    y2 = min(img.rows - 1, y2);

    if (x2 <= x1 || y2 <= y1) {
        target_rect = Rect(0, 0, 0, 0);
        return false;
    }

    // 更新 target_rect 接口数据
    target_rect = Rect(x1, y1, x2 - x1, y2 - y1);

    // 5. 绘制结果
    if (is_draw) {
        rectangle(img, target_rect, Scalar(0, 255, 0), 2);
    }

    // 6. 提取 ROI
    roi = img(target_rect).clone();
    return true; 
}