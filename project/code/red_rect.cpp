/**
 * @file    red_rect.cpp
 * @brief   基于红色色块定位的目标图像动态裁剪器实现
 */

#include "red_rect.hpp"

// =============================================================================
// 构造 / 析构
// =============================================================================

RedRectDetector::RedRectDetector(int img_w, int img_h, float r_threshold)
    : target_rect(0, 0, 0, 0),   
    center_x(0),                
    center_y(0),                
    r_thres(r_threshold),       
    img_width(img_w),           
    img_height(img_h)                           
{
    // 默认有效区域覆盖全图，即不做任何位置限制
    valid_x_min = 0;
    valid_x_max = img_w;
    valid_y_min = 0;
    valid_y_max = img_h;
}

RedRectDetector::~RedRectDetector() {}

// =============================================================================
// 公开接口
// =============================================================================

/**
 * 将百分比边距转换为像素坐标并缓存，后续每帧只做整数比较，无浮点开销。
 *
 * 有效区域示意（left=0.2, right=0.2, top=0.1, bottom=0.1）：
 *
 *   0        20%                  80%      100%
 *   ┌─────────┬────────────────────┬─────────┐  0
 *   │  无效   │                    │  无效   │
 *   │         │    有效区域         │         │ 10%
 *   │         │                    │         │
 *   │         │                    │         │ 90%
 *   │  无效   │                    │  无效   │
 *   └─────────┴────────────────────┴─────────┘ 100%
 */
void RedRectDetector::set_valid_region(float left, float right, float top, float bottom) {
    valid_x_min = (int)(img_width  * left);
    valid_x_max = (int)(img_width  * (1.0f - right));
    valid_y_min = (int)(img_height * top);
    valid_y_max = (int)(img_height * (1.0f - bottom));
}

// =============================================================================
// 私有辅助函数
// =============================================================================

/**
 * 红色判定采用双重条件：
 *   1. 比例条件：R / (R+G+B) > r_thres，对光照变化有一定鲁棒性
 *   2. 绝对值条件：R > 80，排除总亮度极低时比例虚高的暗色噪点
 *   3. 总亮度条件：R+G+B > 30，排除纯黑像素
 *
 * 注意：目标图片内容可能含有红色物体（如红色急救包），但这些红色出现在
 * 图片上半部分，色块定位扫描从下往上进行，且要求连续多行高密度红色，
 * 单个红色物体不会触发色块确认条件。
 */
bool RedRectDetector::is_red(Vec3b color) {
    int b = color[0];
    int g = color[1];
    int r = color[2];
    int total = b + g + r;
    if (total < 30) return false;
    return (r / (float)total) > r_thres && r > 80;
}

/**
 * 逐像素扫描，统计指定行区间内红色像素总数。
 * 此函数是热路径，保持实现简单以利于编译器优化。
 */
int RedRectDetector::count_red_in_row(Mat& img, int y, int x_start, int x_end) {
    int cnt = 0;
    for (int x = x_start; x < x_end; x++) {
        if (is_red(img.at<Vec3b>(y, x))) cnt++;
    }
    return cnt;
}

/**
 * 红色色块定位算法（行扫描法）：
 *
 * 步骤一：自下而上逐行扫描（跳过最外侧 5% 边缘避免噪声）
 *   - 计算每行红色像素占扫描宽度的比例
 *   - 比例 >= RED_ROW_RATIO(0.45) 视为"红色行"
 *   - 连续 CONFIRM_ROWS(3) 行均为红色行时确认找到色块
 *   - 继续向上扫描直到红色行中断，记录完整的上下边界
 *   - 若连续性中断且未达到确认数，重置计数（过滤单行噪声）
 *
 * 步骤二：精确边界扫描
 *   - 在色块垂直中间行，从左向右找第一个红色像素 → 左边界
 *   - 从右向左找第一个红色像素 → 右边界
 *
 * 输出 block_bottom 为色块最底部红色行（整体裁剪的下基准），
 * 输出 block_top 为色块最顶部红色行（目标图片的下边界）。
 */
bool RedRectDetector::find_red_block(Mat& img,
                                     int* block_top, int* block_bottom,
                                     int* x_left,    int* x_right) {
    int scan_x_start = (int)(img.cols * 0.05f);
    int scan_x_end   = (int)(img.cols * 0.95f);
    int scan_width   = scan_x_end - scan_x_start;

    // 预计算每行红色像素占比
    vector<float> row_ratios(img.rows, 0.0f);
    for (int y = 0; y < img.rows; y++) {
        row_ratios[y] = count_red_in_row(img, y, scan_x_start, scan_x_end) / (float)scan_width;
    }

    // 阶段一：自下而上，找到连续 RED_CONFIRM_ROWS 行满足阈值的区域（确认色块底部）
    int confirmed_bottom = -1;
    int red_row_count    = 0;
    for (int y = img.rows - 1; y >= 0; y--) {
        if (row_ratios[y] >= RED_ROW_RATIO) {
            red_row_count++;
            if (red_row_count >= RED_CONFIRM_ROWS) {
                confirmed_bottom = y + RED_CONFIRM_ROWS - 1;
                break;
            }
        } else {
            red_row_count = 0;
        }
    }

    if (confirmed_bottom < 0) return false;

    // 阶段二：从确认底部向下补全真正的底边
    int blk_bottom = confirmed_bottom;
    for (int y = confirmed_bottom + 1; y < img.rows; y++) {
        if (row_ratios[y] >= RED_ROW_RATIO) blk_bottom = y;
        else break;
    }

    // 向上找顶部，允许最多 2 行间断（应对透视边缘不整齐）
    int blk_top = confirmed_bottom;
    int gap     = 0;
    for (int y = confirmed_bottom - 1; y >= 0; y--) {
        if (row_ratios[y] >= RED_ROW_RATIO) {
            blk_top = y;
            gap     = 0;
        } else {
            gap++;
            if (gap > 2) break;
        }
    }

    // 在色块中间行精确扫描左右边界
    int mid_y = (blk_top + blk_bottom) / 2;
    int lx = -1, rx = -1;

    for (int x = scan_x_start; x < scan_x_end; x++) {
        if (is_red(img.at<Vec3b>(mid_y, x))) { lx = x; break; }
    }
    for (int x = scan_x_end; x >= scan_x_start; x--) {
        if (is_red(img.at<Vec3b>(mid_y, x))) { rx = x; break; }
    }

    if (lx < 0 || rx < 0 || rx <= lx) return false;

    *block_top    = blk_top;
    *block_bottom = blk_bottom;
    *x_left       = lx;
    *x_right      = rx;
    return true;
}

// =============================================================================
// 主处理函数
// =============================================================================

/**
 * 完整处理流程：
 *
 * 目标图片结构（数据集与实际场景一致）：
 *
 *   ┌──────────────┐  ← 目标图片上边缘
 *   │  目标图片     │  正方形，边长 ≈ 色块宽度
 *   │  (白色背景)   │
 *   ├──────────────┤  ← 色块上边缘（block_top）= 目标图片下边缘
 *   │  红色色块     │  高度 = block_bottom - block_top
 *   └──────────────┘  ← 色块下边缘（block_bottom）= 整体下边界
 *
 * [1] find_red_block → 获取色块完整上下边界及左右边界
 *
 * [2] 动态正方形边长计算
 *       block_w    = x_right - x_left          （色块宽度 ≈ 目标图片宽度）
 *       block_h    = block_bottom - block_top   （色块高度，透视下不固定）
 *       total_h    = block_w + block_h           （目标图片高 + 色块高 = 整体高度）
 *       side       = max(total_h, block_w, MODEL_INPUT_WIDTH)
 *     取三者最大值确保裁剪框能同时容纳目标图片和色块。
 *
 * [3] 裁剪框定位
 *       水平：以色块水平中心为基准，左右各展开 side/2（居中对齐色块）
 *       垂直：下边界 = block_bottom（色块底部），上边界 = block_bottom - side
 *     这样色块始终贴近裁剪框底部，目标图片在上方，
 *     远距离时上方填充白色地板背景，与数据集采集效果一致。
 *
 * [4] 边界平移修正
 *     优先整体平移裁剪框而非单边截断，保持正方形比例不变形。
 *
 * [5] 有效区域校验
 *     裁剪框中心必须落在 set_valid_region 设定的范围内。
 *     超出时 center_x/center_y 仍会更新（供调用方打印调试），但返回 false。
 *
 * [6] 缩放输出
 *     side == MODEL_INPUT_WIDTH：直接 clone，跳过 resize
 *     side  > MODEL_INPUT_WIDTH：cv::resize + INTER_AREA（缩小无锯齿）
 */
bool RedRectDetector::model_roi_cut(Mat& img, Mat& roi, bool is_draw) {
    int blk_top = 0, blk_bottom = 0, x_left = 0, x_right = 0;

    // [1] 定位红色色块，获取完整上下边界
    if (!find_red_block(img, &blk_top, &blk_bottom, &x_left, &x_right)) {
        target_rect = Rect(0, 0, 0, 0);
        center_x = 0;
        center_y = 0;
        return false;
    }

    int block_w = x_right - x_left;                // 色块宽度（≈ 目标图片宽度）
    int block_h = blk_bottom - blk_top;            // 色块高度（透视下不固定）

    if (block_w <= 0 || block_h <= 0) {
        target_rect = Rect(0, 0, 0, 0);
        center_x = 0;
        center_y = 0;
        return false;
    }

    // [2] 动态正方形边长
    //     CROP_EDGE_PADDING  补偿色块底部透视渐变边缘被阈值截断的部分
    //     CROP_BOTTOM_RATIO  色块下方留白，使目标在裁剪框内整体偏上，与数据集布局一致
    int total_h       = block_w + block_h + CROP_EDGE_PADDING;
    int side          = max({total_h, block_w, MODEL_INPUT_WIDTH});
    int bottom_margin = (int)(side * CROP_BOTTOM_RATIO);
    side              = max(side, total_h + bottom_margin);

    // [3] 裁剪框坐标
    //     水平：以色块中心居中
    //     垂直：下边界 = 色块底部 + CROP_EDGE_PADDING + bottom_margin
    int cx = (x_left + x_right) / 2;
    int x1 = cx - side / 2;
    int x2 = x1 + side;
    int y2 = blk_bottom + CROP_EDGE_PADDING + bottom_margin;
    int y1 = y2 - side;

    // [4] 边界平移修正（保持正方形，优先平移而非截断）
    if (x1 < 0)        { x2 -= x1;              x1 = 0; }
    if (x2 > img.cols) { x1 -= (x2 - img.cols); x2 = img.cols; }
    if (y1 < 0)        { y2 -= y1;              y1 = 0; }
    if (y2 > img.rows) { y1 -= (y2 - img.rows); y2 = img.rows; }
    // 最终夹紧，防止平移后仍越界
    x1 = max(0, x1); x2 = min(img.cols, x2);
    y1 = max(0, y1); y2 = min(img.rows, y2);

    if (x2 <= x1 || y2 <= y1) {
        target_rect = Rect(0, 0, 0, 0);
        center_x = 0;
        center_y = 0;
        return false;
    }

    // [5] 有效区域校验
    //     无论是否通过校验，都更新中心坐标，供调用方区分"区域外"和"未找到"
    int crop_cx = (x1 + x2) / 2;
    int crop_cy = (y1 + y2) / 2;
    center_x = crop_cx;
    center_y = crop_cy;

    if (crop_cx < valid_x_min || crop_cx > valid_x_max ||
        crop_cy < valid_y_min || crop_cy > valid_y_max) {
        target_rect = Rect(0, 0, 0, 0);
        return false;
    }

    target_rect = Rect(x1, y1, x2 - x1, y2 - y1);

    // [6] 裁剪并按需缩放
    Mat cropped = img(target_rect);

    if (cropped.cols == MODEL_INPUT_WIDTH && cropped.rows == MODEL_INPUT_WIDTH) {
        roi = cropped.clone();
    } else {
        // INTER_AREA：缩小时取像素面积均值，无锯齿，嵌入式低分辨率场景首选
        int interp = (cropped.cols > MODEL_INPUT_WIDTH) ? INTER_AREA : INTER_LINEAR;
        resize(cropped, roi, Size(MODEL_INPUT_WIDTH, MODEL_INPUT_WIDTH), 0, 0, interp);
    }

    // 调试绘制：绿框=裁剪区域，红线=色块上边缘，蓝线=色块下边缘，黄框=有效区域边界
    if (is_draw) {
        rectangle(img, target_rect, Scalar(0, 255, 0), 1);
        line(img, Point(x_left, blk_top),    Point(x_right, blk_top),    Scalar(0, 0, 255), 1);
        line(img, Point(x_left, blk_bottom), Point(x_right, blk_bottom), Scalar(255, 0, 0), 1);
        rectangle(img,
                  Rect(valid_x_min, valid_y_min,
                       valid_x_max - valid_x_min,
                       valid_y_max - valid_y_min),
                  Scalar(0, 255, 255), 1);
    }

    return true;
}
