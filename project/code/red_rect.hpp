/**
 * @file    red_rect.hpp
 * @brief   基于红色色块定位的目标图像动态裁剪器
 *
 * 适用场景：
 *   赛道上的目标图片（白色背景）正下方紧接一块与图片等宽的红色长方形色块，
 *   以该色块作为定位锚点，向上裁剪出目标图片区域，送入模型推理。
 *
 * 依赖：OpenCV（仅使用 Mat / Rect / resize / rectangle / line）
 */

#ifndef __RED_RECT_HPP__
#define __RED_RECT_HPP__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

/** 模型输入边长（像素），同时也是裁剪区域的最小边长 */
#define MODEL_INPUT_WIDTH   80

/** 色块底部边缘补偿（像素）：补偿透视/模糊导致色块底部渐变行被阈值截断 */
#define CROP_EDGE_PADDING   4

/** 色块下方留白比例（0.0~1.0）：使目标在裁剪框内整体偏上，与数据集布局一致
 *  0.20 表示色块下方留出 side*20% 的空白，调大则目标更靠上 */
#define CROP_BOTTOM_RATIO   0.20f

/** 红色行判定阈值：该行红色像素数占扫描宽度的比例超过此值视为红色行
 *  实测辅助色块约 0.12~0.27，救护车条纹 strict=0，阈值取 0.10 留有余量 */
#define RED_ROW_RATIO       0.10f

/** 色块确认所需连续红色行数：防止单行噪声误触发 */
#define RED_CONFIRM_ROWS    3

class RedRectDetector {
public:
    /**
     * @brief 构造函数，必须在使用前提供图像分辨率
     * @param img_w       摄像头输出图像宽度（像素）
     * @param img_h       摄像头输出图像高度（像素）
     * @param r_threshold 红色判定阈值，R 通道占 RGB 总和的比例，默认 0.48
     *                    值越大要求越纯红，抗干扰越强但漏检率越高
     */
    RedRectDetector(int img_w, int img_h, float r_threshold = 0.48f);
    ~RedRectDetector();

    /**
     * @brief 设置目标有效区域（百分比边距，调用一次即可，运行期不变）
     *
     * 只有裁剪框中心落在有效区域内，model_roi_cut 才返回 true 并执行推理。
     * 用于过滤视野边缘的目标（如红色障碍物）以及确保目标足够靠近中心后再识别。
     *
     * 示例：set_valid_region(0.2f, 0.2f, 0.1f, 0.1f)
     *   → 有效区域为图像中央 60% 宽 × 80% 高的矩形
     *
     * @param left   左侧无效边距，占图像宽度的比例（0.0 ~ 1.0）
     * @param right  右侧无效边距，占图像宽度的比例
     * @param top    上侧无效边距，占图像高度的比例
     * @param bottom 下侧无效边距，占图像高度的比例
     */
    void set_valid_region(float left, float right, float top, float bottom);

    /** 裁剪并缩放后的输出图像，尺寸固定为 MODEL_INPUT_WIDTH × MODEL_INPUT_WIDTH */
    Mat  target_roi;

    /** 裁剪框在原图中的实际位置与尺寸；未检测到时为 Rect(0,0,0,0) */
    Rect target_rect;

    /** 裁剪框中心在原图中的 x 坐标；目标在有效区域外时也会更新，便于调试打印 */
    int  center_x;

    /** 裁剪框中心在原图中的 y 坐标；目标在有效区域外时也会更新，便于调试打印 */
    int  center_y;

    /** @brief 返回当前裁剪框位置，等价于直接访问 target_rect */
    Rect get_target_location() { return target_rect; }

    /**
     * @brief 主处理函数：定位红色色块 → 动态裁剪 → 有效区域校验 → 缩放输出
     *
     * @param img     输入原始图像（BGR，与摄像头输出格式一致）
     * @param roi     输出裁剪图像（MODEL_INPUT_WIDTH × MODEL_INPUT_WIDTH，BGR）
     * @param is_draw 是否在 img 上叠加调试图形（绿框=裁剪区，红线=色块上边缘，黄框=有效区域）
     * @return true   成功找到目标且中心在有效区域内，roi 已填充
     * @return false  未找到红色色块，或目标中心超出有效区域限制
     *                两种情况可通过 center_x/center_y 是否为 0 区分
     */
    bool model_roi_cut(Mat& img, Mat& roi, bool is_draw = false);

private:
    float r_thres;      ///< 红色判定阈值（构造时传入）
    int   img_width;    ///< 图像宽度（像素）
    int   img_height;   ///< 图像高度（像素）

    /// 有效区域像素边界，由 set_valid_region 预计算，避免每帧浮点运算
    int valid_x_min;
    int valid_x_max;
    int valid_y_min;
    int valid_y_max;

    /**
     * @brief 判断单个像素是否为红色
     *        条件：R/(R+G+B) > r_thres 且 R > 80 且总亮度 > 30
     *        双重过滤可同时排除暗色噪点和低饱和度伪红色
     */
    bool is_red(Vec3b color);

    /**
     * @brief 统计指定行 [x_start, x_end) 范围内红色像素的数量
     */
    int count_red_in_row(Mat& img, int y, int x_start, int x_end);

    /**
     * @brief 从图像下方向上扫描，定位红色色块的完整上下边界及左右边界
     *
     * 算法：逐行统计红色像素占比，连续 CONFIRM_ROWS 行超过 RED_ROW_RATIO
     * 则确认找到色块，继续向上扫描直到红色行中断，记录完整上下边界，
     * 在色块中间行左右各向内扫描确定精确左右边界。
     *
     * @param[out] block_top    色块最顶部红色行 y 坐标（目标图片下边界）
     * @param[out] block_bottom 色块最底部红色行 y 坐标（整体裁剪下基准）
     * @param[out] x_left       色块左边界 x 坐标
     * @param[out] x_right      色块右边界 x 坐标
     * @return true 找到色块，false 未找到
     */
    bool find_red_block(Mat& img, int* block_top, int* block_bottom,
                        int* x_left, int* x_right);
};

#endif // __RED_RECT_HPP__
