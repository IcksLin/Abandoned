#include "imgproc.hpp"
#include <cstdint>

/*--图片去畸映射表（由load_undistort_map初始化）--*/
cv::Mat ud_map_cv;
//去畸变后Mat
cv::Mat De_distortion_image;
//灰度图像帧
cv::Mat frame_gray;
// 灰度图像指针（爬线处理对象）
uint8_t* img_gray = nullptr;
// 二值化
uint8_t img_bin[IMG_W * IMG_H] = {0};
/*爬线函数爬到的信息**********************************/
uint8_t l_border[IMG_H];    // 左线数组
uint8_t r_border[IMG_H];    // 右线数组
uint8_t center_line[IMG_H]; // 中线数组
uint8_t lukuan[IMG_H];    // 路宽数组
//特征变量===============================
// 赛道搜索到的最高点（最远点）所在的行号
uint8_t top_point = 0; 
// 识别到的路宽突变点的行号（用于识别十字、丢线等）
uint8_t break_point = 0; 
// 符合“大路宽”条件的行数统计（用于识别起跑线）
uint16_t wide_track_count = 0; 
// 扫描过程中发现的最大路宽数值
uint8_t lukuan_max_x = 0; 
// 最大路宽所在的行号
uint8_t lukuan_max_y = 0; 
// 整个视野中最靠左的边界值（float 类型方便后续计算偏移）
float min_l_border = 0.0f; 
// 整个视野中最靠右的边界值
float max_r_border = 0.0f;
//
uint8_t left_lenth = 0; // 左侧边界长度
uint8_t right_lenth = 0; // 右侧边界长度
/*********************************************/

/*去畸变矩阵*/
float undistort_map_x[IMG_H][IMG_W];
float undistort_map_y[IMG_H][IMG_W];
//方向
float onto = 0.0f;


//去畸变矩阵初始化
void load_undistort_map(void){
    // 正式打开
    std ::ifstream fin("maps/undistort_map.txt");
    std ::string line;
    int line_count = 0;
    while (getline(fin, line) && line_count < IMG_H) {
        if (line.empty()) break; // 空行终止读取
        std ::istringstream iss(line); // 按空格拆分读取映射表
        for (int x = 0; x < IMG_W; x++) iss >> undistort_map_x[line_count][x];
        line_count++;
    }
    while (line.empty()) getline(fin, line);
    line_count = 0;
    while (getline(fin, line) && line_count < IMG_H) {
        std ::istringstream iss(line);
        for (int x = 0; x < IMG_W; x++) iss >> undistort_map_y[line_count][x];
        line_count++;
    }
    fin.close();

    ud_map_cv = cv::Mat(IMG_H, IMG_W, CV_16SC2); 
    int16_t* map1_ptr = (int16_t*)ud_map_cv.data; // 获取矩阵数据指针
    // 逐像素组合map_x和map_y
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            int idx = y * IMG_W + x; // 一维索引
            // 获取原始去畸变坐标
            int16_t orig_x = undistort_map_x[y][x];
            int16_t orig_y = undistort_map_y[y][x];
            
            // 对去畸变坐标翻转
            int16_t flip_x = IMG_W - 1 - orig_x; // 左右翻转
            int16_t flip_y = IMG_H - 1 - orig_y; // 上下翻转
            
            // 将翻转后的坐标写入映射表
            map1_ptr[2*idx] = flip_x;   
            map1_ptr[2*idx + 1] = flip_y; 
        }
    }

    std ::cout << "-图片去畸变文件加载成功-" << std ::endl;
}

//获取图像,将最终灰度图像存储在img_gray中
void get_image(){
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
}

//大津法二值化 返回阈值
uint8 get_otsu_thres(uint8 *img, int x0, int x1, int y0, int y1){
    /*灰度直方图参数*/
    uint16 histogram[256] = {0}; // 灰度直方图
    uint32 min_value, max_value;

    uint32 pix_amount = 0;   // 像素点总数
    uint32 pix_integral = 0; // 灰度值总数

    uint32 pix_back_amount = 0;   // 前景像素点总数
    uint32 pix_back_integral = 0; // 前景灰度值

    int32 pix_fore_amount = 0;   // 背景像素点总数
    int32 pix_fore_integral = 0; // 背景灰度值

    float omega_back, omega_fore, micro_back, micro_fore, sigma_beta, sigma; // 类间方差：浮点型更精确

    uint8 thres_result = 0;

    // 隔一行取一个值，更快
    for (int y = y0; y < y1; y +=2)
        for (int x = x0; x < x1; x +=2)
            histogram[IMG_AT(img, x, y)]++;

    for (min_value = 0; min_value < 256 && histogram[min_value] == 0; min_value++)
    {
        ; // 获取最小灰度的值
    }
    for (max_value = 255; max_value > min_value && histogram[min_value] == 0; max_value--)
    {
        ; // 获取最大灰度的值
    }

    if (max_value == min_value)
    {
        return ((uint8)(max_value)); // 图像中只有一个颜色
    }
    if (min_value + 1 == max_value)
    {
        return ((uint8)(min_value)); // 图像中只有二个颜色
    }

    /*OSTU大律法*/
    pix_integral = 0;
    for (uint16 j = (uint16)min_value; j <= max_value; j++)
    {
        pix_amount += histogram[j];       // 像素总数
        pix_integral += histogram[j] * j; // 灰度值总数
    }
    sigma_beta = -1;

    for (uint16 j = (uint16)min_value; j < max_value; j++)
    {
        pix_back_amount = pix_back_amount + histogram[j];                                        // 前景像素点数
        pix_fore_amount = pix_amount - pix_back_amount;                                          // 背景像素点数
        omega_back = (float)pix_back_amount / pix_amount;                                        // 前景像素百分比
        omega_fore = (float)pix_fore_amount / pix_amount;                                        // 背景像素百分比
        pix_back_integral += histogram[j] * j;                                                   // 前景灰度值
        pix_fore_integral = pix_integral - pix_back_integral;                                    // 背景灰度值
        micro_back = (float)pix_back_integral / pix_back_amount;                                 // 前景灰度百分比
        micro_fore = (float)pix_fore_integral / pix_fore_amount;                                 // 背景灰度百分比
        sigma = omega_back * omega_fore * (micro_back - micro_fore) * (micro_back - micro_fore); // 计算类间方差
        if (sigma > sigma_beta)                                                                  // 遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            sigma_beta = sigma;
            thres_result = (uint8)j;
        }
    }
    return thres_result; // 返回最佳阈值;
}

//对出入图像依照最佳阈值进行二值化处理复制到输出图像中(会标记图像边框)
void image_binarization(uint8_t* input_img, uint8_t* output_image, uint8_t best_threshold) {
    int border_width = 1;  // 边框宽度，可以根据需要调整
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            int index = y * IMG_W + x;
            // 判断是否为边框区域
            if (y < border_width || y >= IMG_H - border_width ||
                x < border_width || x >= IMG_W - border_width) {
                output_image[index] = BORDER;
            } else {
                // 非边框区域，正常二值化
                output_image[index] = (input_img[index] >= best_threshold) ? WHITE : BLACK;
            }
        }
    }
}

/**
 * @brief 赛道边界搜索与特征提取 (避开轮胎区域)
 * @param img_ptr 传入的 frame_gray.ptr(0) 指针
 * @param image_thereshold 阈值
 */

SearchResult processTrackSearch(uint8_t* img_ptr, uint8_t image_thereshold) {
    // 1. 初始化变量
    top_point = 2;
    break_point = 0;
    wide_track_count = 0;
    
    // --- 新增：长度初始化 ---
    left_lenth = 0;
    right_lenth = 0;
    
    int current_left_x = -1, current_left_y = -1;
    int current_right_x = -1, current_right_y = -1;
    bool left_seed_ok = false;
    bool right_seed_ok = false;

    // 2. 寻找起始种子点 (保持不变)
    for (int j = IMG_H - 1; j > IMG_H / 2; --j) {
        if (!left_seed_ok) {
            for (int x = FIND_LEFT_START_X; x > 1; --x) {
                if (IMG_AT(img_ptr, x, j) >= image_thereshold) {
                    current_left_x = x; current_left_y = j;
                    left_seed_ok = true; break;
                }
            }
        }
        if (!right_seed_ok) {
            for (int x = FIND_RIGHT_START_X; x < IMG_W - 1; ++x) {
                if (IMG_AT(img_ptr, x, j) >= image_thereshold) {
                    current_right_x = x; current_right_y = j;
                    right_seed_ok = true; break;
                }
            }
        }
        if (left_seed_ok && right_seed_ok) break;
    }

    if (!left_seed_ok && !right_seed_ok) return SEARCH_FAIL;

    // 3. 向上双边爬取
    bool left_broken = !left_seed_ok;
    bool right_broken = !right_seed_ok;
    static const int8_t L_template[9][2] = {{0,-1},{-1,-1},{1,-1},{-2,-1},{2,-1},{-3,-1},{3,-1},{-4,-1},{4,-1}};
    static const int8_t R_template[9][2] = {{0,-1},{1,-1},{-1,-1},{2,-1},{-2,-1},{3,-1},{-3,-1},{4,-1},{-4,-1}};

    for (int j = IMG_H - 1; j > 2; --j) {
        // --- 左边爬取逻辑 ---
        if (!left_broken && j <= current_left_y) {
            bool found = false;
            for (int i = 0; i < 9; ++i) {
                int nx = current_left_x + L_template[i][0];
                int ny = j - 1;
                if (nx > 1 && nx < IMG_W - 1 && IMG_AT(img_ptr, nx, ny) >= image_thereshold) {
                    l_border[j] = 2;
                    for (int k = nx; k > 1; --k) {
                        if (IMG_AT(img_ptr, k, j) <= image_thereshold) { l_border[j] = k; break; }
                    }
                    current_left_x = nx; 
                    found = true; 
                    // --- 新增：只要找到了点，左线长度就增加 ---
                    left_lenth++; 
                    break;
                }
            }
            if (!found) left_broken = true;
        } else if (left_broken) {
            l_border[j] = (j < IMG_H - 1) ? l_border[j + 1] : 2;
        }

        // --- 右边爬取逻辑 ---
        if (!right_broken && j <= current_right_y) {
            bool found = false;
            for (int i = 0; i < 9; ++i) {
                int nx = current_right_x + R_template[i][0];
                int ny = j - 1;
                if (nx > 1 && nx < IMG_W - 1 && IMG_AT(img_ptr, nx, ny) >= image_thereshold) {
                    r_border[j] = IMG_W - 2;
                    for (int k = nx; k < IMG_W - 1; ++k) {
                        if (IMG_AT(img_ptr, k, j) <= image_thereshold) { r_border[j] = k; break; }
                    }
                    current_right_x = nx; 
                    found = true; 
                    // --- 新增：只要找到了点，右线长度就增加 ---
                    right_lenth++; 
                    break;
                }
            }
            if (!found) right_broken = true;
        } else if (right_broken) {
            r_border[j] = (j < IMG_H - 1) ? r_border[j + 1] : IMG_W - 2;
        }
        
        if (!left_broken || !right_broken) top_point = j;
    }

    // --- 4. 滤波逻辑 (保持不变) ---
    uint8_t l_tmp[IMG_H], r_tmp[IMG_H];
    blur_line(l_border, IMG_H, l_tmp, 5);
    blur_line(r_border, IMG_H, r_tmp, 5);

    // --- 5. 合成最终结果 (保持不变) ---
    for (int j = IMG_H - 1; j >= top_point; --j) {
        l_border[j] = l_tmp[j];
        r_border[j] = r_tmp[j];
        center_line[j] = (l_border[j] + r_border[j]) / 2;
        lukuan[j] = r_border[j] - l_border[j];
        
        if (break_point == 0 && j < IMG_H - 3) {
            if (lukuan[j+2] - lukuan[j] >= 10) {
                break_point = j;
            }
        }
    }

    return SEARCH_OK;
}

/**
 * @brief 对赛道线数组进行一维三角滤波 (平滑处理)
 * @param line_in  输入数组 (如 l_border)
 * @param num      数组长度 (IMG_H)
 * @param line_out 输出平滑后的数组
 * @param kernel_size 滤波核大小 (建议取 3, 5, 7 等奇数)
 */
void blur_line(uint8_t line_in[], int num, uint8_t line_out[], int kernel_size) {
    int half = kernel_size / 2;
    float total_weight = (float)(half + 1) * (half + 1);

    for (int i = 0; i < num; i++) {
        float sum_x = 0;

        for (int j = -half; j <= half; j++) {
            // 计算当前点的权重: 距离中心越近权重越高 (1 2 ... half+1 ... 2 1)
            int weight = half + 1 - abs(j);
            
            // 索引限幅处理，防止越界 (clip 逻辑)
            int index = i + j;
            if (index < 0) index = 0;
            if (index >= num) index = num - 1;

            sum_x += (float)line_in[index] * weight;
        }

        // 归一化并回写，加上 0.5f 用于四舍五入
        line_out[i] = (uint8_t)(sum_x / total_weight + 0.5f);
    }
}

/**
 * @brief 计算加权前瞻偏移角度
 * @return float 最终的加权偏移角度（弧度或度，根据需求调整）
 */
float calculate_weighted_offset_angle() {
    float total_weighted_angle = 0.0f;
    float total_weight = 0.0f;
    
    // 图像中心参考点
    const int origin_x = IMG_W / 2;
    const int origin_y = IMG_H - 1; 

    // 确定计算范围：从底部往上爬，跳过前 15 个点
    // 扫描区间：[IMG_H - 1 - 15] 到 [top_point]
    int start_y = IMG_H - 1 - 15;
    
    if (start_y <= top_point) return 0.0f; // 有效点不足

    for (int j = start_y; j >= top_point; j--) {
        // 1. 计算当前点相对于底部中心的坐标差
        float dx = (float)(center_line[j] - origin_x);
        float dy = (float)(origin_y - j); // 向上为正，dy 为正数

        if (dy == 0) continue;

        // 2. 计算当前点的偏角 (弧度值)
        // atan2f(dx, dy) 返回 [-pi, pi]，直行时接近 0
        float current_angle = atan2f(dx, dy);

        // 3. 计算权重：越近权重越大
        // 权重设计：(当前行到远端的距离) / (总搜索区间长度)
        // 这样 j 越大（越近），weight 越大
        float weight = (float)(j - top_point + 1);
        
        // 如果想让权重随距离更剧烈地衰减，可以使用平方：
        // float weight = powf((float)(j - top_point + 1), 2);

        // 4. 累加
        total_weighted_angle += current_angle * weight;
        total_weight += weight;
    }

    // 5. 输出加权平均值
    if (total_weight < 1e-5f) return 0.0f;

    float final_angle = total_weighted_angle / total_weight;

    // 如果需要转化为角度制：
    // final_angle = final_angle * 180.0f / 3.1415926f;

    return final_angle;
}

//边线状态判断函数
void judge_border_state(uint8_t )
{
    
}

/**************************补线函数*****************************/
//------------------------------------------------------------------------------------------------------------------
//  @brief    输入起始点，终点坐标，补一条宽度为2的黑线,补的就是一条线，需要重新扫线
//------------------------------------------------------------------------------------------------------------------
void Draw_black_Line(int startX, int startY, int endX, int endY,uint8_t*input_img)
{
    int i,x,y;
    int start=0,end=0;
    if(startX>=IMG_W-1)//限幅处理
        startX=IMG_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=IMG_H-1)
        startY=IMG_H-1;
    else if(startY<=0)
        startY=0;
    if(endX>=IMG_W-1)
        endX=IMG_W-1;
    else if(endX<=0)
        endX=0;
    if(endY>=IMG_H-1)
        endY=IMG_H-1;
    else if(endY<=0)
        endY=0;
    if(startX==endX)//一条竖线
    {
        if (startY > endY)//互换
        {
            start=endY;
            end=startY;
        }
        for (i = start; i <= end; i++)
        {
            if(i<=1)
                i=1;
            IMG_AT(input_img, i, startX)=0;
            IMG_AT(input_img, i-1, startX)=0;
        }
    }
    else if(startY == endY)//补一条横线
    {
        if (startX > endX)//互换
        {
            start=endX;
            end=startX;
        }
        for (i = start; i <= end; i++)
        {
            if(startY<=1)
                startY=1;
            IMG_AT(input_img, startY, i)=0;
            IMG_AT(input_img, startY-1, i)=0;
        }
    }
    else //上面两个是水平，竖直特殊情况，下面是常见情况
    {
        if(startY>endY)//起始点矫正
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=IMG_W-1)
                x=IMG_W-1;
            else if (x<=1)
                x=1;
            IMG_AT(input_img, i, x)=0;
            IMG_AT(input_img, i, x-1)=0;
        }
        if(startX>endX)
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)//横向补线，保证每一列都有黑点
        {

            y =(int)(startY+(endY-startY)*(i-startX)/(endX-startX));//两点式变形
            if(y>=IMG_H-1)
                y=IMG_H-1;
            else if (y<=0)
                y=0;
            IMG_AT(input_img, y, i)=0;
        }
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief    斜率补黑线
//------------------------------------------------------------------------------------------------------------------

void K_Draw_Line(float k, int startX, int startY,int endY,uint8_t*input_img)
{
    int endX=0;

    if(startX>=IMG_W-1)//限幅处理
        startX=IMG_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=IMG_H-1)
        startY=IMG_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=IMG_H-1)
        endY=IMG_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    Draw_black_Line(startX,startY,endX,endY,input_img);
}
//------------------------------------------------------------------------------------------------------------------
//  @brief    补一条实线
//------------------------------------------------------------------------------------------------------------------
void Add_Line(int x1, int y1, int x2, int y2, unsigned char a, uint8_t*input_img) {
    const int max_x = IMG_W - 1;
    const int max_y = IMG_H - 1;

    // 裁剪坐标到有效范围
    x1 = (x1 < 0) ? 0 : (x1 > max_x) ? max_x : x1;
    y1 = (y1 < 0) ? 0 : (y1 > max_y) ? max_y : y1;
    x2 = (x2 < 0) ? 0 : (x2 > max_x) ? max_x : x2;
    y2 = (y2 < 0) ? 0 : (y2 > max_y) ? max_y : y2;

    // 确保y1 <= y2 (C语言手动交换)
    if (y1 > y2) {
        int temp = x1; x1 = x2; x2 = temp;
        temp = y1; y1 = y2; y2 = temp;
    }

    // 处理水平线段
    if (y1 == y2) {
        int hx = (x1 + x2) / 2;
        hx = (hx < 0) ? 0 : (hx > max_x) ? max_x : hx;
        switch(a) {
            case 0: center_line[y1] = hx; break;
            case 1: l_border[y1] = hx; break;
            case 2: r_border[y1] = hx; break;
        }
        return;
    }

    int dx = x2 - x1;
    int dy = y2 - y1;

    // 处理垂直线段
    if (dx == 0) {
        int hx = (x1 < 0) ? 0 : (x1 > max_x) ? max_x : x1;
        for (int i = y1; i <= y2; ++i) {
            switch(a) {
                case 0: center_line[i] = hx; break;
                case 1: l_border[i] = hx; break;
                case 2: r_border[i] = hx; break;
            }
        }
        return;
    }

    // 处理一般情况
    int step_x = 1;
    int abs_dx = dx;
    if (dx < 0) {
        step_x = -1;
        abs_dx = -abs_dx;
    }

    int step = abs_dx / dy;
    int remainder = abs_dx % dy;
    int current = x1;
    int error = 0;

    for (int i = y1; i <= y2; ++i) {
        int hx = current;
        hx = (hx < 0) ? 0 : (hx > max_x) ? max_x : hx;
        switch(a) {
            case 0: center_line[i] = hx; break;
            case 1: l_border[i] = hx; break;
            case 2: r_border[i] = hx; break;
        }

        error += remainder;
        if (error >= dy) {
            current += step_x * (step + 1);
            error -= dy;
        } else {
            current += step_x * step;
        }
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief    斜率补实线
//------------------------------------------------------------------------------------------------------------------
void K_Draw_Line_center_line(float k, int startX, int startY,int endY,uint8_t*input_img)
{
    int endX=0;

    if(startX>=IMG_W-1)//限幅处理
        startX=IMG_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=IMG_H-1)
        startY=IMG_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=IMG_H-1)
        endY=IMG_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    Add_Line(startX,startY,endX,endY,0,input_img);
}