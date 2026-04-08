#include "imgproc.hpp"
using namespace cv;

/*决策变量******************/
float onto = 0.0f;            // 最终处理方向，已限制幅度在-30~30.0f之间
float angle_compensation = 0; // 方向补偿量(静态最中间偏差)
int middle_line_length = 0;   // 中线长度
float max_angle = 0.0f;       // 最大角点值,用于调试
/******************图像变量*/
//图像
Mat frame_color;                // 用于处理的图像帧
Mat frame_gray;                 // 灰度图像帧
Mat frame_bin;                  // 二值图像帧

uint8_t* img_gray = nullptr;              // 灰度图像指针
AimPoint_TypeDef aim_point; 


/*--图片去畸--*/
Mat ud_map_cv;
/*去畸变矩阵*/
float undistort_map_x[IMG_H][IMG_W];
float undistort_map_y[IMG_H][IMG_W];
/*图像变量******************/

/**************边线处理变量*/
/*参数变量*/
uint8_t all_block_size = 7;
uint8_t adapt_clip = 7;         // 自适应迷宫巡线偏差
uint8_t start_thre = 130;
float avg_angle;
float map_x[IMG_H][IMG_W];      // 原图点(x,y)透视后x坐标
float map_y[IMG_H][IMG_W];      // 原图点(x,y)透视后y坐标
// 方向数组
int dir_front[4][2]      = {{0,  -1},{1, 0},{0, 1},{-1, 0}};
int dir_frontleft[4][2]  = {{-1, -1},{1,-1},{1, 1},{-1, 1}};
int dir_frontright[4][2] = {{1,  -1},{1, 1},{-1,1},{-1,-1}};

/*边线变量*/
//采样前后边线长度
int Lline_num, Rline_num;
int sampled_Lline_num, sampled_Rline_num;
int Mline_num;
// 边线
int Lline[POINTS_MAX_LEN][2], Rline[POINTS_MAX_LEN][2];                      // 原始
int ud_Lline[POINTS_MAX_LEN][2], ud_Rline[POINTS_MAX_LEN][2];                // 去畸变
float per_Lline[POINTS_MAX_LEN][2], per_Rline[POINTS_MAX_LEN][2];            // 透视
float blurred_Lline[POINTS_MAX_LEN][2], blurred_Rline[POINTS_MAX_LEN][2];    // 滤波
float sampled_Lline[POINTS_MAX_LEN][2], sampled_Rline[POINTS_MAX_LEN][2];    // 等距采样
float L2Mline[POINTS_MAX_LEN][2], R2Mline[POINTS_MAX_LEN][2];                // 左右得到的中线
float (*Mline)[2] = L2Mline;                                                 // 最终中线
float dangle_Lline[POINTS_MAX_LEN], dangle_Rline[POINTS_MAX_LEN];            // 局部角度变化率
// 非极大抑制
float nms_Lline,nms_Rline;          // 角点值
int nms_Lline_idx,nms_Rline_idx;    // 索引

cv::Mat M = (cv::Mat_<float>(3, 3) <<
-1.7987879419009005,-4.6836929815029515,224.950105042017,
-0.02597752463054235,-8.582940180382508,304.59513546798064,
-0.0003848522167487723,-0.05828171785955768,1.0);

cv::Mat M_Reverse = (cv::Mat_<float>(3, 3) <<
-0.5567704942449283,0.5116792735001106,-30.609436463231717,
0.005540545406144685,0.10396670497412587,-32.91409885685576,
0.00010863814521852267,0.006256279068760795,-0.9300703326531625);
/*边线处理变量**************/

//巡线决策机
Tracking_Decision_Machine_TypeDef tracking_decision_machine = {
    .max_L_angle = 0.0f,        
    .max_R_angle = 0.0f,        
    .max_angle = 0.0f,          

    .state = 0,  
    .element_processing_flage = 0,            
    .state_time_locking = STATE_TIME_LOCKING,    

    .longest_side = 0,          
    .left_length = 0,           
    .right_length = 0,          

    .target_boundary = 0   //0左，1右     
};

Circle_Tracking_Machine_TypeDef cricle_decision_machine = {
    .state = 0,         //初始状态
    .state_locking = 0, //初始状态未上锁
    .side = 0           //初始
};


// 完整的一个边线处理
void line_process(uint8_t height_start, uint8_t height_min){

    // 左右巡线
    search_Lline(height_start, height_min);
    search_Rline(height_start, height_min);
    // 点集透视
    perspective_transform_points(Lline,Lline_num,per_Lline);
    // 点集滤波
    blur_points(per_Lline,Lline_num,blurred_Lline);
    // 点集采样
    resample_points(blurred_Lline,Lline_num,sampled_Lline,&sampled_Lline_num);
    // 点集角度
    local_angle_points(sampled_Lline,sampled_Lline_num,dangle_Lline);
    // 极大角度
    nms_angle(dangle_Lline,sampled_Lline_num,&nms_Lline,&nms_Lline_idx);
    // track_leftline();

    // 右边线处理
    perspective_transform_points(Rline,Rline_num,per_Rline);
    blur_points(per_Rline,Rline_num,blurred_Rline);
    resample_points(blurred_Rline,Rline_num,sampled_Rline,&sampled_Rline_num);
    local_angle_points(sampled_Rline,sampled_Rline_num,dangle_Rline);
    nms_angle(dangle_Rline,sampled_Rline_num,&nms_Rline,&nms_Rline_idx);
    // track_rightline();
    
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

// 保存映射
void save_per_map(void) {
    // 创建写入文本对象 - 明确使用std命名空间
    std::ofstream fout("maps/per_map.txt");
    
    // 检查文件是否成功打开
    if (!fout.is_open()) {
        std::cerr << "错误：无法打开文件 maps/per_map.txt" << std::endl;
        return;
    }
    
    // 计算透视映射
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {   
            float U = M.at<float>(0, 0) * x + M.at<float>(0, 1) * y + M.at<float>(0, 2);
            float V = M.at<float>(1, 0) * x + M.at<float>(1, 1) * y + M.at<float>(1, 2);
            float W = M.at<float>(2, 0) * x + M.at<float>(2, 1) * y + M.at<float>(2, 2);
            
            // 避免除以零
            if (W != 0.0f) {
                map_x[y][x] = U / W;
                map_y[y][x] = V / W;
            } else {
                map_x[y][x] = 0.0f;
                map_y[y][x] = 0.0f;
            }
        }
    }
    
    // 设置四位小数精度
    fout << std::fixed << std::setprecision(4);
    
    // 写入map_x
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            fout << map_x[y][x];
            if (x != IMG_W - 1) fout << " ";
        }
        fout << std::endl;
    }
    
    // 写入空行分隔
    fout << std::endl;
    
    // 写入map_y
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            fout << map_y[y][x];
            if (x != IMG_W - 1) fout << " ";
        }
        fout << std::endl;
    }
    
    // 关闭文件
    fout.close();
    
    // std::cout << "透视映射已保存到 maps/per_map.txt" << std::endl;
}

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

    ud_map_cv = Mat(IMG_H, IMG_W, CV_16SC2); 
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

// /*******************************显示函数*********************************/
/**
 * @brief 对点应用透视矩阵
 */
void point_per(const cv::Mat& M, float x, float y, int& x_out, int& y_out) {
    const float M11 = M.at<float>(0, 0), M12 = M.at<float>(0, 1), M13 = M.at<float>(0, 2); 
    const float M21 = M.at<float>(1, 0), M22 = M.at<float>(1, 1), M23 = M.at<float>(1, 2); 
    const float M31 = M.at<float>(2, 0), M32 = M.at<float>(2, 1), M33 = M.at<float>(2, 2);

    float X = M11 * x + M12 * y + M13 * 1.0f;
    float Y = M21 * x + M22 * y + M23 * 1.0f;
    float W = M31 * x + M32 * y + M33 * 1.0f;
    x_out = (int)(X/W+0.5);
    y_out = (int)(Y/W+0.5);
}
/*---------------------------边线处理-----------------------------*/

// 透视变换
void perspective_transform_points(int pts_in[][2],int num,float pts_out[][2]){
    for (int i=0;i<num;i++){
        int x = pts_in[i][0];
        int y = pts_in[i][1];
        pts_out[i][0] = map_x[y][x];
        pts_out[i][1] = map_y[y][x];
    }
}

// 左手迷宫巡线
void findline_lefthand_adaptive(int x, int y){
    int half = all_block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < Lline_num && half < x && x < IMG_W - half - 1 && TRACK_HEIGHT_MAX < y && y < IMG_H - half - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += IMG_AT(img_gray, x + dx, y + dy);
            }
        }
        local_thres /= all_block_size * all_block_size;
        local_thres -= adapt_clip;

        uint8_t front_value = IMG_AT(img_gray, x + dir_front[dir][0], y + dir_front[dir][1]);
        uint8_t frontleft_value = IMG_AT(img_gray, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            Lline[step][0] = x;
            Lline[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            Lline[step][0] = x;
            Lline[step][1] = y;
            step++;
            turn = 0;
        }
        
        // uvc.frame_rgb.at<Vec3b>(y,x) = Vec3b(0,0,255);
    }
    Lline_num = step;
}

// 右手迷宫巡线
void findline_righthand_adaptive(int x, int y){
    int half = all_block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < Rline_num && half < x && x < IMG_W - half - 1 && TRACK_HEIGHT_MAX < y && y < IMG_H - half - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += IMG_AT(img_gray, x + dx, y + dy);
            }
        }
        local_thres /= all_block_size * all_block_size;
        local_thres -= adapt_clip;

        uint8_t front_value = IMG_AT(img_gray, x + dir_front[dir][0], y + dir_front[dir][1]);
        uint8_t frontright_value = IMG_AT(img_gray, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            Rline[step][0] = x;
            Rline[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            Rline[step][0] = x;
            Rline[step][1] = y;
            step++;
            turn = 0;
        }
        // uvc.frame_rgb.at<Vec3b>(y,x) = Vec3b(0,0,255);
    }
    Rline_num = step;
}

// 寻找左起始点并巡线
// 修正版：使用有符号 int，加入边界检查，避免 wrap-around
void search_Lline(int height_start, int height_min)
{
    int begin_x, begin_y;
    bool found_flag = false;

    // 从左侧开始
    begin_y = height_start - all_block_size / 2 - 2;  // 可能为负，下面会裁剪
    begin_x = all_block_size / 2 + 1;

    // 裁剪起点到合法范围
    if (begin_y < 0) begin_y = 0;
    if (begin_y >= IMG_H) begin_y = IMG_H - 1;
    if (begin_x < 0) begin_x = 0;
    if (begin_x >= IMG_W) begin_x = IMG_W - 1;

    // 如果起始点为黑点（边线外）则按 x 轴向内寻找白点
    auto safe_IMG_AT = [&](int x, int y)->int {
        if (x < 0 || x >= (int)IMG_W || y < 0 || y >= (int)IMG_H) return 0; // treat out-of-bounds as black
        return IMG_AT(img_gray, x, y);
    };

    if (safe_IMG_AT(begin_x, begin_y) <= start_thre)
    {
        int right_limit = IMG_W / 2 - CAR_IMGAGE_W / 2;
        if (right_limit < 0) right_limit = 0;
        // iterate x to the right, but ensure we don't overflow
        for (int x = begin_x; x < right_limit; /* increment inside */)
        {
            int i = 0;
            if (safe_IMG_AT(x, begin_y) > start_thre)
            {
                // 检查后续 CHECK_DIS 个像素是否为噪点, 且不要越界
                bool is_noise = false;
                for (i = 1; i <= CHECK_DIS; ++i)
                {
                    int xi = x + i;
                    if (xi >= (int)IMG_W) { is_noise = true; break; } // 到边界视作噪点/结束
                    if (safe_IMG_AT(xi, begin_y) <= start_thre)
                    {
                        // 是噪点，跳过到 x + i + 1（但确保不越界）
                        x = x + i + 1;
                        if (x >= (int)IMG_W) x = IMG_W - 1;
                        is_noise = true;
                        break;
                    }
                }
                if (is_noise)
                {
                    // 继续主循环（x 已更新）
                    if (x >= right_limit) break;
                    continue;
                }
            }

            // 如果 i > CHECK_DIS 表示不是噪点（即连续 CHECK_DIS 点都为白）
            // 但是上面我们 only set i upto CHECK_DIS; check condition:
            // If last loop ran fully (i == CHECK_DIS + 1) -> not noise
            // Simpler: re-evaluate region to decide
            // 直接判定当前点及周围是否满足为真实边线
            bool accept = false;
            // 检查当前点后续 CHECK_DIS 是否全为白 (作为简单判定)
            bool ok = true;
            for (int k = 0; k <= CHECK_DIS; ++k)
            {
                int xi = x + k;
                if (xi >= (int)IMG_W || safe_IMG_AT(xi, begin_y) <= start_thre) { ok = false; break; }
            }
            if (ok) accept = true;

            if (accept)
            {
                begin_x = x;
                found_flag = true;
                break;
            }
            else
            {
                // 否则移动到下一个像素继续
                x++;
                if (x >= right_limit) break;
            }
        }
    }
    else
    {
        // 起点为白点，则按 y 轴向上寻找黑点
        for (int y = begin_y; y > height_min && y >= 0; --y)
        {
            int i = 0;
            if (safe_IMG_AT(begin_x, y) <= start_thre)
            {
                bool is_noise = false;
                for (i = 1; i <= CHECK_DIS; ++i)
                {
                    int yi = y - i;
                    if (yi < 0) { is_noise = true; break; }
                    if (safe_IMG_AT(begin_x, yi) > start_thre)
                    {
                        // 噪点，继续从 y - (i+1) 处搜索
                        y = y - (i + 1);
                        if (y < height_min) y = height_min;
                        is_noise = true;
                        break;
                    }
                }
                if (is_noise) continue;
            }

            // 不是噪点，判定为起点
            // 简单判定：当前点连续 CHECK_DIS 皆为黑
            bool ok = true;
            for (int k = 0; k <= CHECK_DIS; ++k)
            {
                int yi = y - k;
                if (yi < 0 || safe_IMG_AT(begin_x, yi) > start_thre) { ok = false; break; }
            }
            if (ok)
            {
                begin_y = y;
                found_flag = true;
                break;
            }
            // 否则继续上移
        }
    }

    if (found_flag)
    {
        Lline_num = POINTS_MAX_LEN;
        findline_lefthand_adaptive(begin_x, begin_y);
    }
    else
    {
        Lline_num = 0;
    }
}

// 寻找右起始点并巡线
// 修正版右侧搜索：同样用 int，加入边界检查
void search_Rline(int height_start, int height_min)
{
    int begin_x, begin_y;
    bool found_flag = false;

    begin_y = height_start - all_block_size / 2 - 2;
    begin_x = IMG_W - all_block_size / 2 - 2;

    if (begin_y < 0) begin_y = 0;
    if (begin_y >= IMG_H) begin_y = IMG_H - 1;
    if (begin_x < 0) begin_x = 0;
    if (begin_x >= IMG_W) begin_x = IMG_W - 1;

    auto safe_IMG_AT = [&](int x, int y)->int {
        if (x < 0 || x >= (int)IMG_W || y < 0 || y >= (int)IMG_H) return 0;
        return IMG_AT(img_gray, x, y);
    };

    if (safe_IMG_AT(begin_x, begin_y) <= start_thre)
    {

        int left_limit = IMG_W / 2 + CAR_IMGAGE_W / 2;
        if (left_limit < 0) left_limit = 0;
        // iterate x to the left
        for (int x = begin_x; x > left_limit; /* decrement inside */)
        {
            int i = 0;
            if (safe_IMG_AT(x, begin_y) > start_thre)
            {
                // 检查左侧 CHECK_DIS 个像素
                bool is_noise = false;
                for (i = 1; i <= CHECK_DIS; ++i)
                {
                    int xi = x - i;
                    if (xi < 0) { is_noise = true; break; } // 到边界当噪点处理
                    if (safe_IMG_AT(xi, begin_y) <= start_thre)
                    {
                        // 噪点，移动到 x - (i+1)
                        x = x - (i + 1);
                        if (x < 0) x = 0;
                        is_noise = true;
                        break;
                    }
                }
                if (is_noise)
                {
                    if (x <= left_limit) break;
                    continue;
                }
            }

            // 简单判定：检查当前及其左侧 CHECK_DIS 是否全为��� -> 接受
            bool ok = true;
            for (int k = 0; k <= CHECK_DIS; ++k)
            {
                int xi = x - k;
                if (xi < 0 || safe_IMG_AT(xi, begin_y) <= start_thre) { ok = false; break; }
            }

            if (ok)
            {
                begin_x = x;
                found_flag = true;
                break;
            }
            else
            {
                // 继续左移
                x--;
                if (x <= left_limit) break;
            }
        }
    }
    else
    {
        for (int y = begin_y; y > height_min && y >= 0; --y)
        {
            int i = 0;
            if (safe_IMG_AT(begin_x, y) <= start_thre)
            {
                bool is_noise = false;
                for (i = 1; i <= CHECK_DIS; ++i)
                {
                    int yi = y - i;
                    if (yi < 0) { is_noise = true; break; }
                    if (safe_IMG_AT(begin_x, yi) > start_thre)
                    {
                        y = y - (i + 1);
                        if (y < height_min) y = height_min;
                        is_noise = true;
                        break;
                    }
                }
                if (is_noise) continue;
            }

            bool ok = true;
            for (int k = 0; k <= CHECK_DIS; ++k)
            {
                int yi = y - k;
                if (yi < 0 || safe_IMG_AT(begin_x, yi) > start_thre) { ok = false; break; }
            }
            if (ok)
            {
                begin_y = y;
                found_flag = true;
                break;
            }
        }
    }

    if (found_flag)
    {
        Rline_num = POINTS_MAX_LEN;
        findline_righthand_adaptive(begin_x, begin_y);
    }
    else
    {
        Rline_num = 0;
    }
}

// 对点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2]) {
    int half = all_block_size / 2;
    for (int i = 0; i < num; i++) {
        // 默认置点为(0,0)
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            //取点为 i-n/2...i-1 i i+2...i+n/2
            pts_out[i][0] += 1.0*pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += 1.0*pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        // 权重分布为 1 2 ...half+1... 2 1
        // 总权重为(1+2+...+half)*2+(half+1)=(half+1)half+(half+1)=(half+1)(half+1)
        pts_out[i][0] /= (half + 1) * (half + 1);
        pts_out[i][1] /= (half + 1) * (half + 1);
    }
}

// 点集等距采样,使走过的每段折线段的距离为固定距离
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2){
    float remain = 0.f; //两点间剩余要采样的长度
    float dist = sampled_dist*M2PIX; // 固定距离
    int len = 0;
    *num2 = POINTS_MAX_LEN;
    //由于用到i+1个in点,这里遍历到num1-1,且总点数要小于最大长度num2
    for(int i=0; i<num1-1 && len < *num2; i++){
        // 当前点坐标
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        // 下一个点和当前点坐标差值
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        // 两点距离,即初始要采样的距离
        float ds = sqrt(dx*dx+dy*dy);
        // 单位方向向量化
        dx /= ds; // dx=cosα
        dy /= ds; // dy=sinα

        // remain不足ds时进行插值
        while (remain < ds && len < *num2){
            // x,y按方向向量移动remain
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            ds -= remain;  // 总采样距离ds减少
            remain = dist; // remain固定为采样距离dist
        }
        remain -= ds;      // 为下次采样开头做准备
    }
    *num2 = len;
}

// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[]){
    // 首个点和最后一个点无法计算变化率,直接置为0
    angle_out[0] = 0;
    angle_out[num-1] = 0;
    // 对每个点,计算pin(i-angle_idx),pin(i+angle_idx)方向向量的夹角
    for (int i = 1; i < num-1; i++) {
         //计算pini和pin(i-dist)间的距离
        float dx0 = pts_in[i][0] - pts_in[clip(i - angle_idx, 0, num - 1)][0];
        float dy0 = pts_in[i][1] - pts_in[clip(i - angle_idx, 0, num - 1)][1];
        float ds0 = sqrtf(dx0 * dx0 + dy0 * dy0);

        //计算pin(i+dist)和pini间的距离
        float dx1 = pts_in[clip(i + angle_idx, 0, num - 1)][0] - pts_in[i][0];
        float dy1 = pts_in[clip(i + angle_idx, 0, num - 1)][1] - pts_in[i][1];
        float ds1 = sqrtf(dx1 * dx1 + dy1 * dy1);

        //计算两个方向向量的角度,此处也是将dx视作cos,dy视作sin
        dx0 /= ds0;
        dy0 /= ds0;
        dx1 /= ds1;
        dy1 /= ds1;

        //atan2f(y/x)严格计算夹角,这里实际上通过atan2f(sinθ/cosθ)=atan2f(tanθ)=θ来计算
        //其中二维向量可以写作三维:l0=(cos0,sin0,0),l1=(cos1,sin1,0),|l0|=|l1|=1
        //求叉积l0×l1=(0,0,cos0*sin1-cos1*sin0),取z分量就是sinθ(正负代表旋转方向)
        //求点积l0·l1=cos0*cos1+sin0*sin1,就是cosθ(正负代表左右象限)
        angle_out[i] = fabs(atan2f(dx0 * dy1 - dx1 * dy0, dx0 * dx1 + dy0 * dy1) / PI * 180);
    }
}

// 角度变化率非极大抑制,返回最大角及索引
void nms_angle(float angle_in[], int num, float *angle_max, int *idx) {
    *angle_max = 0;
    //末端角点无效
    num--;
    //从第2个点开始
    for (uint8_t i=1;i<num;i++){
        if (angle_in[i] > *angle_max) {
            *angle_max = angle_in[i];
            *idx = i;
        }
    }
}

// 从左边线跟踪中线
void track_leftline(float dist) {
    for (int i = 0; i < sampled_Lline_num; i++) {
        // 求解±approx_idx内
        float dx = sampled_Lline[clip(i + approx_idx, 0, sampled_Lline_num - 1)][0] 
                 - sampled_Lline[clip(i - approx_idx, 0, sampled_Lline_num - 1)][0];
        float dy = sampled_Lline[clip(i + approx_idx, 0, sampled_Lline_num - 1)][1]
                 - sampled_Lline[clip(i - approx_idx, 0, sampled_Lline_num - 1)][1];
        float ds = sqrt(dx * dx + dy * dy);
        dx /= ds;
        dy /= ds;
        // dist就是得到的像素距离下的半赛道长
        // 对(dx,dy)即(cosθ,sinθ)向左旋转90°
        // 得到(cos(θ+π/2),sin(θ+π/2))=(-sinθ,cosθ)=(-dy,dx)
        L2Mline[i][0] = sampled_Lline[i][0] - dy * dist;
        L2Mline[i][1] = sampled_Lline[i][1] + dx * dist;
    }
}

// 从右边线跟踪中线
void track_rightline(float dist) {
    for (int i = 0; i < sampled_Rline_num; i++) {
        float dx = sampled_Rline[clip(i + approx_idx, 0, sampled_Rline_num - 1)][0] 
                 - sampled_Rline[clip(i - approx_idx, 0, sampled_Rline_num - 1)][0];
        float dy = sampled_Rline[clip(i + approx_idx, 0, sampled_Rline_num - 1)][1] 
                 - sampled_Rline[clip(i - approx_idx, 0, sampled_Rline_num - 1)][1];
        float ds = sqrt(dx * dx + dy * dy);
        dx /= ds;
        dy /= ds;
        //(cos(θ-π/2),sin(θ-π/2))=(sinθ,-cosθ)=(dy,-dx)
        R2Mline[i][0] = sampled_Rline[i][0] + dy * dist;
        R2Mline[i][1] = sampled_Rline[i][1] - dx * dist;
    }
}

/**
 * @brief 十字补线函数（固定数组长度版）
 * @param pts_in 边线点数组
 * @param num 指向数组当前有效点数的指针（作为上限参考，不增加其值）
 * @param corner_index 角点索引（补线的起点）
 * @param dist 补线步长
 */
void supplement_line(float pts_in[][2], int* num, int corner_index, float dist) {
    if (corner_index <= 1 || corner_index >= *num) return;

    float avg_angle = 0;
    for (int i = 0; i < corner_index - 1; i++) {
        float dx = pts_in[i + 1][0] - pts_in[i][0];
        float dy = pts_in[i + 1][1] - pts_in[i][1];
        avg_angle += -atan2f(dy, dx);
    }
    avg_angle /= (corner_index - 1);

    float start_x = pts_in[corner_index][0];
    float start_y = pts_in[corner_index][1];
    float abs_angle = fabs(avg_angle);

    // 垂直趋势判定：45° ~ 135° (PI/4 ~ 3PI/4)
    if (abs_angle > PI / 4 && abs_angle < 3 * PI / 4) {
        int current_idx = corner_index;

        // 循环直到 current_idx 达到数组的最大索引 (POINTS_MAX_LEN - 1)
        while (current_idx < (POINTS_MAX_LEN - 1)) {
            start_x += dist * (float)cos(avg_angle);
            start_y -= dist * (float)sin(avg_angle);

            current_idx++; // 移动到下一个位置
            pts_in[current_idx][0] = start_x;
            pts_in[current_idx][1] = start_y;
        }
        // 更新有效点数为填满后的总长度
        *num = POINTS_MAX_LEN;
        // --- 修改部分结束 ---
    } 
    else {
        // 水平趋势：从拐点坐标向下拉线，覆盖水平边线信息
        for (int i = 0; i < corner_index; i++)
        {
            pts_in[i][0] = pts_in[corner_index][0];
            pts_in[i][1] = pts_in[corner_index][1] + dist * (corner_index - i);
        }
    }
}

/*---------------------------角度计算-----------------------------*/
// /**
//  * @brief 计算加权前瞻偏移角度 (适配 Mline[i][2] 数据结构)
//  * @param Mline 中线点集指针，Mline[i][0]为x, Mline[i][1]为y
//  * @param num 中线数组的有效点数
//  * @return float 最终的加权偏移角度（度）
//  */
// float calculate_weighted_offset_angle(float (*Mline)[2], int num) {
//     const int target_samples = 15;
//     const int start_idx = 3;
//     static float last_angle = 0.0f; // 静态变量保持记忆
    
//     // 情况 A: 点数不足，直接返回上一帧
//     if (num <= start_idx) return last_angle;

//     float total_weighted_angle = 0.0f;
//     float total_weight = 0.0f;
//     const int origin_x = IMG_W / 2;
//     const int origin_y = IMG_H - 1;

//     int available_points = num - start_idx;
//     float step = (available_points > target_samples) ? 
//                  (float)available_points / (float)target_samples : 1.0f;

//     for (int k = 0; k < target_samples; k++) {
//         int i = start_idx + (int)(k * step);
//         if (i >= num) break;

//         float dx = Mline[i][0] - (float)origin_x;
//         float dy = (float)origin_y - Mline[i][1];

//         if (dy <= 0) continue;

//         float current_angle = atan2f(dx, dy);
//         float weight = (float)(target_samples - k); 
        
//         total_weighted_angle += current_angle * weight;
//         total_weight += weight;

//         if (step == 1.0f && i == num - 1) break;
//     }

//     // 情况 B: 循环完发现没有有效权重（计算失败），也要返回上一帧
//     // 而不是返回 0.0f
//     if (total_weight < 1e-5f) return last_angle; 

//     // 计算平均弧度并转换
//     float final_angle = (total_weighted_angle / total_weight) * 180.0f / 3.14159265f;
//     final_angle += angle_compensation;

//     // 限幅
//     if (final_angle > 30.0f)  final_angle = 30.0f;
//     if (final_angle < -30.0f) final_angle = -30.0f;

//     // 更新记忆
//     last_angle = final_angle;

//     return final_angle;
// }

/**
 * @brief 计算加权前瞻偏移角度 (适配数组权重分配，压制近端震荡)
 * @param Mline 中线点集指针，Mline[i][0]为x, Mline[i][1]为y
 * @param num 中线数组的有效点数
 * @return float 最终的加权偏移角度（度）
 */
float calculate_weighted_offset_angle(float (*Mline)[2], int num) {
    const int target_samples = 15;
    const int start_idx = 3;
    static float last_angle = 0.0f; // 静态变量保持记忆
    
    // --- 权重分配数组 (由近及远) ---
    // 索引 0-4 为近处，5-9 为中处，10-14 为远处
    // 你可以根据实测调整这些数值：数值越大，该点对转向的影响越大
    const float weights[15] = {
        0.20f,  0.32f,  0.7f,  1.00f,  1.1f,  // 近处：低权重，减少抖动
        1.20f,  1.20f,  1.3f,  1.30f,  1.3f,  // 中间：过渡区
        1.2f,   1.2f,   1.0f,  1.0f,   1.0f   // 远处：高权重，提供前瞻预判
    };

    // 情况 A: 点数不足，直接返回上一帧
    if (num <= start_idx) return last_angle;

    float total_weighted_angle = 0.0f;
    float total_weight = 0.0f;
    const int origin_x = IMG_W / 2;
    const int origin_y = IMG_H - 1;

    // 计算采样步长
    int available_points = num - start_idx;
    float step = (available_points > target_samples) ? 
                 (float)available_points / (float)target_samples : 1.0f;

    for (int k = 0; k < target_samples; k++) {
        int i = start_idx + (int)(k * step);
        if (i >= num) break;

        float dx = Mline[i][0] - (float)origin_x;
        float dy = (float)origin_y - Mline[i][1];

        // --- 已根据要求删除 dy_safe 阻尼补偿逻辑，直接判断 dy ---
        if (dy <= 0) continue;

        // 直接使用原始 dy 计算当前点连线角度（弧度）
        float current_angle = atan2f(dx, dy);
        
        // 从权重数组中直接取值
        float weight = weights[k]; 
        
        total_weighted_angle += current_angle * weight;
        total_weight += weight;

        // 步长为1且点数较少时的退出保护
        if (step == 1.0f && i == num - 1) break;
    }

    // 情况 B: 循环完发现没有有效权重（计算失败），返回上一帧
    if (total_weight < 1e-5f) return last_angle; 

    // 计算加权平均角度并从弧度转为角度
    float final_angle = (total_weighted_angle / total_weight) * 180.0f / 3.14159265f;
    
    // 加上外部偏置补偿（如零位校准）
    final_angle += angle_compensation;

    // 限幅控制
    if (final_angle > 30.0f)  final_angle = 30.0f;
    if (final_angle < -30.0f) final_angle = -30.0f;

    // 更新记忆，用于下一帧丢线或点数不足时使用
    last_angle = final_angle;

    return final_angle;
}


//去畸变后Mat
cv::Mat De_distortion_image;

// 一次图像处理
void image_proc() {   
    cv::Mat frame_resized;
    cv::resize(uvc.frame_mjpg, frame_resized, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);

    static cv::Mat frame_gray_small;
    cv::cvtColor(frame_resized, frame_gray_small, cv::COLOR_BGR2GRAY);
    img_gray = reinterpret_cast<uint8_t*>(frame_gray_small.ptr(0));

    start_thre = get_otsu_thres(img_gray, 0, 160, TRACK_HEIGHT_MAX, 120);
    line_process(120, 120 / 2);

    element_status();
    no_element_process();
    crossing_process();
    circle_process();
    auto_tracking();

    max_angle = std::max(nms_Lline, nms_Rline);
    onto = calculate_weighted_offset_angle(Mline, middle_line_length);

    // if (udp.is_enable()) {
    //     // A. 发送原始 320x160 彩色图像 (用于确认识别结果)
    //     udp.send_image(uvc.frame_mjpg); 

    //     // B. 准备并打包轨迹点 (适配 160x120 尺度)
    //     // 定义符合上位机格式的缓冲区
    //     static uint8_t L_buf[120][2];
    //     static uint8_t R_buf[120][2];
    //     static uint8_t M_buf[120][2];

    //     // 清零缓冲区防止旧数据干扰
    //     memset(L_buf, 0, sizeof(L_buf));
    //     memset(R_buf, 0, sizeof(R_buf));
    //     memset(M_buf, 0, sizeof(M_buf));

    //     // 转换边线坐标到传输格式 (uint8_t[y][x])
    //     for (int i = 0; i < 120; ++i) {
    //         // 左边线
    //         if (i < sampled_Lline_num) {
    //             L_buf[i][0] = (uint8_t)std::clamp((int)sampled_Lline[i][0], 0, 159);
    //             L_buf[i][1] = (uint8_t)std::clamp((int)sampled_Lline[i][1], 0, 119);
    //         }
    //         // 右边线
    //         if (i < sampled_Rline_num) {
    //             R_buf[i][0] = (uint8_t)std::clamp((int)sampled_Rline[i][0], 0, 159);
    //             R_buf[i][1] = (uint8_t)std::clamp((int)sampled_Rline[i][1], 0, 119);
    //         }
    //         // 中线
    //         if (i < middle_line_length) {
    //             M_buf[i][0] = (uint8_t)std::clamp((int)Mline[i][0], 0, 159);
    //             M_buf[i][1] = (uint8_t)std::clamp((int)Mline[i][1], 0, 119);
    //         }
    //     }

    //     // C. 发送轨迹点数据包 (根据你 udp_sender 的实现调用)
    //     // 如果你的上位机支持单独发送数组，则按顺序发送。
    //     // 下面是示例：将三条线作为一个整体 Data 包发送
    //     struct {
    //         uint8_t L[120][2];
    //         uint8_t R[120][2];
    //         uint8_t M[120][2];
    //     } track_packet;
        
    //     memcpy(track_packet.L, L_buf, sizeof(L_buf));
    //     memcpy(track_packet.R, R_buf, sizeof(R_buf));
    //     memcpy(track_packet.M, M_buf, sizeof(M_buf));

    //     udp.send_data(&track_packet, sizeof(track_packet));
    // }
    // printf("onto:   %f     ,middle_line_length: %d    \r",onto,middle_line_length);

    // 调试要看状态机请解注释这行
    // printf("state:%d ,element_state:%d ,left:%f  ,right:%f  \r   ",tracking_decision_machine.state,cricle_decision_machine.state,nms_Lline, nms_Rline);

}
//状态机初始化
void tracking_decision_machine_init(){
    tracking_decision_machine.max_angle = 0;
    tracking_decision_machine.state = 0;
    tracking_decision_machine.element_processing_flage = 0; //元素处理标志位，0未处理，1已处理
    tracking_decision_machine.max_L_angle = 0;
    tracking_decision_machine.max_R_angle = 0;
    tracking_decision_machine.right_length = 0;
    tracking_decision_machine.left_length = 0;
    tracking_decision_machine.state_time_locking = STATE_TIME_LOCKING;
    tracking_decision_machine.target_boundary = 0;
    tracking_decision_machine.longest_side = 0;

}

// 元素函数，圆环
// 直角最大角点弯曲率不会超过100,但赛道大圆环平均110以上，小圆环100以上
//由于小圆环个个拐点可能不太好和直角区分，所以设置状态机，当检测到超过100度的点时判断可能出现圆环
//此时同时检查两路线的最大拐点弯曲率，如果全部都大于60～80中的某一个值，则为十字路口误判，如果是单边最大值，记录该变，进入对应的左右圆环状态，准备切换边线寻线
//总状态机函数，0无元素，1十字，2左圆环，3右圆环
/**
 * @brief 元素识别状态机
 * @details 负责检测十字、左圆环、右圆环，并引入仅针对圆环的出环后冷却保护
 */
void element_status() {
    // --- 1. 冷却时间管理（仅针对圆环生效） ---
    if (tracking_decision_machine.is_cooling) {
        tracking_decision_machine.cooldown_timer.stop(); // 获取当前时刻点
        tracking_decision_machine.cooldown_threshold_sec = 5;
        if (tracking_decision_machine.cooldown_timer.elapsed_sec() >= tracking_decision_machine.cooldown_threshold_sec) {
            tracking_decision_machine.is_cooling = false; // 冷却时间到，允许再次识别圆环
        }
    }

    // 只有在非处理阶段，才允许进入检测
    if (!tracking_decision_machine.element_processing_flage) 
    {
        // --- 2. 十字检测 (不受冷却限制) ---
        // if(sampled_Lline_num > LOST_LINE && sampled_Rline_num > LOST_LINE) {
            // if(nms_Lline > CORNER_ANGLE_THRE && nms_Rline > CORNER_ANGLE_THRE) {
            //     tracking_decision_machine.state = 1; // 十字路口状态
            //     tracking_decision_machine.element_processing_flage = 1; 
            //     return;
            // }
        // }

        // --- 3. 圆环检测 (受冷却限制) ---
        if (!tracking_decision_machine.is_cooling) // 仅在非冷却状态下检测圆环
        {
            if(std::max(nms_Lline, nms_Rline) > CIRCLE_ANGLE_THRE) {
                if (sampled_Rline_num >= LOST_LINE && sampled_Lline_num >= LOST_LINE) 
                {
                    if (nms_Lline > CORNER_ANGLE_THRE && nms_Rline > CORNER_ANGLE_THRE) 
                    {
                        // 兜底逻辑：如果两边都很大，依然判定为十字
                        tracking_decision_machine.state = 1; 
                        tracking_decision_machine.element_processing_flage = 1;
                        return;
                    }
                    else if(nms_Lline > CIRCLE_ANGLE_THRE && nms_Rline < STRAIGHT_ROAD_THRE) {
                        tracking_decision_machine.state = 2; // 左圆环状态
                        tracking_decision_machine.element_processing_flage = 1;
                        return;
                    }
                    else if(nms_Rline > CIRCLE_ANGLE_THRE && nms_Lline < STRAIGHT_ROAD_THRE) {
                        tracking_decision_machine.state = 3; // 右圆环状态
                        tracking_decision_machine.element_processing_flage = 1;
                        return;
                    }
                }
            }
        }
        
        if(nms_Lline > CORNER_ANGLE_THRE || nms_Rline > CORNER_ANGLE_THRE) {
            tracking_decision_machine.state = 1; // 十字路口状态
            tracking_decision_machine.element_processing_flage = 1; 
            return;
        }

        //无元素检测到，状态机回到无元素状态
        tracking_decision_machine.state = 0; 
    }
}
//无元素状态处理函数
void no_element_process(){
    if (tracking_decision_machine.state == 0)
    {
        tracking_decision_machine.element_processing_flage = 0; //元素处理标志位常态失活
        //选择最长边线跟踪中线
        if(sampled_Lline_num>sampled_Rline_num){
            tracking_decision_machine.target_boundary = 0; //左边线最长
        }else if(sampled_Rline_num>sampled_Lline_num){
            tracking_decision_machine.target_boundary = 1; //右边线最长
        }
    }
}

//十字路口处理函数
void crossing_process(){
    if (tracking_decision_machine.state == 1)
    {
        if (nms_Lline>CORNER_ANGLE_THRE)
        {   //左补线
            supplement_line(sampled_Lline,&sampled_Lline_num,nms_Lline_idx,sampled_dist*M2PIX);
        }
        if (nms_Rline>CORNER_ANGLE_THRE)
        {   //右补线
            supplement_line(sampled_Rline,&sampled_Rline_num,nms_Rline_idx,sampled_dist*M2PIX);
        }
        //选择最长边线跟踪中线
        if(sampled_Lline_num>sampled_Rline_num){
            tracking_decision_machine.target_boundary = 0; //左边线最长
        }else if(sampled_Rline_num>sampled_Lline_num){
            tracking_decision_machine.target_boundary = 1; //右边线最长
        }
        //若相等，保持原有状态
        if((nms_Lline<CORNER_ANGLE_THRE&&nms_Rline<CORNER_ANGLE_THRE)&&sampled_Rline_num>=LOST_LINE&&sampled_Lline_num>=LOST_LINE){
        //有一边小于直线阈值且两边均未丢线，失活元素处理状态
        tracking_decision_machine.element_processing_flage = 0; 
        }
        return ;  
    }
    
 
    
}

//圆环处理函数
void circle_process(){
    if(tracking_decision_machine.state == 2||tracking_decision_machine.state == 3){
        //cricle_decision_machine
        //圆环状态机部分
        if(tracking_decision_machine.state == 2){
            cricle_decision_machine.side = 2;
            // cricle_decision_machine.start_angle = ahrs
        }
        else if(tracking_decision_machine.state == 3){
            cricle_decision_machine.side = 3;
        }
        
        //左圆环处理===============================================================================
        if(cricle_decision_machine.side == 2){
            // 状态 0-1：识别到左圆环特征
            if(!cricle_decision_machine.state_locking && cricle_decision_machine.state == 0){
                if(nms_Lline > CORNER_ANGLE_THRE && nms_Rline < CORNER_ANGLE_THRE){
                    cricle_decision_machine.state = 1;
                }
            }

            // 状态 1-2：准备入环阶段
            if(cricle_decision_machine.state == 1){
                tracking_decision_machine.target_boundary = 0; // 巡左线
                
                if (nms_Lline > CORNER_ANGLE_THRE) {   
                    supplement_line(sampled_Lline, &sampled_Lline_num, nms_Lline_idx, sampled_dist * M2PIX);
                    cricle_decision_machine.state_locking = 0; 
                }

                if(sampled_Lline_num < LOST_LINE){
                    if(nms_Lline < CORNER_ANGLE_THRE){
                        if(!cricle_decision_machine.state_locking){
                            cricle_decision_machine.state = 2;
                        }
                        tracking_decision_machine.target_boundary = 0;
                        cricle_decision_machine.state_locking = 1;
                    }
                }
            }

            // 状态 2-3：环岛内行驶阶段
            if(cricle_decision_machine.state == 2){
                tracking_decision_machine.target_boundary = 0; // 持续巡左
                
                if (sampled_Rline_num < LOST_LINE) {
                    cricle_decision_machine.state_locking = 0; // 右边丢线解锁
                }   

                if(sampled_Rline_num >= LOST_LINE && !cricle_decision_machine.state_locking){
                    if(nms_Rline < CORNER_ANGLE_THRE){
                        if(!cricle_decision_machine.state_locking){
                            cricle_decision_machine.state = 3;
                            cricle_decision_machine.state_locking = 1;
                        }
                    }
                }
                left_path_adjust(); // 持续矫正左侧路径
            }

            // 状态 3-4：出环阶段
            if(cricle_decision_machine.state == 3){
                tracking_decision_machine.target_boundary = 0; // 出左环阶段通常保持巡左或切回寻优
                
                if (nms_Rline > CORNER_ANGLE_THRE) {   
                    supplement_line(sampled_Rline, &sampled_Rline_num, nms_Rline_idx, sampled_dist * M2PIX);
                    cricle_decision_machine.state_locking = 0;
                }
                
                // 对标右圆环：当右边线平均弯曲度展现为直道判定出环岛
                if(sampled_Rline_num > LOST_LINE && nms_Rline < STRAIGHT_ROAD_THRE){
                    if(!cricle_decision_machine.state_locking) cricle_decision_machine.state = 4;
                }
                // right_path_adjust(0); 
            }

            // 状态 4-0：重置与释放
            if(cricle_decision_machine.state == 4 || cricle_decision_machine.state == 0){
                // --- 触发冷却保护机制 ---
                // 只有当真正完成圆环处理（flage为1）并进入重置态时，启动5秒计时器
                if(tracking_decision_machine.element_processing_flage == 1){
                    tracking_decision_machine.is_cooling = true;
                    tracking_decision_machine.cooldown_timer.start(); // 记录当前时间戳，开始5秒冷却
                }

                cricle_decision_machine.state = 0;
                cricle_decision_machine.state_locking = 0;
                // 关键：释放全局处理标志位，允许 element_status 重新检测新元素
                tracking_decision_machine.element_processing_flage = 0;
                tracking_decision_machine.target_boundary = 0; // 初始化回左边界巡线
            }
        }

        //右圆环处理===============================================================================
        else if(cricle_decision_machine.side == 3){
            //拐点弯曲度判断,锁失活才切换
            //状态 0-1：识别到右圆环特征（右边有拐点，左边直）
            if(!cricle_decision_machine.state_locking && cricle_decision_machine.state == 0){
                if(nms_Rline > CORNER_ANGLE_THRE && nms_Lline < CORNER_ANGLE_THRE){
                    cricle_decision_machine.state = 1;
                }
            }

            //状态 1-2：准备入环阶段
            if(cricle_decision_machine.state == 1){
                tracking_decision_machine.target_boundary = 1;
                
                // 如果出现右拐点，进行右补线以稳定入环路径
                if (nms_Rline > CORNER_ANGLE_THRE)
                {   
                    supplement_line(sampled_Rline, &sampled_Rline_num, nms_Rline_idx, sampled_dist * M2PIX);
                    cricle_decision_machine.state_locking = 0;//解锁
                }

                if(sampled_Rline_num < LOST_LINE){
                    // 当右侧丢线，说明车头已对准环内，切换到状态 2
                    if(nms_Rline < CORNER_ANGLE_THRE){
                        if(!cricle_decision_machine.state_locking){
                            cricle_decision_machine.state = 2;//修改状态时先看锁
                        }
                        
                        // 强制巡线边切换为右边（环岛内圆线），如果出现右圆环，后面的自动巡线机会自动切换到右环，否则寻有线测
                        tracking_decision_machine.target_boundary = 1;
                        // 上锁，防止在环内误触发状态 0
                        cricle_decision_machine.state_locking = 1;
                    }
                }
                //右边环岛赛道矫正
                // right_path_adjust();
            }

            //状态 2-3：环岛内行驶阶段
            if(cricle_decision_machine.state == 2){
                // 持续巡右边线
                tracking_decision_machine.target_boundary = 1;
                
                // 检测左边线（对侧）是否丢线，若丢线则解锁，准备找出口拐点
                if (sampled_Lline_num < LOST_LINE)
                {
                    cricle_decision_machine.state_locking = 0;
                }   
                
                // 当左边重新找到线（出口特征）且无大弯曲度时，准备切换
                if(sampled_Lline_num >= LOST_LINE && !cricle_decision_machine.state_locking){
                    if(nms_Lline < CORNER_ANGLE_THRE){
                        if(!cricle_decision_machine.state_locking){
                            cricle_decision_machine.state = 3;
                            cricle_decision_machine.state_locking = 1;
                        }
                    }
                }
                // 右环岛赛道矫正
                right_path_adjust();
            }

            //状态 3-4：出环阶段
            if(cricle_decision_machine.state == 3){
                tracking_decision_machine.target_boundary = 1;
                
                // 如果左边出现拐点，进行左补线并解锁
                if (nms_Lline > CORNER_ANGLE_THRE)
                {  
                    supplement_line(sampled_Lline, &sampled_Lline_num, nms_Lline_idx, sampled_dist * M2PIX);
                    cricle_decision_machine.state_locking = 0;
                }
                
                // 当左边线平均弯曲度展现为直道判定出环岛
                if(sampled_Lline_num > LOST_LINE && nms_Lline < STRAIGHT_ROAD_THRE){
                    if(!cricle_decision_machine.state_locking) cricle_decision_machine.state = 4;
                }
            }

            //状态 4-0：重置与释放
            if(cricle_decision_machine.state == 4 || cricle_decision_machine.state == 0){
                // --- 触发冷却保护机制 ---
                if(tracking_decision_machine.element_processing_flage == 1){
                    tracking_decision_machine.is_cooling = true;
                    tracking_decision_machine.cooldown_timer.start(); // 开始5秒冷却倒计时
                }

                cricle_decision_machine.state = 0;
                cricle_decision_machine.state_locking = 0;
                // 关键：释放全局处理标志位，允许 element_status 重新检测新元素
                tracking_decision_machine.element_processing_flage = 0;
                tracking_decision_machine.target_boundary = 1;
            }
        }

    }else{
        cricle_decision_machine.state = 0;
        cricle_decision_machine.state_locking = 0;
    }
}

//巡线函数，根据tracking_decision_machine.target_boundary优先巡线，若发现线丢失，则自动切换线
void auto_tracking(){
    // 0 代表优先巡左线
    if(tracking_decision_machine.target_boundary == 0 && sampled_Lline_num > 5){
        track_leftline();
        Mline = L2Mline; 
        middle_line_length = sampled_Lline_num;
    }
    // 1 代表优先巡右线
    else if(tracking_decision_machine.target_boundary == 1 && sampled_Rline_num > 5){
        track_rightline();
        Mline = R2Mline;
        middle_line_length = sampled_Rline_num;
    }
    // 自动兜底逻辑：当指定的目标线丢失（小于等于5个点）时，自动选择较长的一边
    else if(sampled_Lline_num > sampled_Rline_num && sampled_Lline_num > 5){
        track_leftline();
        Mline = L2Mline;
        middle_line_length = sampled_Lline_num;
    }
    else if(sampled_Rline_num > 5)
    {
        track_rightline();
        Mline = R2Mline;
        middle_line_length = sampled_Rline_num;
    }
    else
    {
        // 极端情况：双线全部丢失
        middle_line_length = 0;
    }
}

/**
 * @details 左环岛巡线轨迹矫正（直接操作 L2Mline）
 * @note 逻辑：在左边线引导的中线中，找到最靠右（x最大）的点，
 * 将该点以下（0 到 max_index）的所有点 x 坐标强制修正为该最大值，
 * 防止左环岛入库凸起导致小车误判向右大幅度转向。
 */
void left_path_adjust(void) {
    // 1. 安全检查
    if (sampled_Lline_num <= 0) return;

    float max_x = sampled_Lline[0][0]; // 初始化为第一个点（底部）的x
    int max_index = 0;

    // 2. 遍历左中线，寻找最靠右（x坐标最大）的点
    // 当左边出现环岛凸起时，中线会被挤向右侧，x值增大
    for (int i = 0; i < sampled_Lline_num; i++) {
        if (sampled_Lline[i][0] > max_x) { 
            max_x = sampled_Lline[i][0];
            max_index = i;
        }
    }

    // 3. 轨迹矫正：将底部到极值点的路径拉直
    // 强制令小车近端的引导线垂直向上，抵消环岛带来的转向斜率
    for (int i = 0; i < max_index; i++) {
        sampled_Lline[i][0] = max_x;
    }
}

/**
 * @details 右环岛巡线轨迹矫正（直接操作 R2Mline）
 * @note 逻辑：在右边线引导的中线中，找到最靠左（x最小）的点，
 * 将该点以下（0 到 min_index）的所有点 x 坐标强制修正为该最小值，
 * 防止环岛入库凸起诱导小车过早转向。
 */
void right_path_adjust(void) {
    // 1. 安全检查：确保有足够的点进行遍历
    if (sampled_Rline_num <= 0) return;

    float min_x = sampled_Rline[0][0]; // 初始化为第一个点的x（通常是图像底部）
    int min_index = 0;

    // 2. 遍历右中线，寻找最靠左（x坐标最小）的点
    // 在右环岛干扰下，这个点是中线向左偏移最厉害的位置
    for (int i = 0; i < sampled_Rline_num; i++) {
        if (sampled_Rline[i][0] < min_x) { 
            min_x = sampled_Rline[i][0];
            min_index = i;
        }
    }

    // 3. 轨迹矫正：将从底部到极值点的所有 x 坐标替换为 min_x
    // 这样可以让小车底部的引导斜率变直，不会一头撞向环岛
    for (int i = 0; i < min_index; i++) {
        sampled_Rline[i][0] = min_x;
    }
}